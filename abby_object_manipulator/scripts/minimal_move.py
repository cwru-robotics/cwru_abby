#! /usr/bin/python

import roslib; roslib.load_manifest('abby_object_manipulator')
import rospy

from abby_gripper.srv import *
import actionlib
from arm_navigation_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from kinematics_msgs.msg import *
from kinematics_msgs.srv import *
from object_manipulation_msgs.msg import *
from object_manipulation_msgs.srv import *
from Queue import Queue
from sensor_msgs.msg import JointState
import tf
import tf.transformations as transformations

import copy
import math
from math import sin, cos, pi
import numpy
from threading import Lock

class MinimalMover:
    #TODO Get these from the parameter server
    plannerServiceName = "/ompl_planning/plan_kinematic_path"
    armGroupName = "irb_120"
    armActionName = 'move_irb_120'
    toolLinkName = "gripper_body"
    frameID = "/irb_120_base_link"
    preGraspDistance = .1 #meters
    gripperFingerLength = 0.115 #meters
    gripperOpenWidth = 0.08 #0.065 #meters
    gripperClosedWidth = 0.046 #meters
    touchLinks = gripperCollisionNames = ("gripper_body", "gripper_jaw_1", "gripper_jaw_2")
    attachLinkName = "gripper_jaw_1"
    
    def __init__(self):
        self._tasks = Queue()
        self._moveArm = actionlib.SimpleActionClient(self.armActionName, MoveArmAction)
        self._moveArm.wait_for_server()
        rospy.loginfo('Minimal mover connected to arm action server.')
        self._gripperClient = rospy.ServiceProxy('abby_gripper/gripper', gripper)
        rospy.loginfo('Minimal mover connected to gripper service.')
        self._joint_state_lock = Lock()
        with self._joint_state_lock:
            self._joint_states = JointState()
        self._joint_state_listener = rospy.Subscriber('/joint_states', JointState, self._jointStateCallback)
        #Wait until we get all the joint states, since they aren't aggregated
        while len(self._joint_states.name) <  14:
            rospy.sleep(0.1)
        rospy.loginfo('Minimal mover got joint states.')
        self._tf_listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                self._tf_listener.waitForTransform("/base_link","/irb_120_base_link", rospy.Time(0), rospy.Duration(2.0))
            except (tf.Exception):
                rospy.loginfo('Minimal mover still waiting on a transform')
            else:
                break;
        self._tf_broadcaster = tf.TransformBroadcaster()
        rospy.loginfo('Minimal Move Object Initialized')
    
    '''Adds received joint information to internally stored joint information'''
    def _jointStateCallback(self, joints):
        with self._joint_state_lock:
            self._joint_states.header = joints.header
            for msg_index, joint_name in enumerate(joints.name):
                if not joint_name in self._joint_states.name:
                    self._joint_states.name.append(joint_name)
                    self._joint_states.position.append(joints.position[msg_index])
                    self._joint_states.velocity.append(0)#append(joints.velocity[msg_index])
                    self._joint_states.effort.append(0)#append(joints.effort[msg_index])
                else:
                    index = self._joint_states.name.index(joint_name)
                    self._joint_states.position[index] = joints.position[msg_index]
                    self._joint_states.velocity[index] = 0#joints.velocity[msg_index]
                    self._joint_states.effort[index] = 0#joints.effort[msg_index]
    
    def clearTasks(self):
        rospy.loginfo('Clearing task list.')
        with self._tasks.mutex:
            self._tasks.queue.clear()
    
    def run(self):
        self.clearTasks()
        self._moveArm.cancel_all_goals()
        
        #Add open gripper task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_OPEN))
        
        #Add a goal at the stow position
        stowGoal = self._makeStow()
        if stowGoal == False:
            return False
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE,stowGoal))
        
        #Add a goal at the stow position
        nextGoal = self._makeMove()
        if nextGoal == False:
            return False
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE,nextGoal))
        
        #Add close gripper task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_CLOSE))
        
        #Start task execution
        self.runNextTask()
    
    def _moveArmDoneCB(self, state, result):
        '''If the arm successfully moved to the position, move on to the next task.'''
        if result.error_code.val == result.error_code.SUCCESS:
            rospy.loginfo("Arm motion success!")
            self._tasks.task_done()
            rospy.sleep(rospy.Duration(1,0))
            self.runNextTask()
        else:
            rospy.logerr("Arm motion failed! Error code:%d",result.error_code.val)
            self.clearTasks()
    
    def _moveArmActiveCB(self):
        pass
    
    def _moveArmFeedbackCB(self, feedback):
        pass
    
    def runNextTask(self):
        '''Process the next task on the queue and send it to the appropriate action server.
        Warning: This function is recursive for non-move tasks. A lot of non-move tasks in a row
        might eat up a lot of stack memory. I need to fix this, but I'm lazy.'''
        task = self._tasks.get(True)
        
        if task.type == task.TYPE_OPEN:
            rospy.loginfo('Opening the gripper')
            self._gripperClient(gripperRequest.OPEN)
            self._tasks.task_done()
            self.runNextTask()
        elif task.type == task.TYPE_CLOSE:
            rospy.loginfo('Closing the gripper')
            self._gripperClient(gripperRequest.CLOSE)
            self._tasks.task_done()
            self.runNextTask()
        elif task.type == task.TYPE_MOVE:
            rospy.loginfo('Waiting for arm action server')
            self._moveArm.wait_for_server()
            rospy.loginfo('Moving the arm')
            print task.move_goal
            self._moveArm.send_goal(task.move_goal, self._moveArmDoneCB, self._moveArmActiveCB, self._moveArmFeedbackCB)
        else:
            rospy.logwarn('Skippping unrecognized task in queue.')
        
    def _makeStow(self):
        '''Make a MoveArmGoal to move to the stow position'''
        #Create Orientation constraint object
        o_constraint = OrientationConstraint()
        o_constraint.header.frame_id = self.frameID
        o_constraint.header.stamp = rospy.Time.now()
        o_constraint.link_name = self.toolLinkName
        o_constraint.orientation = Quaternion(0.656788, 0.261971, 0.648416, -0.282059)
        o_constraint.absolute_roll_tolerance = 0.04
        o_constraint.absolute_pitch_tolerance = 0.04
        o_constraint.absolute_yaw_tolerance = 0.04
        
        #Determine position and tolerance from vector and box size
        pos_constraint = PositionConstraint()
        pos_constraint.header = o_constraint.header
        pos_constraint.link_name = self.toolLinkName
        pos_constraint.position = Point(-0.064433, 0.609915, 0)
        pos_constraint.constraint_region_shape.type = Shape.SPHERE
        pos_constraint.constraint_region_shape.dimensions = [0.1]
        
        preGraspGoal = MoveArmGoal()
        preGraspGoal.planner_service_name = self.plannerServiceName
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.armGroupName
        motion_plan_request.num_planning_attempts = 5
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        pos_constraint.weight = 1
        o_constraint.weight = 1
        motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
        motion_plan_request.goal_constraints.orientation_constraints.append(o_constraint)
        preGraspGoal.motion_plan_request = motion_plan_request
        return preGraspGoal
    
    def _makeMove(self):
        '''Move forward a small distance'''
        print "Making move goal"
        graspGoal = MoveArmGoal()
        graspGoal.planner_service_name = self.plannerServiceName
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.armGroupName
        motion_plan_request.num_planning_attempts = 5
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        #Orientation constraint is the same as for previous
        #Create Orientation constraint object
        o_constraint = OrientationConstraint()
        o_constraint.header.frame_id = self.frameID
        o_constraint.header.stamp = rospy.Time.now()
        o_constraint.link_name = self.toolLinkName
        o_constraint.orientation = Quaternion(0.656788, 0.261971, 0.648416, -0.282059)
        o_constraint.absolute_roll_tolerance = 0.04
        o_constraint.absolute_pitch_tolerance = 0.04
        o_constraint.absolute_yaw_tolerance = 0.04
        o_constraint.weight = 1
        motion_plan_request.goal_constraints.orientation_constraints.append(o_constraint)
        
        #Translate from pregrasp position to final position in a roughly straight line
        o = o_constraint.orientation
        p = Point(-0.064433, 0.609915, 0)
        preGraspMat = transformations.quaternion_matrix([o.x,o.y,o.z,o.w])
        preGraspMat[:3, 3] = [p.x,p.y,p.z]
        distance = .3#self.preGraspDistance + self.gripperFingerLength/2
        graspTransMat = transformations.translation_matrix([0,0,distance])
        graspMat = transformations.concatenate_matrices(preGraspMat, graspTransMat)
        #print preGraspMat
        #print graspTransMat
        #print graspMat
        p = transformations.translation_from_matrix(graspMat)
       
        #Publish grasp transform for visualization
        self._tf_broadcaster.sendTransform(
                (p[0],p[1],p[2]),
                (o.x, o.y, o.x, o.w),
                o_constraint.header.stamp,
                "grasp",
                o_constraint.header.frame_id)
 
        pos_constraint = PositionConstraint()
        pos_constraint.header = o_constraint.header
        pos_constraint.link_name = self.toolLinkName
        pos_constraint.position = Point(p[0],p[1],p[2])
        pos_constraint.constraint_region_shape.type = Shape.SPHERE
        pos_constraint.constraint_region_shape.dimensions = [0.01]#[0.01, 0.01, 0.01]
        pos_constraint.weight = 1
        motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
        graspGoal.motion_plan_request = motion_plan_request
        return graspGoal

class ManipulatorTask:
    
    TYPE_OPEN = 0
    TYPE_CLOSE = 1
    TYPE_MOVE = 2
    TYPE_ATTACH = 3
    TYPE_DETACH = 4
    
    def __init__(self, type=0, move_goal=MoveArmActionGoal(), object_name=""):
        self.type = type
        self.move_goal = move_goal
        self.object_name = object_name

if __name__ == "__main__":
    rospy.init_node('minimal_move')
    rospy.loginfo('Minimal Move Tester node started.')
    manipulator = MinimalMover()
    rospy.loginfo('Running the minimal mover.')
    manipulator.run()
    rospy.spin()
