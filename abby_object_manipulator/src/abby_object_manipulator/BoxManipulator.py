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

class BoxManipulator:
    #TODO Get these from the parameter server
    plannerServiceName = "/ompl_planning/plan_kinematic_path"
    armGroupName = "irb_120"
    armActionName = 'move_irb_120'
    toolLinkName = "gripper_body"
    frameID = "/irb_120_base_link"
    preGraspDistance = .075 #meters
    gripperFingerLength = 0.115 #meters
    gripperOpenWidth = 0.085 #0.065 #meters
    gripperClosedWidth = 0.046 #meters
    touchLinks = gripperCollisionNames = ("gripper_body", "gripper_jaw_1", "gripper_jaw_2")
    attachLinkName = "gripper_jaw_1"
    
    def __init__(self):
        self._tasks = Queue()
        self._moveArm = actionlib.SimpleActionClient(self.armActionName, MoveArmAction)
        self._moveArm.wait_for_server()
        rospy.loginfo('Box manipulator Connected to arm action server.')
        self._ik_server = rospy.ServiceProxy('/abby_irb_120_kinematics/get_constraint_aware_ik', GetConstraintAwarePositionIK)
        self._gripperClient = rospy.ServiceProxy('abby_gripper/gripper', gripper)
        self._joint_state_lock = Lock()
        with self._joint_state_lock:
            self._joint_states = JointState()
        self._joint_state_listener = rospy.Subscriber('/joint_states', JointState, self._jointStateCallback)
        #Wait until we get all the joint states, since they aren't aggregated
        while len(self._joint_states.name) <  14:
            rospy.sleep(0.1)
        rospy.loginfo('Box manipulator Connected to gripper service.')
        self._attachPub = rospy.Publisher('attached_collision_object', AttachedCollisionObject)
        rospy.loginfo('Box manipulator Publishing collision attachments on /attached_collision_object')
        self._boundingBoxClient = rospy.ServiceProxy('find_cluster_bounding_box', FindClusterBoundingBox)
        rospy.loginfo('Box manipulator Connected to bounding box service.')
        while not rospy.is_shutdown():
            try:
                self._tf_listener = tf.TransformListener()
                self._tf_listener.waitForTransform("/base_link","/irb_120_base_link", rospy.Time(0), rospy.Duration(5.0))
            except (tf.Exception):
                rospy.loginfo('Box manipulator still waiting on a transform')
                del self._tf_listener
            else:
                break;
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._pickServer = actionlib.SimpleActionServer('object_manipulation_pickup', PickupAction, auto_start=False)
        self._pickServer.register_goal_callback(self._pickGoalCB)
        self._pickServer.register_preempt_callback(self._pickPreemptCB)
        self._placeServer = actionlib.SimpleActionServer('object_manipulation_place', PlaceAction, auto_start=False)
        self._placeServer.register_goal_callback(self._placeGoalCB)
        self._placeServer.register_preempt_callback(self._placePreemptCB)
        self._pickServer.start()
        rospy.loginfo('Started pickup action server.')
        self._placeServer.start()
        rospy.loginfo('Started place action server.')
        rospy.loginfo('Box Manipulator Ready for Action Requests')
    
    def clearTasks(self):
        rospy.loginfo('Clearing task list.')
        with self._tasks.mutex:
            self._tasks.queue.clear()
    
    def _pickGoalCB(self):
        rospy.loginfo('Got a pick request')
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Accept the goal and read the object information
        goal = self._pickServer.accept_new_goal()
        box = self._boundingBoxClient(goal.target.cluster)
        #Check that the box width and/or length are within the gripper limits
        useX = not(box.box_dims.x > self.gripperOpenWidth or box.box_dims.x < self.gripperClosedWidth)
        useY = not(box.box_dims.y > self.gripperOpenWidth or box.box_dims.y < self.gripperClosedWidth)
        if not(useX or useY):
            rospy.logerr("Could not pick up the box because its dimensions are too large or small for the gripper")
            self._pickServer.set_aborted()
            return False
        #Add open gripper task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_OPEN))
        #Create goal message for pregrasp position and add it to the task queue
        preGraspGoal = self._makePreGrasp(box, goal.collision_object_name)
        if preGraspGoal == False:
            self._pickServer.set_aborted()
            return False
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE,preGraspGoal))
        #Add final approach task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE, self._makeGraspPath(preGraspGoal)))
        #Add close gripper task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_CLOSE))
        #Add attach object task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_ATTACH, object_name = goal.collision_object_name))
        #Add lift task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE, self._makeLiftPath(preGraspGoal, goal.collision_object_name)))
        #Mark successful
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_PICK_SUCCESS))
        self.runNextTask()
    
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
    
    def _pickPreemptCB(self):
        rospy.loginfo('Got a pick preempt request')
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Set current goal preempted
        self._pickServer.set_preempted()
    
    def _placeGoalCB(self):
        rospy.loginfo('Got a place request')
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Accept the goal and read the object information
        goal = self._placeServer.accept_new_goal()
        #Create goal message for place position and add it to the task queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE,self._makePlace(goal)))
        #Add open gripper task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_OPEN))
        #Add detach object task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_DETACH, object_name = goal.collision_object_name))
        #Mark successful
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_PLACE_SUCCESS))
        self.runNextTask()
    
    def _placePreemptCB(self):
        rospy.loginfo('Got a place preempt request')
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Set current goal preempted
        self._placeServer.set_preempted()
    
    def _moveArmDoneCB(self, state, result):
        '''If the arm successfully moved to the position, move on to the next task.'''
        if result.error_code.val == result.error_code.SUCCESS:
            rospy.loginfo("Arm motion successful. ")
            self._tasks.task_done()
            self.runNextTask()
        else:
            rospy.logerr("Arm motion failed! Error code:%d",result.error_code.val)
            if self._placeServer.is_active():
                result = PlaceResult()
                result.manipulation_result.value = result.manipulation_result.FAILED
                self._placeServer.set_aborted(result)
            if self._pickServer.is_active():
                result = PickupResult()
                result.manipulation_result.value = result.manipulation_result.FAILED
                self._pickServer.set_aborted(result)
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
            #print task.move_goal
            self._moveArm.send_goal(task.move_goal, self._moveArmDoneCB, self._moveArmActiveCB, self._moveArmFeedbackCB)
        elif task.type == task.TYPE_ATTACH:
            rospy.loginfo('Attaching object: %s', task.object_name)
            obj = AttachedCollisionObject()
            obj.object.header.stamp = rospy.get_rostime()
            obj.object.header.frame_id = self.frameID
            obj.object.operation.operation = CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
            obj.object.id = task.object_name
            obj.link_name = self.attachLinkName
            obj.touch_links = self.touchLinks
            #self._attachPub.publish(obj)
            self._tasks.task_done()
            self.runNextTask()
        elif task.type == task.TYPE_DETACH:
            rospy.loginfo('Detaching an object')
            obj = AttachedCollisionObject()
            obj.object.header.stamp = rospy.get_rostime()
            obj.object.header.frame_id = self.frameID
            obj.object.operation.operation = CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
            obj.object.id = task.object_name
            #self._attachPub.publish(obj)
            self._tasks.task_done()
            self.runNextTask()
        elif task.type == task.TYPE_PICK_SUCCESS:
            rospy.loginfo('Sending success')
            result = PickupResult()
            result.manipulation_result.value = result.manipulation_result.SUCCESS
            self._pickServer.set_succeeded(result)
            self._tasks.task_done()
            self.clearTasks()
        elif task.type == task.TYPE_PLACE_SUCCESS:
            rospy.loginfo('Sending success')
            result = PlaceResult()
            result.manipulation_result.value = result.manipulation_result.SUCCESS
            self._placeServer.set_succeeded(result)
            self._tasks.task_done()
            self.clearTasks()
        else:
            rospy.logwarn('Skippping unrecognized task in queue.')
            pass
        
    '''Function to make a grasp pose by searching through the arc of possible grasp angles until one
        gives you a successful IK result.'''
    def _makePreGraspPose(self, boxMat, axis):
        if axis==0: #x axis
            alpha = 0
            gamma = 0
        else: #y axis
            alpha = pi/2
            gamma = -pi/2
        ik_request = PositionIKRequest()
        ik_request.ik_link_name = self.toolLinkName
        with self._joint_state_lock:
            ik_request.ik_seed_state.joint_state = copy.deepcopy(self._joint_states)
        ik_request.pose_stamped = PoseStamped()
        ik_request.pose_stamped.header.stamp = rospy.Time.now()
        ik_request.pose_stamped.header.frame_id = self.frameID
        beta = pi/2
        while beta < 3*pi/2:
            rotationMatrix = transformations.euler_matrix(alpha, beta, gamma, 'rzyz')
            distance = self.preGraspDistance + self.gripperFingerLength
            preGraspMat = transformations.translation_matrix([0,0,-distance])
            fullMat = transformations.concatenate_matrices(boxMat, rotationMatrix, preGraspMat)
            p = transformations.translation_from_matrix(fullMat)
            q = transformations.quaternion_from_matrix(fullMat)
            print "trying {} radians".format(beta)
            try:
                self._ik_server(ik_request, Constraints(), rospy.Duration(5.0))
            except rospy.service.ServiceException:
                beta += 0.1
            else:
                if ik_resp.error_code.val > 0:
                    return beta
                else:
                    #print ik_resp.error_code.val
                    beta += 0.01
        rospy.logerr('No way to pick this up. All IK solutions failed.')
        return 7 * pi / 6
            
    def _makePreGrasp(self, box, objectName):
        '''Given a bounding box, identify an 
        approach vector, and designate a pregrasp position along that vector. Returns a MoveArmGoal
        message to move to the preGrasp position'''
        
        useX = not(box.box_dims.x > self.gripperOpenWidth or box.box_dims.x < self.gripperClosedWidth)
        useY = not(box.box_dims.y > self.gripperOpenWidth or box.box_dims.y < self.gripperClosedWidth)
         
        #Do all geometry in robot's base_link frame
        #Turn box pose into a tf for visualization
        self._tf_broadcaster.sendTransform(
                (box.pose.pose.position.x,box.pose.pose.position.y,box.pose.pose.position.z),
                (box.pose.pose.orientation.x,box.pose.pose.orientation.y,box.pose.pose.orientation.z,box.pose.pose.orientation.w),
                box.pose.header.stamp,
                objectName,
                box.pose.header.frame_id)
        #Turn box pose into a tf matrix in the proper frame
        boxPose = self._tf_listener.transformPose(self.frameID, box.pose)
        boxMat = transformations.quaternion_matrix([boxPose.pose.orientation.x,boxPose.pose.orientation.y,boxPose.pose.orientation.z,boxPose.pose.orientation.w])
        boxMat[:3, 3] = [boxPose.pose.position.x,boxPose.pose.position.y,boxPose.pose.position.z]
        #Approach Vector is in the principal plane of the box most closely aligned with robot XZ. This plane will be tool YZ
        #Vector is at a downward 30 degree angle
        #Orientation of box should be (approximately) a pure z rotation from the robot base_link
        #Therefore the angle of rotation about the z axis is appoximately
        # 2 * acos(q_w)
        boxPose_base = self._tf_listener.transformPose('/base_link', boxPose)
        theta = 2 * math.acos(boxPose_base.pose.orientation.w)
        rospy.logdebug('Angle is %f',theta)
        #Determine what to rotate the pose quaternion by to get the vector quaternion
        #Start with a 30 degree downward rotation
        if useX and not useY:
            rospy.loginfo("Can only grab box along x axis")
            width = box.box_dims.x
            if theta >= pi/2 and theta <= 3*pi/2:
            	rotationMatrix = transformations.euler_matrix(0, 5*pi/6, 0, 'rzyz')
            else:
                rotationMatrix = transformations.euler_matrix(pi, 5*pi/6, 0, 'rzyz')
        elif useY and not useX:
            rospy.loginfo("Can only grab box along y axis")
            width = box.box_dims.y
            rotationMatrix = transformations.euler_matrix(pi/2, 5*pi/6, -pi/2, 'rzyz')
        else:
            #Can use either face for pickup, so pick the one best aligned to the robot
            #TODO only -x and -y are currently correct
            rospy.loginfo("Theta is %f",theta)
            if theta >= pi/4 and theta <= 3*pi/4:
                rospy.loginfo("Grabbing box along -y axis")
                width = box.box_dims.x
                rotationMatrix = transformations.euler_matrix(pi/2, 7*pi/6, -pi/2, 'rzyz')
            elif theta >= 3*pi/4 and theta <= 5*pi/4:
                rospy.loginfo("Grabbing box along -x axis")
                width = box.box_dims.y
                rotationMatrix = transformations.euler_matrix(pi, 5*pi/6, pi/2, 'rzyz')
            elif theta >= 5*pi/4 and theta <= 7*pi/4:
                rospy.loginfo("Grabbing box along y axis")
                width = box.box_dims.x
                rotationMatrix = transformations.euler_matrix(pi/2, self._makePreGraspPose(boxMat, 1), -pi/2, 'rzyz')
            else:
                rospy.loginfo("Grabbing box along x axis")
                width = box.box_dims.y
                rotationMatrix = transformations.euler_matrix(0, 5*pi/6, pi/2, 'rzyz')
        #Rotated TF for visualization
        self._tf_broadcaster.sendTransform(
                (0,0,0), 
                transformations.quaternion_from_matrix(rotationMatrix),
                boxPose.header.stamp,
                objectName+'_rotated',
                objectName)
        #Pregrasp TF is rotated box TF translated back along the z axis
        distance = self.preGraspDistance + self.gripperFingerLength
        preGraspMat = transformations.translation_matrix([0,0,-distance])
        fullMat = transformations.concatenate_matrices(boxMat, rotationMatrix, preGraspMat)
        p = transformations.translation_from_matrix(fullMat)
        q = transformations.quaternion_from_matrix(fullMat)
        self._tf_broadcaster.sendTransform(
                p,
                q,
                boxPose.header.stamp,
                objectName+'_pregrasp',
                self.frameID)
        
        #Create Orientation constraint object
        o_constraint = OrientationConstraint()
        o_constraint.header.frame_id = self.frameID
        o_constraint.header.stamp = rospy.Time.now()
        o_constraint.link_name = self.toolLinkName
        o_constraint.orientation = Quaternion(q[0],q[1],q[2],q[3])
        o_constraint.absolute_roll_tolerance = 0.04
        o_constraint.absolute_pitch_tolerance = 0.04
        o_constraint.absolute_yaw_tolerance = 0.04
        
        #Determine position and tolerance from vector and box size
        pos_constraint = PositionConstraint()
        pos_constraint.header = o_constraint.header
        pos_constraint.link_name = self.toolLinkName
        pos_constraint.position = Point(p[0],p[1],p[2])
        #Position tolerance (in final tool frame) is:
        #  x = gripper open width - box x width
        #  y = object height * precision multiplier (<=1)
        #  z = 1 cm
        #Note that this is probably in the wrong frame, negating the advantage of all of the thinking in the preceding 4 lines
        #TODO: If you're feeling ambitious, turn this into a mesh and make it the right shape
        pos_constraint.constraint_region_shape.type = Shape.SPHERE
        pos_constraint.constraint_region_shape.dimensions = [0.01]
        #    self.gripperOpenWidth - width, 
        #    box.box_dims.z*0.2,
        #    0.01]
        
        preGraspGoal = MoveArmGoal()
        preGraspGoal.planner_service_name = self.plannerServiceName
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.armGroupName
        motion_plan_request.num_planning_attempts = 5
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        pos_constraint.weight = 1
        motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
        o_constraint.weight = 1
        motion_plan_request.goal_constraints.orientation_constraints.append(o_constraint)
        preGraspGoal.motion_plan_request = motion_plan_request
        
        #Turn off collision operations between the gripper and all objects
        for collisionName in self.gripperCollisionNames:
            collisionOperation = CollisionOperation(collisionName, 
                                    CollisionOperation.COLLISION_SET_ALL,
                                    0.0,
                                    CollisionOperation.DISABLE)
            preGraspGoal.operations.collision_operations.append(collisionOperation)
        return preGraspGoal
    
    def _makeGraspPath(self, preGraspGoal):
        '''Given a pregrasp MoveArmGoal message, generate a MoveArmGoal message to perform the final
        approach to the object to grasp it.'''
        
        graspGoal = MoveArmGoal()
        graspGoal.planner_service_name = self.plannerServiceName
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.armGroupName
        motion_plan_request.num_planning_attempts = 5
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        
        #Orientation constraint is the same as for pregrasp
        motion_plan_request.goal_constraints.orientation_constraints = copy.deepcopy(preGraspGoal.motion_plan_request.goal_constraints.orientation_constraints)
        #motion_plan_request.goal_constraints.orientation_constraints[0].orientation = Quaternion(0.656778, 0.261999, 0.648401, -0.282093) #stow orientation for debug
        graspGoal.motion_plan_request = motion_plan_request
        
        #Translate from pregrasp position to final position in a roughly straight line
        o = motion_plan_request.goal_constraints.orientation_constraints[0].orientation
        p = preGraspGoal.motion_plan_request.goal_constraints.position_constraints[0].position
        preGraspMat = transformations.quaternion_matrix([o.x,o.y,o.z,o.w])
        preGraspMat[:3, 3] = [p.x,p.y,p.z]
        distance = self.preGraspDistance * .75 + self.gripperFingerLength * .25
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
                motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp,
                "grasp",
                motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id)
 
        pos_constraint = PositionConstraint()
        pos_constraint.header = motion_plan_request.goal_constraints.orientation_constraints[0].header
        pos_constraint.link_name = self.toolLinkName
        pos_constraint.position = Point(p[0],p[1],p[2])
        #pos_constraint.position = Point(-0.0644721, 0.609922, 0) #Stow position for debug
        pos_constraint.constraint_region_shape.type = Shape.SPHERE
        pos_constraint.constraint_region_shape.dimensions = [0.01]
        pos_constraint.weight = 1
        motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
        #TODO: Add path constraint to require a (roughly) cartesian move
        
        #Turn off collision operations between the gripper and all objects
        for collisionName in self.gripperCollisionNames:
            collisionOperation = CollisionOperation(collisionName, 
                                    CollisionOperation.COLLISION_SET_ALL,
                                    0.0,
                                    CollisionOperation.DISABLE)
            graspGoal.operations.collision_operations.append(collisionOperation)
        return graspGoal
    
    def _makeLiftPath(self, preGraspGoal, objectName):
        '''Given a pregrasp MoveArmGoal message, generate a MoveArmGoal message to lift the object.'''
        
        liftGoal = preGraspGoal
        
        #Turn off collision operations between the gripper and all objects
        for collisionName in self.gripperCollisionNames:
            collisionOperation = CollisionOperation(collisionName, 
                                    CollisionOperation.COLLISION_SET_ALL,
                                    0.0,
                                    CollisionOperation.DISABLE)
            liftGoal.operations.collision_operations.append(collisionOperation)
        
        #Turn off collision operations between the object and all links
        collisionOperation = CollisionOperation(objectName, 
                                    CollisionOperation.COLLISION_SET_ALL,
                                    0.0,
                                    CollisionOperation.DISABLE)
        liftGoal.operations.collision_operations.append(collisionOperation)
        return liftGoal
  
    def _makePlace(self, placeGoal):
        placePose = placeGoal.place_locations.pop()
        
        goal = MoveArmGoal()
        goal.planner_service_name = self.plannerServiceName
        
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.armGroupName
        motion_plan_request.num_planning_attempts = 5
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = placePose.header.frame_id
        pos_constraint.link_name = self.toolLinkName
        pos_constraint.position = placePose.pose.position
        pos_constraint.constraint_region_shape.type = Shape.SPHERE
        pos_constraint.constraint_region_shape.dimensions = [0.01]
        pos_constraint.constraint_region_orientation.x = 0;
        pos_constraint.constraint_region_orientation.y = 0;
        pos_constraint.constraint_region_orientation.z = 0;
        pos_constraint.constraint_region_orientation.w = 1.0;
        pos_constraint.weight = 1
        motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
        
        o_constraint = OrientationConstraint()
        o_constraint.header = pos_constraint.header
        o_constraint.link_name = self.toolLinkName
        o_constraint.orientation = placePose.pose.orientation
        o_constraint.absolute_roll_tolerance = 0.04
        o_constraint.absolute_pitch_tolerance = 0.04
        o_constraint.absolute_yaw_tolerance = .555554
        o_constraint.weight = 1
        motion_plan_request.goal_constraints.orientation_constraints.append(o_constraint)
        
        goal.motion_plan_request = motion_plan_request
        return goal

class ManipulatorTask:
    
    TYPE_OPEN = 0
    TYPE_CLOSE = 1
    TYPE_MOVE = 2
    TYPE_ATTACH = 3
    TYPE_DETACH = 4
    TYPE_PICK_SUCCESS = 5
    TYPE_PLACE_SUCCESS = 6
    
    def __init__(self, type=0, move_goal=MoveArmActionGoal(), object_name=""):
        self.type = type
        self.move_goal = move_goal
        self.object_name = object_name

if __name__ == "__main__":
    rospy.init_node('box_manipulator')
    rospy.loginfo('Box manipulator node started.')
    manipulator = BoxManipulator()
    rospy.spin()
