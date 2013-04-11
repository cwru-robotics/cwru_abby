#! /usr/bin/env python

import roslib; roslib.load_manifest('abby_arm_actions')
import rospy
import actionlib
from arm_navigation_msgs.msg import *
from abby_gripper.srv import *

class StoreObject:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_irb_120', MoveArmAction)
        
        rospy.logdebug("Setting up goal request message.")
        self.goal = MoveArmGoal()
        self.goal.planner_service_name = "/ompl_planning/plan_kinematic_path"
        
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = "irb_120"
        motion_plan_request.num_planning_attempts = 1
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        for i in range(0,6):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = "joint"+str(i+1)
            joint_constraint.tolerance_below = 0.05
            joint_constraint.tolerance_above = 0.05
            motion_plan_request.goal_constraints.joint_constraints.append(joint_constraint)
        #Joint Positions
        motion_plan_request.goal_constraints.joint_constraints[0].position = -1.03
        motion_plan_request.goal_constraints.joint_constraints[1].position =  0.21
        motion_plan_request.goal_constraints.joint_constraints[2].position = 1.09
        motion_plan_request.goal_constraints.joint_constraints[3].position = 0.0
        motion_plan_request.goal_constraints.joint_constraints[4].position = 0.03
        motion_plan_request.goal_constraints.joint_constraints[5].position = 1.33
        
        self.goal.motion_plan_request = motion_plan_request
        
        rospy.loginfo("Waiting for arm action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to arm action server.")
        rospy.loginfo("Waiting for gripper service server...")
        rospy.wait_for_service('abby_gripper/gripper')
        self.gripperClient = rospy.ServiceProxy('abby_gripper/gripper', gripper)
        rospy.loginfo("Connected to gripper service server.")
        
    def sendOnce(self, timeOut = 60):
        rospy.loginfo('Sending bin position goal...')
        self.client.send_goal(self.goal)
        if self.client.wait_for_result(rospy.Duration(timeOut, 0)):
            return self.client.get_result()
        else:
            rospy.logwarn('Timed out attempting to move to bin.')
            return False
    
    def sendUntilSuccess(self, timeOut = 60):
        result = False
        r = rospy.Rate(1)
        while not result and not rospy.is_shutdown():
            result = self.sendOnce(timeOut)
            if not result:
                r.sleep()
        if rospy.is_shutdown():
            return False
        status = result.error_code
        if status.val == status.SUCCESS:
            rospy.loginfo("Arm successfully moved to bin.")
        else:
            rospy.logwarn("Arm failed to go to bin.")
        return result
    
    def storeObject(self, timeOut = 60):
        result = self.sendUntilSuccess(timeOut)
        if result.error_code.val == result.error_code.SUCCESS:
            try:
                response = self.gripperClient(gripperRequest.OPEN)
                rospy.loginfo("Stored object")
                return True
            except rospy.ServiceException, e:
                rospy.logerr("Gripper service did not process request")
        rospy.logerr("Failed to store object. Error code: %d", result.error_code.val)
        return False

if __name__ == '__main__':
    rospy.init_node('store_object')
    rospy.loginfo("Node initialized.")
    storeObject = StoreObject()
    storeObject.storeObject()
