#! /usr/bin/env python

import roslib; roslib.load_manifest('abby_arm_actions')
import rospy
import actionlib
from arm_navigation_msgs.msg import *

LEFT = 0
RIGHT = 1

class ClearArm:
    
    def __init__(self):
        
        self.client = actionlib.SimpleActionClient('move_irb_120', MoveArmAction)
        
        self.LEFT = LEFT
        self.RIGHT = RIGHT
        
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
        motion_plan_request.goal_constraints.joint_constraints[0].position = 0.50
        motion_plan_request.goal_constraints.joint_constraints[1].position = -1.57
        motion_plan_request.goal_constraints.joint_constraints[2].position = 0.75#1.22
        motion_plan_request.goal_constraints.joint_constraints[3].position = 0.0
        motion_plan_request.goal_constraints.joint_constraints[4].position = 0.03
        motion_plan_request.goal_constraints.joint_constraints[5].position = -0.84
        
        self.goal.motion_plan_request = motion_plan_request
        
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to action server.")
        
    def sendOnce(self, direction = RIGHT, timeOut = 60):
        rospy.loginfo('Sending move goal...')
        if direction==self.LEFT:
            self.goal.motion_plan_request.goal_constraints.joint_constraints[0].position = -0.75
        else:
            self.goal.motion_plan_request.goal_constraints.joint_constraints[0].position = 0.75
        self.client.send_goal(self.goal)
        if self.client.wait_for_result(rospy.Duration(timeOut, 0)):
            return self.client.get_result()
        else:
            rospy.logwarn('Timed out attempting to move arm.')
            return False
    
    def sendUntilSuccess(self, direction = RIGHT, timeOut = 60):
        result = False
        r = rospy.Rate(1)
        while not result and not rospy.is_shutdown():
            result = self.sendOnce(direction, timeOut)
            if not result:
                r.sleep()
        if rospy.is_shutdown():
            return False
        status = result.error_code
        if status.val == status.SUCCESS:
            rospy.loginfo("Arm successfully moved.")
        else:
            rospy.logwarn("Arm failed to move.")

if __name__ == '__main__':
    rospy.init_node('clear_arm')
    rospy.loginfo("Node initialized.")
    try:
        stowArm = ClearArm()
        stowArm.sendUntilSuccess(RIGHT)
        stowArm.sendUntilSuccess(LEFT)
    except KeyboardInterrupt:
        rospy.loginfo("Got Keyboard Interrupt. Shutting down.")
