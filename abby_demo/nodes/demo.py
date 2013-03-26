#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Edward Venator, Case Western Reserve University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Case Western Reserve University nor the names 
#    of its contributors may be used to endorse or promote products 
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = "esv@case.edu (Edward Venator)"

import roslib; roslib.load_manifest('abby_demo')
import rospy
import actionlib
import sys

#Object manipulation
#from abby_object_manipulator import manipulation_controller

#arm
from arm_navigation_msgs.msg import MoveArmAction
#from abby_arm_actions.stow_arm import StowArm

#Gripper
from abby_gripper.srv import gripper, gripperRequest

#Nav
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal

#AMCL
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == '__main__':
    rospy.init_node('abby_demo')
    rospy.loginfo("Demo node initialized. Waiting for AMCL...")
    
    #Wait on services and set initial conditions
    #Wait on AMCL
    rospy.wait_for_service('global_localization')
    rospy.loginfo("AMCL is up. Waiting Move Base...")
    #Wait on move base action server
    move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base.wait_for_server()
    rospy.loginfo("Move Base is up. Waiting for move_irb_120...")
    #Wait for arm action server
    move_irb_120 = actionlib.SimpleActionClient('move_irb_120', MoveArmAction)
    move_irb_120.wait_for_server()
    rospy.loginfo("move_irb_120 is up. Stowing arm...")
    #Stow arm
    stowArm = StowArm()
    stowArm.sendUntilSuccess()
    #Wait on gripper service
    rospy.loginfo("Closing Gripper...")
    rospy.wait_for_service('abby_gripper/gripper')
    gripper = rospy.ServiceProxy('abby_gripper/gripper', gripper)
    #Close gripper
    gripper(gripperRequest.CLOSE)
    #Set up manipulator
    rospy.loginfo('Starting the object manipulation controller')
    controller = ObjectManipulationController()
    
    
    #Go to pickup position
    rospy.loginfo('Going to the table')
    if not goToTable():
        rospy.logerr("Error going to the table")
        sys.exit(1)
    
    while not rospy.is_shutdown(): 
        #Run detection service
        resp = controller.runSegmentation()
        if resp.result == resp.SUCCESS:
            rospy.loginfo("Tabletop detection service returned %d clusters", len(resp.clusters))
            break;
        elif resp.result == resp.NO_TABLE:
            rospy.logwarn("No table detected")
            perturbBase()
        elif resp.result == resp.NO_CLOUD_RECEIVED:
            rospy.logwarn("Tabletop segmenter did not receive a point cloud. Has the kinect crashed?")
            rospy.sleep(rospy.Duration(1,0))
        elif resp.result == resp.OTHER_ERROR:
            rospy.logerr("Tabletop segmentation error")
            sys.exit(1)
        #Pick up all objects on table
        for index in range(len(controller.getMapResponse().graspable_objects)):
            rospy.loginfo("Picking up object number %d", index)
            while not controller.pickup(controller.getMapResponse(), index):
                #If can't pick up any object, perturb drivetrain, try again
                perturbBase()
            while not controller.storeObject():
                #If can't stow object (due to table collision), back up, try again
                backup()
            #Stow the arm and return to the table
            stowArm.sendUntilSuccess()
            goToTable()
    #Return to operator position
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x = 0.0 #X goes here
    goal.target_pose.pose.position.y = 0.0 #Y goes here
    quaternion = tf.transformations.quaternion_about_axis(0.0 , (0,0,1)) #Theta
    goal.target_pose.pose.orientation = Quaternion(*quaternion) 
    move_base.send_goal(goal)
    move_base.wait_for_result()
    if move_base.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("Returned to operator position")
    else:
        rospy.logerr("Could not drive to operator position")
        sys.exit(1)
    rospy.loginfo("Finished script. Shutting down")

def goToTable():
    '''Go to the table to pick things up from'''
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x = 0.0 #X goes here
    goal.target_pose.pose.position.y = 0.0 #Y goes here
    quaternion = tf.transformations.quaternion_about_axis(0.0 , (0,0,1)) #Theta
    goal.target_pose.pose.orientation = Quaternion(*quaternion) 
    move_base.send_goal(goal)
    move_base.wait_for_result()
    if move_base.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("Arrived at pickup position")
    else:
        rospy.logerr("Could not drive to pickup position")
        return False


def perturbBase():
    '''Randomly move the drivetrain a little to enable picking stuff up'''
    return False
