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

import roslib; roslib.load_manifest("abby_object_manipulator")
import rospy
import actionlib
from std_srvs.srv import Empty
from tabletop_object_detector.srv import TabletopSegmentation
from tabletop_object_detector.msg import TabletopDetectionResult
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from household_objects_database_msgs.msg import DatabaseModelPoseList
from object_manipulation_msgs.msg import *
from geometry_msgs.msg import *
'''This node coordinates messages and services for the object manipulation pipeline
on Abby. It serves a similar purpose to tabletop_complete, but does not perform
object detection.'''

class ObjectManipulationController:
    '''Not threadsafe'''
    def __init__(self):
        rospy.loginfo('Waiting for tabletop_segmentation service')
        rospy.wait_for_service('tabletop_segmentation')
        self.segmentationService = rospy.ServiceProxy('tabletop_segmentation', TabletopSegmentation)
        rospy.loginfo('Connected to tabletop segmentation service')
        rospy.loginfo('Waiting for tabletop_collision_map_processing service')
        rospy.wait_for_service('tabletop_collision_map_processing/tabletop_collision_map_processing')
        self.collisionMapService = rospy.ServiceProxy('tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)
        rospy.loginfo('Connected to collision map processing service')
        self._pickupClient = actionlib.SimpleActionClient('object_manipulation_pickup', PickupAction)
        self._placeClient = actionlib.SimpleActionClient('object_manipulation_place', PlaceAction)
        rospy.loginfo('Object manipulation controller waiting for pick and place servers.')
        self._pickupClient.wait_for_server()
        self._placeClient.wait_for_server()
        rospy.loginfo('Object manipulation controller connected to pick and place servers.')
        rospy.loginfo('Object manipulation controller started.')
    
    def segmentationReplyToDetectionReply(self, segmentationResult):
        '''Converts a TableTopSegmentationReply into a TabletopDetectionResult'''
        detectionResult = TabletopDetectionResult()
        detectionResult.table = segmentationResult.table
        detectionResult.clusters = segmentationResult.clusters
        detectionResult.models.append(DatabaseModelPoseList())
        for i in range(len(detectionResult.clusters)):
            detectionResult.cluster_model_indices.append(0)
        detectionResult.result = segmentationResult.result
        return detectionResult
    
    def runSegmentation(self):
        '''Calls the segmentation service and sends the result on to the collision map processing server.'''
        segmentationResult = self.segmentationService()
        detectionResult = self.segmentationReplyToDetectionReply(segmentationResult)
        self.mapResponse = self.collisionMapService(detectionResult, True, True, '/base_link')
        return segmentationResult
    
    def getMapResponse(self):
        '''Returns graspable objects from the collision map server. Run runSegmenation() first.'''
        return self.mapResponse
    
    def pickup(self, mapResponse, index):
        '''Sends a command to pick up the graspable object at the given index in mapResponse'''
        goal = PickupGoal()
        goal.arm_name = 'irb_120'
        goal.target = mapResponse.graspable_objects[index]
        goal.collision_object_name = mapResponse.collision_object_names[index]
        goal.collision_support_surface_name = mapResponse.collision_support_surface_name
        goal.allow_gripper_support_collision = False
        goal.use_reactive_execution = False
        goal.use_reactive_lift = False
        goal.ignore_collisions = False
        
        #Set this to false to enable actual execution of grasping
        #This not actually implemented in manipulator node, so it does nothing
        goal.only_perform_feasibility_test = True
        
        self._pickupClient.wait_for_server()
        self._pickupClient.send_goal(goal)
        if self._pickupClient.wait_for_result(rospy.Duration.from_sec(120.0)):
            result = self._pickupClient.get_result()
            if result.manipulation_result.value == result.manipulation_result.SUCCESS:
                self.currentlyHeldObject = goal.target
                rospy.loginfo("Successfully picked up object")
                return True;
            rospy.logwarn("Pickup failed. Error %d",result.manipulation_result.value)
        rospy.logwarn("Pickup timed out after %f seconds", 60.0)
        return False;

    def storeObject(self):
        '''Sends a command to store the currently held object in the bin'''
        rospy.loginfo('Storing the currently held object in the bin')
        goal = PlaceGoal()
        goal.arm_name = 'irb_120'
        bin_location = PoseStamped()
        bin_location.header.frame_id = '/frame1'
        bin_location.pose.position.x = -0.174#-0.198
        bin_location.pose.position.y = -0.672#-0.758
        bin_location.pose.position.z =  0.835#0.671#0.668
        bin_location.pose.orientation.x = -0.665#-0.708
        bin_location.pose.orientation.y =  0.302# 0.057
        bin_location.pose.orientation.z = -0.122#-0.386
        bin_location.pose.orientation.w =  0.672# 0.589
        goal.place_locations.append(bin_location)
        goal.collision_object_name = self.currentlyHeldObject.collision_name
        rospy.loginfo('Sent place goal to box manipulator')
        self._placeClient.send_goal(goal)
        if self._placeClient.wait_for_result(rospy.Duration.from_sec(60.0)):
            result = self._placeClient.get_result()
            if result.manipulation_result.value == result.manipulation_result.SUCCESS:
                self.currentlyHeldObject = None
                rospy.loginfo("Successfully stored object in bin")
                return True
            rospy.logwarn("Place failed. Error %d",result.manipulation_result.value)
        rospy.logwarn("Place timed out after %f seconds", 60.0)
        return False
if __name__ == '__main__':
    rospy.init_node('object_manipulation_controller')
    rospy.loginfo('Started the object manipulation controller')
    rospy.wait_for_service("/collider_node/reset");
    resetCollider = rospy.ServiceProxy("/collider_node/reset", Empty)
    resetCollider()
    controller = ObjectManipulationController()
    timer = rospy.Rate(.2)
    #while not rospy.is_shutdown():
    #    resp = controller.runSegmentation()
    #    if resp.result == resp.SUCCESS:
    #        rospy.loginfo("Tabletop detection service returned %d clusters", len(resp.clusters))
    #        mapResponse = controller.getMapResponse()
    #        break
    #    elif resp.result == resp.NO_TABLE:
    #        rospy.loginfo("No table detected")
    #    elif resp.result == resp.NO_CLOUD_RECEIVED:
    #        rospy.logwarn("No cloud received")
    #    elif resp.result == resp.OTHER_ERROR:
    #        rospy.logerr("Tabletop segmentation error")
    #for index in range(len(mapResponse.graspable_objects)):
    #    rospy.loginfo("Picking up object number %d", index)
    #    if controller.pickup(mapResponse, index):
    #        controller.storeObject()
    controller.currentlyHeldObject = GraspableObject()
    controller.storeObject()
    #rospy.spin()
    #timer.sleep()
        
