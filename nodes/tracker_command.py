#!/usr/bin/env python

"""
    Send gesture-based commands to other nodes using a skeleton tracker such
    as the OpenNI tracker package in junction with a Kinect RGB-D camera.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import pi_tracker_lib as PTL
from skeleton_markers.msg import Skeleton
from pi_tracker.srv import *
import PyKDL as KDL
from math import copysign
import time

class TrackerCommand():
    def __init__(self):
        rospy.init_node('tracker_command')
        rospy.on_shutdown(self.shutdown)
        
        # How frequently do we publish      
        self.rate = rospy.get_param("~command_rate", 1)
        rate = rospy.Rate(self.rate)
        
        # Subscribe to the skeleton topic.
        rospy.Subscriber('skeleton', Skeleton, self.skeleton_handler)
        
        # Store the current skeleton configuration in a local dictionary.
        self.skeleton = dict()
        self.skeleton['confidence'] = dict()
        self.skeleton['position'] = dict()
        self.skeleton['orientation'] = dict()
        
        """ Define a dictionary of gestures to look for.  Gestures are defined by the corresponding function
            which generally tests for the relative position of various joints. """
            
        self.gestures = {'right_foot_up': self.right_foot_up,
                         'left_foot_up': self.left_foot_up,
                         'arms_crossed': self.arms_crossed
                         }
                                    
        # Connect to the base controller set_command service
        #rospy.wait_for_service('tracker_base_controller/set_command')
        self.base_controller_proxy = rospy.ServiceProxy('tracker_base_controller/set_command', SetCommand, persistent=True)
        self.base_active = False
        
        # Connect to the joint controller set_command service
        #rospy.wait_for_service('tracker_joint_controller/set_command')
        self.joint_controller_proxy = rospy.ServiceProxy('tracker_joint_controller/set_command', SetCommand, persistent=True)
        self.joints_active = False
        
        # Initialize the robot in the stopped state.
        self.tracker_command = "STOP"
        
        rospy.loginfo("Initializing Tracker Command Node...")
        
        while not rospy.is_shutdown():                           
            """ Get the scale of a body dimension, in this case the shoulder width, so that we can scale
                interjoint distances when computing gestures. """
            self.shoulder_width = PTL.get_body_dimension(self.skeleton, 'left_shoulder', 'right_shoulder', 0.4)
            
            # Compute the tracker command from the user's gesture
            self.tracker_command = self.get_command(self.tracker_command)
     
            rate.sleep()
            
    def right_foot_up(self):
        if self.confident(['right_foot', 'left_foot']):
            if (self.skeleton['position']['right_foot'].y() - self.skeleton['position']['left_foot'].y()) / self.shoulder_width > 0.7:
                return True
        return False
            
    def left_foot_up(self):
        if self.confident(['right_foot', 'left_foot']):
            if (self.skeleton['position']['left_foot'].y() - self.skeleton['position']['right_foot'].y()) / self.shoulder_width > 0.7:
                return True
        return False
        
    def arms_crossed(self):
        if self.confident(['left_elbow', 'right_elbow', 'left_hand', 'right_hand']):
            if copysign(1.0, self.skeleton['position']['left_elbow'].x() - self.skeleton['position']['right_elbow'].x()) == \
                 -copysign(1.0, self.skeleton['position']['left_hand'].x() - self.skeleton['position']['right_hand'].x()):
                return True
        return False
    
    def confident(self, joints):
        try:
            for joint in joints:
                if self.skeleton['confidence'][joint] < 0.5:
                    return False
            return True
        except:
            return False
        
    def get_command(self, current_command): 
        try:
            # Raise right knee to engage base controller
            if self.gestures['right_foot_up']():
                if self.base_active:
                    command = 'STOP'
                    self.base_active = False
                else:
                    command = 'DRIVE_BASE_FEET'
                    self.base_active = True
                try:
                    self.base_controller_proxy(command)
                except:
                    pass
                
            # Raise left knee to engage joint teleoperation
            elif self.gestures['left_foot_up']():
                if self.joints_active:
                    command = 'STOP'
                    self.joints_active = False
                else:
                    command = 'TELEOP_JOINTS'
                    self.joints_active = True
                try:
                    self.joint_controller_proxy(command)
                except:
                    pass
                    
            # Cross the arms to stop the robot base 
            elif self.gestures['arms_crossed']():
                if self.base_active:
                    command = "STOP"
                    self.base_active = False
                    try:
                        self.base_controller_proxy(command)
                    except:
                        pass
                else:
                    command = current_command   
  
            else:
                command = current_command
    
            if command != current_command:
                rospy.loginfo("Gesture Command: " + command)    
                
            return command
        except:
            return current_command
        
    def skeleton_handler(self, msg):
        for joint in msg.name:  
            self.skeleton['confidence'][joint] = msg.confidence[msg.name.index(joint)]
            self.skeleton['position'][joint] = KDL.Vector(msg.position[msg.name.index(joint)].x, msg.position[msg.name.index(joint)].y, msg.position[msg.name.index(joint)].z)
            self.skeleton['orientation'][joint] = msg.orientation[msg.name.index(joint)]

    def shutdown(self):
        rospy.loginfo("Shutting down Tracker Command Node.")
        
if __name__ == '__main__':
    try:
        TrackerCommand()
    except rospy.ROSInterruptException:
        pass