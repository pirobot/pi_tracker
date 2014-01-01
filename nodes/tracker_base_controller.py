#!/usr/bin/env python

"""
    Control a differential drive robot base using a skeleton tracker such
    as the OpenNI tracker package in conjunction with a Kinect RGB-D camera.
    
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
from geometry_msgs.msg import Twist
import pi_tracker_lib as PTL
from skeleton_markers.msg import Skeleton
from pi_tracker.srv import *
import PyKDL as KDL
from math import atan2, sqrt, copysign, pi

class TrackerBaseController():
    def __init__(self):
        rospy.init_node('tracker_base_controller')
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("Initializing Base Controller Node...")
        
        self.rate = rospy.get_param('~base_controller_rate', 1)  
        rate = rospy.Rate(self.rate)
        
        self.max_drive_speed = rospy.get_param('~max_drive_speed', 0.3)
        self.max_rotation_speed = rospy.get_param('~max_rotation_speed', 0.5)
        self.base_control_side = rospy.get_param('~base_control_side', "right")
        self.holonomic = rospy.get_param('~holonomic', False)
        self.scale_drive_speed = rospy.get_param('~scale_drive_speed', 1.0)
        self.scale_rotation_speed = rospy.get_param('~scale_rotation_speed', 1.0)
        self.reverse_rotation = rospy.get_param('~reverse_rotation', False)
        
        self.HALF_PI = pi / 2.0
        
        # Subscribe to the skeleton topic.
        rospy.Subscriber('skeleton', Skeleton, self.skeleton_handler)
        
        # Store the current skeleton configuration in a local dictionary.
        self.skeleton = dict()
        self.skeleton['confidence'] = dict()
        self.skeleton['position'] = dict()
        self.skeleton['orientation'] = dict()
        
        # Set up the base controller service.
        self.set_state = rospy.Service('~set_command', SetCommand, self.set_command_callback)
        
        # We will control the robot base with a Twist message.
        self.cmd_vel = Twist()
        
        # And publish the command on the /cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)
        
        # Keep track of the average width of the shoulders to scale gestures to body size.
        self.shoulder_width = PTL.get_body_dimension(self.skeleton, 'left_shoulder', 'right_shoulder', 0.4)
        self.count = 1.0
        
        drive_vector = dict()
        drive_side_test = dict()
        
        # Initialize the robot in the stopped state.
        self.tracker_command = "STOP"
        
        while not rospy.is_shutdown():        
            # If we receive the STOP command, or we lose sight of both hands then stop all motion.
            if self.tracker_command == 'STOP' or (self.skeleton['confidence']['left_hand'] < 0.5 and self.skeleton['confidence']['right_hand'] < 0.5):
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.linear.y = 0.0
                self.cmd_vel.angular.z = 0.0
                
            elif self.tracker_command == 'DRIVE_BASE_HANDS':
                self.drive_base_hands()      
                    
            elif self.tracker_command == 'DRIVE_BASE_FEET':
                self.drive_base_feet()
                
            else:
                pass      
            
            # Some robots might require a scale factor on the speeds and a reversal of the rotation direction.
            self.cmd_vel.linear.x = self.cmd_vel.linear.x * self.scale_drive_speed
            self.cmd_vel.angular.z = self.cmd_vel.angular.z * self.scale_rotation_speed
            if self.reverse_rotation:
                self.cmd_vel.angular.z = -self.cmd_vel.angular.z
                
            #rospy.loginfo('DRIVE: ' + str(self.cmd_vel.linear.x) + ' ROTATE: ' + str(self.cmd_vel.angular.z))

            # Publish the drive command on the /cmd_vel topic.
            #rospy.loginfo('X: ' + str(self.cmd_vel.linear.x) + ' Y: ' + str(self.cmd_vel.linear.y) + ' Z: ' + str(self.cmd_vel.angular.z))               

            self.cmd_vel_pub.publish(self.cmd_vel)
            
            rate.sleep()
            
    def drive_base_hands(self):
        # Compute the drive command based on the position of the control hand relative to the shoulder.
        if self.base_control_side == "right":
            side = "right"
        else:
            side = "left"
        
        # Use the upper arm length to scale the gesture.
        self.upper_arm_length = PTL.get_body_dimension(self.skeleton, side + '_shoulder', side + '_elbow', 0.25)
        
        # Use the forearm length to compute the origin of the control plane.
        self.forearm_length = PTL.get_body_dimension(self.skeleton, side + '_elbow', side + '_hand', 0.3)
        
        if not self.confident([side + '_shoulder', side + '_hand']):
            return
            
        drive_vector = self.skeleton['position'][side + '_shoulder'] - self.skeleton['position'][side + '_hand']
                
        """ Split out the x, y motion components.  The origin is set near the position of the elbow when the arm
            is held straigth out. """
        self.cmd_vel.linear.x = self.max_drive_speed * (drive_vector.z() - self.upper_arm_length ) / self.upper_arm_length
        self.cmd_vel.linear.y = -self.max_drive_speed * drive_vector.x() / self.upper_arm_length
                    
        # For holonomic control, use torso rotation to rotate in place.
        if self.holonomic:
            try:
                # Check for torso rotation to see if we want to rotate in place.
                try:
                    torso_quaternion = self.skeleton['orientation']['torso']
                    torso_rpy = torso_quaternion.GetRPY()
                    torso_angle = torso_rpy[1]
                except:
                    torso_angle = 0
                if abs(torso_angle) > 0.6:
                    self.cmd_vel.angular.z = self.max_rotation_speed * torso_angle * 1.5 / self.HALF_PI
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.linear.y = 0.0
                else:
                    self.cmd_vel.angular.z = 0.0
            except:
                pass
                
        else:
            # For non-holonomic control, segment the control plane into forward/backward and rotation motions.
            try:     
                drive_speed = self.cmd_vel.linear.x
                
                drive_angle = atan2(self.cmd_vel.linear.y, self.cmd_vel.linear.x)
        
                self.cmd_vel.linear.y = 0
                
                if abs(drive_speed) < 0.07:
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                    return
                    
                #rospy.loginfo("DRIVE SPEED: " + str(drive_speed) + " ANGLE: " + str(drive_angle))

                if (drive_speed < 0 and abs(drive_angle) > 2.5) or (drive_angle > -0.7 and drive_angle < 1.5):
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel.linear.x = drive_speed
                
                elif drive_angle < -0.7:
                    self.cmd_vel.angular.z = self.max_rotation_speed
                    self.cmd_vel.linear.x = 0.0
                
                elif drive_angle > 1.5:
                    self.cmd_vel.angular.z = -self.max_rotation_speed
                    self.cmd_vel.linear.x = 0.0
                
                else:
                    pass
            except:
                pass
            
    def drive_base_feet(self):
        # Compute the drive command based on the position of the legs relative to the torso.
        if self.base_control_side == "right":
            drive_vector = self.skeleton['position']['left_foot'] - self.skeleton['position']['right_foot']
        else:
            drive_vector = self.skeleton['position']['right_foot'] - self.skeleton['position']['left_foot']

        drive_angle = 0.0
        drive_speed = 0.0
        self.cmd_vel.linear.y = 0     

        drive_speed = sqrt(drive_vector.x() * drive_vector.x() + drive_vector.z() * drive_vector.z())
        drive_angle = atan2(-drive_vector.x(), drive_vector.z())
        drive_angle = drive_angle - self.HALF_PI
        
        drive_speed = drive_speed / (self.shoulder_width * 4)
        
        if drive_speed > self.max_drive_speed:
            drive_speed = self.max_drive_speed
        elif drive_speed < 0.05:
            drive_speed = 0.0
        
        drive_speed = copysign(drive_speed, drive_vector.z())
        
        rospy.loginfo('FEET SPD: ' + str(drive_speed) + ' ANG: ' + str(drive_angle))    
            
        if drive_angle > -0.7 and drive_angle < 0.0:
            self.cmd_vel.linear.x = 0.0
            if abs(drive_speed) > 0.15:
                self.cmd_vel.angular.z = -self.max_rotation_speed
            else:
                self.cmd_vel.angular.z = 0
        
        elif drive_angle > 0.0 and drive_angle < 0.7:
            self.cmd_vel.linear.x = 0.0
            if abs(drive_speed) > 0.15:
                self.cmd_vel.angular.z = self.max_rotation_speed
            else:
                self.cmd_vel.angular.z = 0

        else:
          self.cmd_vel.linear.x = drive_speed
          self.cmd_vel.angular.z = 0.0

    def confident(self, joints):
        try:
            for joint in joints:
                if self.skeleton['confidence'][joint] < 0.5:
                    return False
            return True
        except:
            return False
            
    def set_command_callback(self, req):
        self.tracker_command = req.command
        return SetCommandResponse()
        
    def skeleton_handler(self, msg):
        for joint in msg.name:  
            self.skeleton['confidence'][joint] = msg.confidence[msg.name.index(joint)]
            self.skeleton['position'][joint] = KDL.Vector(msg.position[msg.name.index(joint)].x, msg.position[msg.name.index(joint)].y, msg.position[msg.name.index(joint)].z)
            self.skeleton['orientation'][joint] = KDL.Rotation.Quaternion(msg.orientation[msg.name.index(joint)].x, msg.orientation[msg.name.index(joint)].y, msg.orientation[msg.name.index(joint)].z, msg.orientation[msg.name.index(joint)].w)

    def shutdown(self):
        rospy.loginfo('Shutting down Tracker Base Controller Node.')
        
if __name__ == '__main__':
    try:
        TrackerBaseController()
    except rospy.ROSInterruptException:
        pass