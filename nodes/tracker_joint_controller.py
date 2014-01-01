#!/usr/bin/env python

"""
    Control the joints of a robot using a skeleton tracker such as the
    OpenNI tracker package in junction with a Kinect RGB-D camera.
    
    Based on Taylor Veltrop's C++ work (http://www.ros.org/wiki/veltrop-ros-pkg)
    
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
from sensor_msgs.msg import JointState
from skeleton_markers.msg import Skeleton
from pi_tracker.srv import *
import PyKDL as KDL
from math import acos, asin, pi

class TrackerJointController():
    def __init__(self):
        rospy.init_node('tracker_joint_controller')
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("Initializing Joint Controller Node...")
        
        self.rate = rospy.get_param('~joint_controller_rate', 5)
        rate = rospy.Rate(self.rate)
        
        self.skel_to_joint_map = rospy.get_param("~skel_to_joint_map", dict())
        self.use_real_robot = rospy.get_param('~use_real_robot', False)
        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.5)
        self.tracker_commands = rospy.get_param('~tracker_commands', ['STOP', 'TELEOP_JOINTS'])
         
        self.HALF_PI = pi / 2.0

        # Subscribe to the skeleton topic.
        rospy.Subscriber('skeleton', Skeleton, self.skeleton_handler)
        
        # Store the current skeleton configuration in a local dictionary.
        self.skeleton = dict()
        self.skeleton['confidence'] = dict()
        self.skeleton['position'] = dict()
        self.skeleton['orientation'] = dict()
         
        # Set up the tracker command service for this node.
        self.set_command = rospy.Service('~set_command', SetCommand, self.set_command_callback)

        # We only use the joint_state publisher if we are not using a real robot (so we can see them in RViz)
        if not self.use_real_robot:
            self.joint_state_pub = rospy.Publisher('/joint_states', JointState)      
        else:
            # The joints are controlled by publishing a joint state message on the /cmd_joints topic.
            self.cmd_joints_pub = rospy.Publisher('/cmd_joints', JointState)
        
        # The get_joints command parses a URDF description of the robot to get all the non-fixed joints.
        self.cmd_joints = JointState()
        self.cmd_joints = PTL.get_joints()
        
        # Store the last joint command so we can stop and hold a given posture.
        self.last_cmd_joints = JointState()
        self.last_cmd_joints = PTL.get_joints()
        
        # Initialize the robot in the stopped state.
        self.tracker_command = "STOP"
        
        while not rospy.is_shutdown():              
            # Execute the behavior appropriate for the current command.
            if self.tracker_command in self.tracker_commands:
                if self.tracker_command == 'STOP':
                    self.stop_joints()
                else:
                    self.teleop_joints()
                
                self.cmd_joints.header.stamp = rospy.Time.now()
                
                if self.use_real_robot:
                    try:
                        self.cmd_joints_pub.publish(self.cmd_joints)
                    except:
                        pass
                else:
                     self.joint_state = self.cmd_joints
                     self.joint_state_pub.publish(self.joint_state)
            else:
                pass
                
            self.last_cmd_joints = self.cmd_joints
            
            rate.sleep()
            
    def stop_joints(self):
        self.cmd_joints = self.last_cmd_joints
        
    def relax_joints(self):
        pass
        
    def reset_joints(self):
        pass
        
    def enable_joints(self):
        pass
            
    def teleop_joints(self):
        # Fixed Joints: Set position to 0 radians.
        self.cmd_joints.position[self.cmd_joints.name.index('left_arm_roll_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_roll_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index('right_arm_roll_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_roll_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index('left_wrist_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index('left_wrist_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index('right_wrist_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index('right_wrist_joint')] = 2
        
                    
        # Torso Rotation          
        try:
            torso_quaternion = self.skeleton['orientation']['torso']
            torso_rpy = torso_quaternion.GetRPY()
            self.cmd_joints.position[self.cmd_joints.name.index(self.skel_to_joint_map['torso'])] = torso_rpy[1] / 2.0
            self.cmd_joints.velocity[self.cmd_joints.name.index('torso_joint')] = self.default_joint_speed
        except:
            pass
        
        # Head Pan           
        try:
            head_quaternion = self.skeleton['orientation']['head']
            head_rpy = head_quaternion.GetRPY()
            self.cmd_joints.position[self.cmd_joints.name.index('head_pan_joint')] = head_rpy[1]
            self.cmd_joints.velocity[self.cmd_joints.name.index('head_pan_joint')] = self.default_joint_speed
        except:
            pass
        
        # Head Tilt
        try:
            self.cmd_joints.position[self.cmd_joints.name.index('head_tilt_joint')] = head_rpy[0]
            self.cmd_joints.velocity[self.cmd_joints.name.index('head_tilt_joint')] = self.default_joint_speed
        except:
            pass
        
        # Left Arm
        try:
            left_shoulder_neck = self.skeleton['position']['neck'] - self.skeleton['position']['left_shoulder']
            left_shoulder_elbow = self.skeleton['position']['left_elbow'] - self.skeleton['position']['left_shoulder']
            left_elbow_hand = self.skeleton['position']['left_hand'] - self.skeleton['position']['left_elbow']
            left_shoulder_hand = self.skeleton['position']['left_hand'] - self.skeleton['position']['left_shoulder']
            
            left_shoulder_neck.Normalize()
            left_shoulder_elbow.Normalize()
            lh = left_elbow_hand.Normalize()
            left_shoulder_hand.Normalize()                           

            left_shoulder_lift_angle = asin(left_shoulder_elbow.y()) + self.HALF_PI           
            left_shoulder_pan_angle = asin(left_elbow_hand.x())
            left_elbow_angle = -acos(KDL.dot(left_shoulder_elbow, left_elbow_hand))
            left_wrist_angle = -left_elbow_angle / 2.0
            left_elbow_angle = left_elbow_angle / 4.0
            
            self.cmd_joints.position[self.cmd_joints.name.index('left_shoulder_lift_joint')] = left_shoulder_lift_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('left_shoulder_lift_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index('left_shoulder_pan_joint')] = left_shoulder_pan_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('left_shoulder_pan_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index('left_elbow_joint')] = left_elbow_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('left_elbow_joint')] = self.default_joint_speed                                   
        
            self.cmd_joints.position[self.cmd_joints.name.index('left_wrist_joint')] = left_wrist_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('left_wrist_joint')] = self.default_joint_speed
            
#                left_elbow_rpy = [0]*3
#                left_elbow_rpy = euler_from_quaternion(self.skeleton['orientation']['left_elbow'])
#                left_arm_roll_angle = -left_elbow_rpy[2]

            left_arm_roll_angle = acos(KDL.dot(left_shoulder_elbow, left_shoulder_neck))
            #left_arm_roll_angle = -asin(left_shoulder_hand.x())
            self.cmd_joints.position[self.cmd_joints.name.index('left_arm_roll_joint')] = 0
            self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_roll_joint')] = self.default_joint_speed
        
#                left_wrist_angle = -acos(min(1, abs((lh - 0.265) / (0.30 - 0.265)))) + QUARTER_PI
#                self.cmd_joints.position[self.cmd_joints.name.index('left_wrist_joint')] = 0
#                self.cmd_joints.velocity[self.cmd_joints.name.index('left_wrist_joint')] = self.default_joint_speed
        
        except KeyError:
            pass      
            
        # Right Arm
        try:
            right_shoulder_neck = self.skeleton['position']['neck'] - self.skeleton['position']['right_shoulder']
            right_shoulder_elbow = self.skeleton['position']['right_elbow'] - self.skeleton['position']['right_shoulder']
            right_elbow_hand = self.skeleton['position']['right_hand'] - self.skeleton['position']['right_elbow']
            right_shoulder_hand = self.skeleton['position']['left_hand'] - self.skeleton['position']['left_shoulder']
            
            right_shoulder_neck.Normalize()
            right_shoulder_elbow.Normalize()
            rh = right_elbow_hand.Normalize()
            right_shoulder_hand.Normalize()                

            right_shoulder_lift_angle = -(asin(right_shoulder_elbow.y()) + self.HALF_PI)
            right_shoulder_pan_angle = -asin(right_elbow_hand.x())
            right_elbow_angle = acos(KDL.dot(right_shoulder_elbow, right_elbow_hand))
            right_wrist_angle = -right_elbow_angle / 2.0
            right_elbow_angle = right_elbow_angle / 4.0
                            
            self.cmd_joints.position[self.cmd_joints.name.index('right_shoulder_lift_joint')] = right_shoulder_lift_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('right_shoulder_lift_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index('right_shoulder_pan_joint')] = right_shoulder_pan_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('right_shoulder_pan_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index('right_elbow_joint')] = right_elbow_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('right_elbow_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index('right_wrist_joint')] = right_wrist_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('right_wrist_joint')] = self.default_joint_speed
 
            right_arm_roll_angle = -asin(right_shoulder_hand.x())
            self.cmd_joints.position[self.cmd_joints.name.index('right_arm_roll_joint')] = 0
            self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_roll_joint')] = self.default_joint_speed
            
#                right_wrist_angle = acos(min(1, abs((rh - 0.265) / (0.30 - 0.265)))) - QUARTER_PI
#                self.cmd_joints.position[self.cmd_joints.name.index('right_wrist_joint')] = 0
#                self.cmd_joints.velocity[self.cmd_joints.name.index('right_wrist_joint')] = self.default_joint_speed
            
        except KeyError:
            pass          
            
    def skeleton_handler(self, msg):
        for joint in msg.name:  
            self.skeleton['confidence'][joint] = msg.confidence[msg.name.index(joint)]
            self.skeleton['position'][joint] = KDL.Vector(msg.position[msg.name.index(joint)].x, msg.position[msg.name.index(joint)].y, msg.position[msg.name.index(joint)].z)
            self.skeleton['orientation'][joint] = KDL.Rotation.Quaternion(msg.orientation[msg.name.index(joint)].x, msg.orientation[msg.name.index(joint)].y, msg.orientation[msg.name.index(joint)].z, msg.orientation[msg.name.index(joint)].w)
            
    def joint_state_handler(self, msg):
        for joint in msg.name:  
            self.joint_state = msg
            
    def set_command_callback(self, req):
        self.tracker_command = req.command
        return SetCommandResponse()

    def shutdown(self):
        rospy.loginfo('Shutting down Tracker Joint Controller Node.')
        
if __name__ == '__main__':
    try:
        TrackerJointController()
    except rospy.ROSInterruptException:
        pass