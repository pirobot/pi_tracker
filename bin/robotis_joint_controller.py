#!/usr/bin/env python

"""
    robotis_joint_controller.py - Version 1.0 2010-12-28
    
    Joint Controller for AX-12 servos on a USB2Dynamixel device using
    the Robotis Controller Package from the Healthcare Robotics Lab at Georgia Tech
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

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

import roslib; roslib.load_manifest('robotis')
import rospy
import servo_config as sc
from robotis.lib_robotis import Robotis_Servo, USB2Dynamixel_Device
from robotis.ros_robotis import ROS_Robotis_Server, ROS_Robotis_Client
from sensor_msgs.msg import JointState
        
class joint_controller():
    def __init__(self):
        rospy.init_node('joint_controller')
        
        rospy.on_shutdown(self.shutdown)
        
        joint_state_pub = rospy.Publisher('/joint_states', JointState)
        
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.rate = rospy.get_param('~rate', 5)
        r = rospy.Rate(self.rate)
        
        """ Read in the servo_config.py file. """
        self.servo_param = sc.servo_param
                
        self.controllers = dict()
        self.joints = dict()
        self.ids = list()
        
        """ Read servo ids and joint names from the servo config file. """
        for id in self.servo_param.keys():
            joint = self.servo_param[id]['name']
            self.joints[id] = joint
            self.ids.append(id)

        """ Connect to the USB2Dynamixel controller """
        usb2dynamixel = USB2Dynamixel_Device(self.port)

        """ Fire up the ROS joint server processes. """
        servos = [ Robotis_Servo( usb2dynamixel, i ) for i in self.ids ]
        ros_servers = [ ROS_Robotis_Server( s, str(j) ) for s, j in zip(servos, self.joints.values()) ]
        
        """ Fire up the ROS services. """
        for joint in self.joints.values():
            self.controllers[joint] = ROS_Robotis_Client(joint)
        
        """ Start the joint command subscriber """
        rospy.Subscriber('cmd_joints', JointState, self.cmd_joints_handler) 
        
        while not rospy.is_shutdown():
            """ Create a JointState object which we use to publish the current state of the joints. """
            joint_state = JointState()
            joint_state.name = list()
            joint_state.position = list()
            joint_state.velocity = list()
            joint_state.effort = list()
            for s in ros_servers:
                joint_state.name.append(s.name)
                joint_state.position.append(s.update_server())
                """ The robotis driver does not currently query the speed and torque of the servos,
                    so just set them to 0. """
                joint_state.velocity.append(0)
                joint_state.effort.append(0)
            joint_state.header.stamp = rospy.Time.now()
            joint_state_pub.publish(joint_state)
            r.sleep()
            
    def cmd_joints_handler(self, req):
        for joint in self.joints.values():
            try:
                self.controllers[joint].move_angle(req.position[req.name.index(joint)],
                max(0.01, req.velocity[req.name.index(joint)]), blocking=False)
            except:
                pass
            
    def shutdown(self):
        rospy.loginfo('Shutting down joint command controller node...')

            
if __name__ == '__main__':
    try:
        jc = joint_controller()
    except rospy.ROSInterruptException:
        pass



