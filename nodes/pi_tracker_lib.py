#!/usr/bin/env python

"""
    Common functions shared by nodes in the pi_tracker package.
    Can be used with the OpenNI tracker package in junction with a
    Kinect RGB-D camera.
        
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
from sensor_msgs.msg import JointState
import xml.dom.minidom
from math import pi
import PyKDL as KDL
    
def get_body_dimension(skeleton, joint1, joint2, default_length):
    try:
        # Compute a body dimension to use to scale gesture measurements.
        if skeleton['confidence'][joint1] > 0.5 and skeleton['confidence'][joint2] > 0.5:
            # The KDL Normalize() function returns the length of the vector being normalized.
            length = (skeleton['position'][joint1] - skeleton['position'][joint2]).Normalize()
            if length == 0:
                length = default_length
        else:
            length = default_length
    except:
         length = default_length
        
    return length
    
def get_joints():
    """ This function is take from the joint_state_publisher package written by David Lu!!
        See http://www.ros.org/wiki/joint_state_publisher
    """
    description = rospy.get_param("robot_description")
    robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
    free_joints = {}
    joint_list = [] # for maintaining the original order of the joints
    dependent_joints = rospy.get_param("dependent_joints", {})
    
    # Find all non-fixed joints.
    for child in robot.childNodes:
        if child.nodeType is child.TEXT_NODE:
            continue
        if child.localName == 'joint':
            jtype = child.getAttribute('type')
            if jtype == 'fixed':
                continue
            name = child.getAttribute('name')

            if jtype == 'continuous':
                minval = -pi
                maxval = pi
            else:
                limit = child.getElementsByTagName('limit')[0]
                minval = float(limit.getAttribute('lower'))
                maxval = float(limit.getAttribute('upper'))

            if name in dependent_joints:
                continue
            if minval > 0 or maxval < 0:
                zeroval = (maxval + minval)/2
            else:
                zeroval = 0

            joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
            free_joints[name] = joint
            joint_list.append(name)

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()

    # Add Free Joints.
    for (name, joint) in free_joints.items():
        joint_state.name.append(str(name))
        joint_state.position.append(joint['value'])
        joint_state.velocity.append(0)
        
    return joint_state
        

    