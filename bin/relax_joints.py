#!/usr/bin/env python

import roslib; roslib.load_manifest('pi_tracker')
import rospy
import sys

from ax12_controller_core.srv import SetTorqueLimit

class RelaxJoints():
    def __init__(self, enable):
        rospy.init_node('relax_joints')
        self.controller_namespace = rospy.get_param('controller_namespace', '/ax12_controller')

        self.set_torque_limit = dict()
        dynamixels = rospy.get_param(self.controller_namespace+'/dynamixels', dict())
        for name in sorted(dynamixels):
            rospy.wait_for_service(self.controller_namespace+'/'+name+'_controller/set_torque_limit')  
            self.set_torque_limit[name] = rospy.ServiceProxy(self.controller_namespace+'/'+name+'_controller/set_torque_limit', SetTorqueLimit)
            rospy.loginfo(name)
            self.set_torque_limit[name](0.0)
        
if __name__ == '__main__':
    try:
        RelaxJoints(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass