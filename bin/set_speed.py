#!/usr/bin/env python

import roslib; roslib.load_manifest('pi_tracker')
import rospy
import sys

from ax12_controller_core.srv import SetSpeed

class SetArmSpeed():
    def __init__(self, speed):
        rospy.init_node('set_arm_speed')
        
        rospy.loginfo("Setting arm speed to " + str(speed))

        max_speed = 0.7
        if speed > max_speed:
            speed = max_speed
        
        self.servo_speed = dict()
        ax12_namespace = '/ax12_controller'
        dynamixels = rospy.get_param(ax12_namespace+'/dynamixels', dict())
        for name in sorted(dynamixels):
            rospy.wait_for_service(ax12_namespace+'/'+name+'_controller/set_speed')  
            self.servo_speed[name] = rospy.ServiceProxy(ax12_namespace+'/'+name+'_controller/set_speed', SetSpeed)
            try:
                self.servo_speed[name](speed)
            except:
                pass
        
if __name__ == '__main__':
    try:
        SetArmSpeed(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass