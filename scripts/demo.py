#!/usr/bin/env python
import rospy
import tf
from mavros_msgs.msg import *
from mavros_msgs.srv import *

rospy.init_node('simple_takeoff', anonymous=True)

def setGuidedMode():
	rospy.wait_for_service('/mavros/set_mode')
	try:
		flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
		isModeChanged = flightModeService(custom_mode='GUIDED')
	except rospy.ServiceException, e:
		print "Service set_mode call failed: %s . GUIDED mode not set"%e

def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e

if __name__ == '__main__':
	rospy.init_node('simple_takeoff', anonymous=True)
	setGuidedMode();
	setArm();
