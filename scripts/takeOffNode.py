#!/usr/bin/env python
import rospy
import mavros
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class takeoff_demo:
	def __init__(self):
		rospy.loginfo("Ensure that EKF2 is using IMU0 & IMU1")
		rospy.sleep(2)
		self.setMode('GUIDED')
		self.setArm()
		self.setTakeOff(5)
		rospy.spinOnce()

	def setMode(self, requested_mode):
		rospy.wait_for_service('/mavros/set_mode')
		rospy.loginfo("Attempting to set mode to: %s"%requested_mode)
		try:
			flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
			isModeChanged = flightModeService(custom_mode=requested_mode)
		except rospy.ServiceException, e:
			print "Service set_mode call failed: %s . Requested mode not set"%e

	def setArm(self):
	    rospy.wait_for_service('/mavros/cmd/arming')
	    try:
	        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
	        armService(True)
	    except rospy.ServiceException, e:
	        print "Service arm call failed: %s"%e


	def setTakeOff(self, commanded_altitude):
		rospy.wait_for_service('/mavros/cmd/takeoff')
		rospy.loginfo("Attempting Takeoff to altitude: [%s m]"%commanded_altitude)
		try:
			takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
			takeoffService(altitude = commanded_altitude, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
		except rospy.ServiceException, e:
			print "Service Takeoff call failed: %s"%e

def main():
	rospy.init_node('simpleTakeOffDemo', anonymous=True)
	instance = takeoff_demo()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
