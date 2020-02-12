#!/usr/bin/env python
import rospy
import mavros
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
class mission_sitl:
	def __init__(self):
		self.setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		rospy.sleep(1)

		self.setpoint_list = [
		[5.00, 0.0, 10.0],
		[10.0, 0.0, 10.0],
		[15.0, 0.0, 10.0],
		[20.0, 0.0, 10.0],
		[20.0, 0.0, 20.0],
		[15.0, 0.0, 20.0],
		[10.0, 0.0, 20.0],
		[5.00, 0.0, 20.0]
		]

		self.setpoint = PoseStamped()
		self.setMode('GUIDED')
		
		rospy.sleep(1)
		
		self.setArm()
		self.setTakeOff(10)
		rospy.sleep(15)
		for i in range(len(self.setpoint_list)):
			self.update_setpoint(self.setpoint, i)
			rospy.sleep(7)
		

		self.setMode('RTL')
		rospy.spin()

	def update_setpoint(self, setpoint, index):
		setpoint.pose.orientation.x = 0.0
		setpoint.pose.orientation.y = 0.0
		setpoint.pose.orientation.z = 0.0
		setpoint.pose.orientation.w = 1.0

		setpoint.pose.position.x = self.setpoint_list[index][0]
		setpoint.pose.position.y = self.setpoint_list[index][1]
		setpoint.pose.position.z = self.setpoint_list[index][2]
		setpoint.header.stamp = rospy.Time.now()
		rospy.loginfo("Setpoint Updated, Publishing index: [%s]", index)
		self.setpoint_publisher.publish(setpoint)

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
		rospy.loginfo("Attempting to Arm throttle")
		try:
			armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException, e:
			print "Service arm call failed: %s"%e

	def setTakeOff(self, commanded_altitude):
		rospy.wait_for_service('/mavros/cmd/takeoff')
		rospy.loginfo("Attempting Takeoff to altitude: [%sm]"%commanded_altitude)
		try:
			takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
			takeoffService(altitude = commanded_altitude, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
		except rospy.ServiceException, e:
			print "Service Takeoff call failed: %s"%e

def main():
	rospy.init_node('wayPointPublisher_node')
	instance = mission_sitl()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass