#!/usr/bin/env python

"""
The nomenclature pattern for the class objects, functions, and variables may not be consistent despite sincere efforts made for the same


ROS-Node Summary:
Name: waypoint_node

Node Activity: 
Commander takeoff of drone
Periodically update waypoints to navigate drone along desired trajectory
Call service advertised by huMoment_node to process image data at each waypoint
Call service advertised by huMoment_node to save raw data for surface fit

TOPICS:
Subscriptions:

---------------------------------------------------------------------
Publications:
/mavros/setpoint_position/local Publish desired waypoint to mavros

---------------------------------------------------------------------

SERVICES:
Advertised:

---------------------------------------------------------------------
Called:
/mavros/set_mode Set desired mode of UAV
mavros/cmd/arming Arm UAV
mavros/cmd/takeoff Takeoff UAV
/snap_image Generate Hu Moments from current location and stack data
/save_data Save generated data of hu moments and corresponding positions

"""

import rospy
import mavros
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from btp.srv import snap
from btp.srv import cherry
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

class mission_sitl:
	"""
	All functionality of this script is organized inside this class & its methods
	"""
	def __init__(self):
		"""
		Initalize Node & Publishers
		Setpoints are given here
		Takeoff UAV & move along trajectory
		Call Services for Image processing and data storage
		"""
		rospy.init_node('wayPointPublisher_node')
		self.setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		rospy.sleep(1)

		self.setpoint_list = [
		[0.00, 0.0, 12],
		[5.00, 0.0, 12],
		[10.0, 0.0, 12],
		[15.0, 0.0, 12],
		[20.0, 0.0, 12],
		[25.0, 0.0, 12],
		[30.0, 0.0, 12],
		[35.0, 0.0, 12],
		[40.0, 0.0, 12],
		[45.0, 0.0, 12],
		[50.0, 0.0, 12],
		[55.0, 0.0, 12],
		[60.0, 0.0, 12],

		[50.0, 0.0, 15],
		[45.0, 0.0, 15],
		[40.0, 0.0, 15],
		[30.0, 0.0, 15],
		[35.0, 0.0, 15],
		[30.0, 0.0, 15],
		[25.0, 0.0, 15],
		[20.0, 0.0, 15],
		[15.0, 0.0, 15],
		[10.0, 0.0, 15],
		[5.00, 0.0, 15],
		[0.00, 0.0, 15],

		[0.00, 0.0, 18],
		[5.00, 0.0, 18],
		[10.0, 0.0, 18],
		[15.0, 0.0, 18],
		[20.0, 0.0, 18],
		[25.0, 0.0, 18],
		[30.0, 0.0, 18],
		[35.0, 0.0, 18],
		[40.0, 0.0, 18],

		[30.0, 0.0, 21],
		[35.0, 0.0, 21],
		[30.0, 0.0, 21],
		[25.0, 0.0, 21],
		[20.0, 0.0, 21],
		[15.0, 0.0, 21],
		[10.0, 0.0, 21],
		[5.00, 0.0, 21],
		[0.00, 0.0, 21],

		[0.00, 0.0, 24],
		[5.00, 0.0, 24],
		[10.0, 0.0, 24],
		[15.0, 0.0, 24],
		[20.0, 0.0, 24],

		[10.0, 0.0, 27],
		[5.00, 0.0, 27],
		[0.00, 0.0, 27]
		]

		self.setpoint = PoseStamped()
		self.setMode('GUIDED')
		
		rospy.sleep(1)
		
		self.setArm()
		self.setTakeOff(15)
		rospy.sleep(15)
		for i in range(len(self.setpoint_list)):
			self.update_setpoint(self.setpoint, i)
			rospy.sleep(7)
			self.captureImage()
			rospy.sleep(3)

		self.setMode('RTL')
		self.saveData()

	def update_setpoint(self, setpoint, index):
		"""
		Update current setpoint
		Function is a wrapper with Logging added
		"""
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
		"""
		set mode of UAV to desired
		Function is a wrapper with Logging added
		"""
		rospy.wait_for_service('/mavros/set_mode')
		rospy.loginfo("Attempting to set mode to: %s"%requested_mode)
		try:
			flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
			isModeChanged = flightModeService(custom_mode=requested_mode)
		except rospy.ServiceException, e:
			rospy.logerr("Service set_mode call failed: %s . Requested mode not set"%e)

	def setArm(self):
		"""
		Arm UAV
		Function is a wrapper with Logging added
		"""
		rospy.wait_for_service('/mavros/cmd/arming')
		rospy.loginfo("Attempting to Arm throttle")
		try:
			armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException, e:
			rospy.logerr("Service arm call failed: %s"%e)

	def setTakeOff(self, commanded_altitude):
		"""
		Takeoff UAV
		Function is a wrapper with Logging added
		"""
		rospy.wait_for_service('/mavros/cmd/takeoff')
		rospy.loginfo("Attempting Takeoff to altitude: [%sm]"%commanded_altitude)
		try:
			takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
			takeoffService(altitude = commanded_altitude, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
		except rospy.ServiceException, e:
			rospy.logerr("Service Takeoff call failed: %s"%e)

	def captureImage(self):
		"""
		Process current camera feed to obtain hu_moments 
		"""
		rospy.wait_for_service('/snap_image')
		rospy.loginfo("Processing current available image")
		try:
			captureService = rospy.ServiceProxy('/snap_image', snap)
			response = captureService(action = 1)
		except rospy.ServiceException, e:
			rospy.logerr("HuMoment Service call failed")

	def saveData(self):
		"""
		Save all generated data
		"""
		rospy.wait_for_service('/save_data')
		rospy.loginfo("Saving all generated data")
		try:
			captureService = rospy.ServiceProxy('/save_data', cherry)
			response = captureService(task = "save data")
		except rospy.ServiceException, e:
			rospy.logerr("HuMoment Service call failed")


def main():
	instance = mission_sitl()
	rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass