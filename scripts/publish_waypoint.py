#!/usr/bin/env python
import rospy
import mavros
import sensor_msgs
import yaml
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class mission_sitl:
	def __init__(self):
		self.latitude = 0.0
		self.longitude = 0.0
		self.altitude = 0.0
		self.last_waypoint = False
		
		rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self.waypoint_callback)
		rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.globalPosition_callback)
	
		self.clearWayPoint()
		rospy.sleep(1)

		self.pullWayPoint()
		rospy.sleep(1)

		self.setMode('GUIDED')
		self.pushWayPoint()
		self.setArm()
		self.setTakeOff(10)
		rospy.sleep(5)

		# self.setMode('AUTO')

		while True:						#waits for last_waypoint in previous WaypointList to be visited
			rospy.sleep(2)
			print("WAITING for last_waypoint == True")
			if self.last_waypoint == True:			#if last_waypoint is in the process of being visited
				while True:
					rospy.sleep(2)
					print("WAITING for last_waypoint == False")
					if self.last_waypoint == False:	#if last_waypoint has been visited (due to previous constraint)
						# self.setMode('RTL')
						break
				break

		rospy.spin()

	def waypoint_callback(self, data):
		rospy.loginfo("Received waypoint: %s", data)
		if len(data.waypoints) != 0:							#if waypoint list is not empty
			rospy.loginfo("is_current: %s", data.waypoints[len(data.waypoints)-1].is_current)
			self.last_waypoint = data.waypoints[len(data.waypoints)-1].is_current	#checks status of "is_current" for last waypoint

	def globalPosition_callback(self, data):
		self.altitude = data.altitude
		self.latitude = data.latitude
		self.longitude = data.longitude

	def clearWayPoint(self):
		rospy.wait_for_service('/mavros/mission/clear')
		rospy.loginfo("Attempting to Clear Way Point")
		try: 
			clearWayPointService = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
			resp = clearWayPointService()
			print(resp)
		except rospy.ServiceException, e:
			print "Service Clear Way Point call failed: %s ."%e

	def pullWayPoint(self):
		rospy.wait_for_service('/mavros/mission/pull')
		rospy.loginfo("Attempting to Pull Waypoint")
		try:
			pullWayPointService = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
			resp = pullWayPointService()
			print resp
		except rospy.ServiceException, e:
			print "Service Pull Way Point call failed: %s."%e

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

	def pushWayPoint(self):
		rospy.wait_for_service('/mavros/mission/push')
		rospy.loginfo("Attempting to Push Waypoint")
		self.waypoints = [
		Waypoint(frame = 3, command = 16, is_current = True, autocontinue = True, param1 = 0, param2 = 0, param3 = 0, x_lat = -35.3632538, y_long = 149.1652414, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 0, param2 = 0, param3 = 0, x_lat = -35.3632738, y_long = 149.1652414, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 0, param2 = 0, param3 = 0, x_lat = -35.3632838, y_long = 149.1652414, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 0, param2 = 0, param3 = 0, x_lat = -35.3633038, y_long = 149.1652414, z_alt = 10)
		]
		# self.waypoints = [
		# 	#WAYPOINTS 1 & 2 are the same!
		# 	# Waypoint(frame = 3, command = 16, is_current =  False, autocontinue = True, param1 = 5, x_lat = -35.3632538, y_long = 149.1652414, z_alt = 10),
		# 	Waypoint(frame = 3, command = 16, is_current =  True, autocontinue = True, x_lat = -35.3632538, y_long = 149.1652414, z_alt = 10),
		# 	Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, x_lat = -35.3632638, y_long = 149.1652514, z_alt = 10),
		# 	Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, x_lat = -35.3632738, y_long = 149.1652614, z_alt = 10),
		# 	Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, x_lat = -35.3632838, y_long = 149.1652714, z_alt = 10),
		# 	Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, x_lat = -35.3632938, y_long = 149.1652814, z_alt = 10),
		# 	Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, x_lat = -35.3633038, y_long = 149.1652914, z_alt = 10),
		# 	Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, x_lat = -35.3633138, y_long = 149.1653014, z_alt = 10),
		# 	Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, x_lat = -35.3633438, y_long = 149.1653414, z_alt = 10)
		# ]
		try:
			pushWayPointService = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
			resp = pushWayPointService(start_index=0, waypoints=self.waypoints)
			print resp
		except rospy.ServiceException, e:
			print "Service Push Way Point call failed: %s . GUIDED mode not set"%e

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