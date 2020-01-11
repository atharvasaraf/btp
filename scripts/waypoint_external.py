#!/usr/bin/env python

import rospy
import mavros
import sensor_msgs
import yaml
#from mavros.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

#global variables
latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False

def waypoint_callback(data):
	global last_waypoint
	#print("\n----------\nwaypoint_callback")
	rospy.loginfo("Received waypoint: %s", data)
	if len(data.waypoints) != 0:							#if waypoint list is not empty
		rospy.loginfo("is_current: %s", data.waypoints[len(data.waypoints)-1].is_current)
		last_waypoint = data.waypoints[len(data.waypoints)-1].is_current	#checks status of "is_current" for last waypoint

def globalPosition_callback(data):
	#print("\n----------\nglobalPosition_callback")
	global latitude
	global longitude
	global altitude
	latitude = data.latitude
	longitude = data.longitude
	altitude = data.altitude

def setTakeOff():
	rospy.wait_for_service('/mavros/cmd/takeoff')
	rospy.loginfo("Attempting Takeoff")
	try:
		takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
		takeoffService(altitude = 2.0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
	except rospy.ServiceException, e:
		print "Service Takeoff call failed: %s"%e

def setMode(requested_mode):
	rospy.wait_for_service('/mavros/set_mode')
	rospy.loginfo("Attempting to set mode to: GUIDED")
	try:
		flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
		isModeChanged = flightModeService(custom_mode=requested_mode)
	except rospy.ServiceException, e:
		print "Service set_mode call failed: %s . Requested mode not set"%e

def setGuidedMode():
	rospy.wait_for_service('/mavros/set_mode')
	rospy.loginfo("Attempting to set mode to: GUIDED")
	try:
		flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
		isModeChanged = flightModeService(custom_mode='GUIDED')
	except rospy.ServiceException, e:
		print "Service set_mode call failed: %s . GUIDED mode not set"%e

def setArm():
	rospy.wait_for_service('/mavros/cmd/arming')
	rospy.loginfo("Attempting to Arm throttle")
	try:
		armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
		armService(True)
	except rospy.ServiceException, e:
		print "Service arm call failed: %s"%e

def clearWayPoint():
	rospy.wait_for_service('/mavros/mission/clear')
	rospy.loginfo("Attempting to Clear Way Point")
	try: 
		clearWayPointService = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
		resp = clearWayPointService()
		print(resp)
	except rospy.ServiceException, e:
		print "Service Clear Way Point call failed: %s ."%e

def pullWayPoint():
	rospy.wait_for_service('/mavros/mission/pull')
	rospy.loginfo("Attempting to Pull Waypoint")
	try:
		pullWayPointService = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
		resp = pullWayPointService()
		print resp
	except rospy.ServiceException, e:
		print "Service Pull Way Point call failed: %s."%e

def pushWayPoint():
	rospy.wait_for_service('/mavros/mission/push')
	rospy.loginfo("Attempting to Push Waypoint")
	waypoints = [
		Waypoint(frame = 3, command = 22, is_current =  True, autocontinue = True, param1 = 5, x_lat = 37.197718, y_long = -80.580626, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.197700, y_long = -80.580638, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.197590, y_long = -80.580301, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.197794, y_long = -80.580285, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.197842, y_long = -80.580127, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.197872, y_long = -80.580083, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.197696, y_long = -80.580072, z_alt = 10),
		Waypoint(frame = 3, command = 16, is_current = False, autocontinue = True, param1 = 5, x_lat = 37.197554, y_long = -80.580061, z_alt = 10)
	]
	try:
		pushWayPointService = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
		resp = pushWayPointService(waypoints)
		print resp
	except rospy.ServiceException, e:
		print "Service Push Way Point call failed: %s . GUIDED mode not set"%e

def main():
	rospy.init_node('wayPoint')
	rospy.Subscriber("/mavros/mission/waypoints", WaypointList, waypoint_callback)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPosition_callback)

	# setGuidedMode()

	#Clearing waypoints
	clearWayPoint()
	rospy.sleep(5)

	#Pull Waypoints
	pullWayPoint()
	rospy.sleep(5)

	# Arming
	# setArm()

	#Sending waypoints_push
	pushWayPoint()
	rospy.sleep(5)
	
	while True:						#waits for last_waypoint in previous WaypointList to be visited
		rospy.sleep(2)
		print("WAITING for last_waypoint == True")
		if last_waypoint == True:			#if last_waypoint is in the process of being visited
			while True:
				rospy.sleep(2)
				print("WAITING for last_waypoint == False")
				if last_waypoint == False:	#if last_waypoint has been visited (due to previous constraint)
					break
			break

	print("EVERYTHING WORKED AS PLANNED!!!")
	rospy.spin()

def temp():
	rospy.init_node('tempNode')
	clearWayPoint()
	setMode('GUIDED')
	rospy.spin()

if __name__ == '__main__':
	# main()
	temp()