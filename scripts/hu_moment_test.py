#!/usr/bin/env python

"""
The nomenclature pattern for the class objects, functions, and variables may not be consistent despite sincere efforts made for the same


ROS-Node Summary:
Name: huMoment_node

Node Activity: 
Generate Hu Moments using the subscribed image
Stack Hu Moments and Save data for surface fit

TOPICS:
Subscriptions:
/mavros/local_position/pose Drone Position from local reference frame
/iris/tiltCam/image_raw/imagestream Onboard Camera Live FeedBack
---------------------------------------------------------------------
Publications:

---------------------------------------------------------------------

SERVICES:
Advertised:
/snap_image - Generate HuMoments and store Moments & Position


"""

import cv2
import time
import rospy
import numpy as np
import sensor_msgs
from btp.srv import snap
from math import copysign, log10
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class HuMoment:
	"""
	All functionality of this script is organized inside this class & its methods
	"""
	def __init__(self):
		"""
		Initialize subscribers and advertise services
		Create all used private variables
		Threshold values for HSV segmentation are defined here
		"""
		rospy.init_node('huMoment_node')
		self.pose_subscriber = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
		self.image_subscriber = rospy.Subscriber("/iris/tiltCam/image_raw/imagestream", Image, self.img_callback)
		self.snap_service = rospy.Service('snap_image', snap, self.snap_image)

		rospy.loginfo("Initialized Hu Moment Node... \n Subscribing to Image and Drone Position Topics \n Advertising Service: snap_image")
		# self.HSV_min = np.array([0, 5, 229])
		# self.HSV_max = np.array([179, 255, 255])
		self.HSV_min = np.array([0, 128, 95])
		self.HSV_max = np.array([179, 255, 136])
		
		self.hu_Moments = np.empty([7, 1])
		self.nrmld_Hu = np.empty([7, 1])
		self.sub_image = Image
		self.current_pose = np.empty([3, 1])
		self.surface_data = 0
		self.flag = True

	def snap_image(self, resp):
		"""
		Callback for /snap_image Service

		"""
		if resp.action == 0:
			pop = cv2.cvtColor(self.sub_image, cv2.COLOR_RGB2BGR)
			cv2.imshow('image', pop)
			cv2.waitKey(0)
			cv2.destroyAllWindows()
			cv2.imwrite('/home/fatguru/catkin_ws/src/btp/b.jpg', pop)

		elif resp.action == 1:
			self.sourceImage('default')
			self.getHSVmask()
			self.getHuMoments()
			self.current_pose = np.zeros([3, 1])
			position_data = np.append(self.current_pose, self.hu_Moments).reshape((1, 10))
			
			if self.flag == True:
				self.surface_data = position_data
				self.flag = False
				print self.surface_data
			else:
				self.surface_data = np.append(self.surface_data, position_data, axis = 0)
				print self.surface_data

		print "Was here!"
		print self.saveSurfaceData()
		return 1

	def getHSVmask(self):
		"""
		Generate Binary Image from HSV thresholding
		"""
		self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
		self.mask = cv2.inRange(self.hsv, self.HSV_min, self.HSV_max)

	def getHuMoments(self):
		"""
		Compute Hu Moments for binary image
		"""
		moments = cv2.moments(self.mask)
		self.hu_Moments = cv2.HuMoments(moments)
		
		for i in range(0,7):
			if self.hu_Moments[i] != 0:
  				self.nrmld_Hu[i] = -1* copysign(1.0, self.hu_Moments[i]) * log10(abs(self.hu_Moments[i]))
			else:
				self.nrmld_Hu[i] = 0

	def saveMask(self):
		"""
		Testing Function
		Save binary image
		Function is prolly redundant/Will be removed
		"""
		cv2.imwrite('/home/fatguru/Desktop/mask.jpg', self.mask)

	def saveImage(self):
		"""
		Testing Function
		Save captured Image
		Function is prolly redundant/Will be removed
		"""
		temp = cv2.cvtColor(self.sub_image, cv2.COLOR_BGR2RGB)
		cv2.imwrite('/home/fatguru/Desktop/image.jpg', temp)

	def showMask(self):
		"""
		Testing Function
		Visualize generate binary image
		Will be removed
		"""
		cv2.imshow('mask', self.mask)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

	def printHuMoments(self):
		print "Hu-Moments Obtained are:"
		for i in range(len(self.hu_Moments)):
			print "[H_%s]"%i," = %s"%self.hu_Moments[i]

		print "Normalized Hu-Moments Obtained are:"
		for i in range(len(self.hu_Moments)):
			print "[H_%s]"%i," = %s"%self.nrmld_Hu[i]

	def img_callback(self, ros_data):
		"""
		Callback for image topic
		Stores image in private data structure
		"""
		np_arr = np.fromstring(ros_data.data, np.uint8).reshape(ros_data.height, ros_data.width, 3)
		raw = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		self.sub_image = np_arr

	def pose_callback(self, data):
		"""
		Callback for local drone position topic
		Stores pose in private data structure
		"""
		self.current_pose[0] = data.pose.position.x
		self.current_pose[1] = data.pose.position.y
		self.current_pose[2] = data.pose.position.z

	def sourceImage(self, topic='default'):
		"""
		Function to define source of image for Hu Moment extraction	
		"""
		if topic == 'default':
			self.image = cv2.imread('/home/fatguru/catkin_ws/src/btp/a.jpg')
		elif topic == 'ROS':
			self.image = self.sub_image

	def fetchAndUpdate(self):
		"""
		Testing Function
		Will be removed
		"""
		self.sourceImage('ROS')
		self.getHSVmask()
		self.getHuMoments()
		self.printHuMoments()
		self.showMask()

	def saveSurfaceData(self):
		"""
		Method to store obtained data for surface generation
		TODO:
			Integrate method into a service (maybe using a wrapper)
		"""
		print "Saving data!"
		filename = time.strftime("%Y%m%d-%H%M%S")
		filepath = '/home/fatguru/catkin_ws/src/btp/data/' + filename
		np.savetxt(filepath, self.surface_data, delimiter=',')

def main():
	HuNode = HuMoment()
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass	