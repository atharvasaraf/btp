#!/usr/bin/env python
"""
The nomenclature pattern for the class objects, functions, and variables may not be consistent despite sincere efforts made for the same

Landing Module for UAV
Tasks:
1] Obtain learned Hu Moment from least square-fit surface
2] Calculate the diameter of landing marker from dome -- Projected Diameter has been obtained
3] Infer position of landing marker
4] Push landing command to drone

"""

import numpy as np 
import cv2
import imutils
import math

class LandingModule:
	def __init__(self):
		"""
		Initialization for Class
		Initialize ROS Node
		Define Subscriber and Publisher Topics with Callback functions
		Define values of Hyperparametes like HSV Thresholding etc

		Outline general flow of script
		"""
		# rospy.init_node('landing_node')
		# self.image_subscriber = rospy.Subscriber("/iris/tiltCam/image_raw/imagestream", Image, self.img_callback)

		self.HSV_min = np.array([0, 128, 95])
		self.HSV_max = np.array([179, 255, 136])
		
		self.sourceImage()
		self.getHSVmask()
		self.findExtremePoints()
		self.calculateExtremeDistance()
		# self.showExtremePoints()

	def findExtremePoints(self):
		"""
		Find Contours in the generated Binary Image
		Grab the contour point on the extreme left and on the extreme right (These two points are projections of the diameter in the ImagePlane)

		"""
		# thresh = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
		# thresh = cv2.threshold(gray, 45, 255, cv2.THRESH_BINARY)[1]
		# thresh = self.image
		thresh = self.mask
		cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		c = max(cnts, key=cv2.contourArea)

 		self.extMin = tuple(c[c[:, :, 0].argmin()][0])	
 		self.extMax = tuple(c[c[:, :, 0].argmax()][0])

	def showExtremePoints(self):	
 		"""
 		To visualize the obtained extremum points
 		"""
 		cv2.circle(self.image, self.extMin, 8, (0, 0, 255), -1)
 		cv2.circle(self.image, self.extMax, 8, (0, 255, 0), -1)
		self.showImage()

	def calculateExtremeDistance(self):
		"""
		Simple trignometric calculation to obtain distance between extremum points
		"""
		self.distance = math.sqrt((self.extMax[0] - self.extMin[0])**2 + (self.extMax[1] - self.extMin[1])**2)

	def getHSVmask(self):
		"""
		Generate Binary Image from HSV thresholding
		"""
		self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
		self.mask = cv2.inRange(self.hsv, self.HSV_min, self.HSV_max)

	def showImage(self):
		"""
		Show image obtained on topic
		"""
		cv2.imshow('image', self.image)
		cv2.waitKey(0)

	def sourceImage(self, topic='default'):
		"""
		Function to define source of image for Hu Moment extraction	
		"""
		if topic == 'default':
			self.image = cv2.imread('/home/fatguru/catkin_ws/src/btp/raw_data/image/20200404-001016/20200404-001721.jpg')
			# self.image = cv2.imread('/home/fatguru/catkin_ws/src/btp/raw_data/mask/20200404-001016/20200404-001721.jpg')
		elif topic == 'ROS':
			self.image = self.sub_image

	def img_callback(self, ros_data):
		"""
		Callback for image_Subcriber topic
		Saves topic image at self.sub_image
		"""
		np_arr = np.fromstring(ros_data.data, np.uint8).reshape(ros_data.height, ros_data.width, 3)
		raw = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		self.sub_image = np_arr
	

def main():
	"""
	Main Function
	"""
	instance = LandingModule()

if __name__ == '__main__':
	main()