#!/usr/bin/env python
"""
The nomenclature pattern for the class objects, functions, and variables may not be consistent despite sincere efforts made for the same

Landing Module for UAV
Tasks:
1] Obtain learned Hu Moment from least square-fit surface
2] Calculate the diameter of landing marker from dome -- Projected Diameter has been obtained
3] Infer position of landing marker

To do:
4] Push landing command to drone

Consider two diametrically opposite points of the landing marker: P1 & P2
The two extremum points of the image of the landing marker are diametrically opposite, because
the longest line in the hemisphere is the diameter of the largest circle.
This diameter is also parallel to the image plane and perpendicular to the principal axis of the camera lens

We find the angle between the two lines (P1f and P2f), (where f is the focus) by finding all three sides of the triangle P1fP2
Then using equation (17) from the paper we find the Absolute distance(z_Dome) of the Camera from landing marker

"""

import os
import cv2
import time
import math
import rospy
import imutils
import numpy as np 
import sensor_msgs
from math import copysign, log10
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

# 60, 12
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
		self.diameter = 4.00

		self.HSV_min = np.array([0, 128, 95])
		self.HSV_max = np.array([179, 255, 136])

		# Focal length is calculated using Image Width(640) and Horizontal Field of View (1.0472) and formula:
		# f = (img_width / 2) / ( tan(deg2rad(h_fieldOfView) / 2) )
		self.img_width = 640
		self.img_height = 400
		self.centreImageX = self.img_width / 2
		self.centreImageY = self.img_height / 2
		h_fov = 1.0472 #(In radians use np.deg2rad() to convert)
		self.focal_length = (self.img_width / 2) / ( math.tan( h_fov / 2 ) ) 
		
		self.bufferLength = 4
		self.HuMomentsFromImageBuffer = None
		self.HuMomentsFromEquationBuffer = None

		self.sourceImage()

		self.getHSVmask()
		self.getExtremePoints()
		self.getCentrePoint()
		self.getDistanceExtremePoints()
		self.getDiameterImagePlane()
		self.getAngleFromFocus()
		
		self.getDistanceLandingMarkerFromFocus()
		self.getHeightDistanceFromFocus()
		
		self.loadSurfaceCoeff()
		self.getMomentFromEquation()
		self.getMomentFromImage()
		# self.loadHuMomentsToBuffer()
		# self.showExtremePoints()

	def getExtremePoints(self):
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
 		To visualize the obtained extremum points(Image)
 		"""
 		cv2.circle(self.image, self.extMin, 8, (0, 0, 255), -1)
 		cv2.circle(self.image, self.extMax, 8, (0, 255, 0), -1)
		self.showImage()

	def getDiameterImagePlane(self):
		"""
		Simple trignometric calculation to obtain distance between extremum points(Image)
		"""
		self.diameterImagePlane = math.sqrt((self.extMax[0] - self.extMin[0])**2 + (self.extMax[1] - self.extMin[1])**2)

	def getCentrePoint(self):
		# From Top Left Corner
		self.centreX = (self.extMax[0] + self.extMin[0]) / 2
		self.centreY = (self.extMax[1] + self.extMin[1]) / 2
		# From centre
		self.centreX = self.centreX - (self.img_width / 2)
		self.centreY = self.centreY - (self.img_height / 2)

	def getHSVmask(self):
		"""
		Generate Binary Image from HSV thresholding
		"""
		self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
		self.mask = cv2.inRange(self.hsv, self.HSV_min, self.HSV_max)
	
	def getDistanceExtremePoints(self):
		"""
		Calculate the distance of the extremum points from the centre of the Image (oP1 and oP2)
		Calculate the distance of the extremum points from the focus (oP1 and oP2)
		"""		
		# Distance of extremum points from centre of Image (oP1 and oP2)
		self.extMaxFromCentre = math.sqrt( (self.extMax[0] - self.centreImageX)**2 + (self.extMax[1] - self.centreImageY)**2 )
		self.extMinFromCentre = math.sqrt( (self.extMin[0] - self.centreImageX)**2 + (self.extMin[1] - self.centreImageY)**2 )
		
		# Distance of extremum points from focus (fP1 anf fP2)
		self.extMaxfromFocus = math.sqrt( self.extMaxFromCentre**2 + self.focal_length**2 )
		self.extMinfromFocus = math.sqrt( self.extMinFromCentre**2 + self.focal_length**2 )

	def getAngleFromFocus(self):
		"""
		Calculate the angle between the lines passing through focus intersecting the two extreme points on the diameter
		Using theta = arccos(b**2 + c**2 - a**2 / 2*b*c)
		"""
		self.theta = np.arccos( (self.extMaxfromFocus**2 + self.extMinfromFocus**2 - self.diameterImagePlane**2) / (2* self.extMinfromFocus* self.extMaxfromFocus) )

	def getDistanceLandingMarkerFromFocus(self):
		"""
		Get the distance of the Landing Marker from Focus
		Equation (17) in Paper 
		"""
		self.zDome = self.diameter / (2* np.tan(self.theta/ 2))

	def getHeightDistanceFromFocus(self):
		"""
		Calculate the Height And Distance of the landing marker from the Focus ( i.e. from the camera)
		Equation (19) from the paper
		"""
		self.alpha = np.arctan( (self.centreY) / math.sqrt(self.focal_length**2 + self.centreX**2))
		self.heightFromMarker = self.zDome* np.sin(self.alpha)
		self.distanceFromMarker = self.zDome* np.cos(self.alpha)

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
			self.image = cv2.imread('/home/fatguru/catkin_ws/src/btp/raw_data/image/20200427-164100/20200427-165008.jpg')
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

	def loadSurfaceCoeff(self):
		"""
		Load coefficients of least squares surface obtained
		"""
		self.surface_coeff = np.loadtxt('/home/fatguru/catkin_ws/src/btp/learned_moments/20200515-145019', delimiter=',')

	def getMomentFromEquation(self):
		"""
		Load HuMoment from Equation
		"""
		x0 = self.distanceFromMarker
		x1 = self.heightFromMarker
		x0_square = x0**2
		x1_square = x1**2
		x0_x1 = x0*x1
		vec = np.array([[x0, x1, x0_square, x1_square, x0_x1, 1]])
		self.HuMomentsFromEquation = np.dot(vec, self.surface_coeff).T
		
		# print "Equation", self.HuMomentsFromEquation.shape
		# print self.HuMomentsFromEquation

	def getMomentFromImage(self):
		"""
		Compute Hu Moments for binary image
		"""
		moments = cv2.moments(self.mask)
		self.HuMomentsFromImage = cv2.HuMoments(moments)
		
		# Pick only the first 4 Moments
		self.HuMomentsFromImage = self.HuMomentsFromImage[0:4]
		
		# print "Image", self.HuMomentsFromImage.shape
		# print self.HuMomentsFromImage	

	def loadHuMomentsToBuffer(self):
		if self.HuMomentsFromEquationBuffer == None:
			self.HuMomentsFromEquationBuffer = self.HuMomentsFromEquation
		else:
			self.HuMomentsFromEquationBuffer = np.append(self.HuMomentsFromEquation, self.HuMomentsFromEquationBuffer, axis = 1)
		
		if self.HuMomentsFromEquationBuffer.shape[1] > self.bufferLength
			self.HuMomentsFromEquationBuffer = np.delete(self.HuMomentsFromEquationBuffer, self.bufferLength - 1, 1)

		if self.HuMomentsFromImageBuffer == None:
			self.HuMomentsFromImageBuffer = self.HuMomentsFromImage
		else:
			self.HuMomentsFromImageBuffer = np.append(self.HuMomentsFromImage, self.HuMomentsFromImageBuffer, axis = 1)
		
		if self.HuMomentsFromImageBuffer.shape[1] > self.bufferLength
			self.HuMomentsFromImageBuffer = np.delete(self.HuMomentsFromImageBuffer, self.bufferLength - 1, 1)

	def getMeanAndStandardDeviation(self):
		
		self.HuImageMean = np.mean(self.HuMomentsFromImageBuffer, axis=1)
		self.HuImageStdDeviation = np.std(self.HuMomentsFromImageBuffer, axis=1)

		self.HuEquationMean = np.mean(self.HuMomentsFromEquationBuffer, axis=1)
		self.HuEquationStdDeviation = np.std(self.HuMomentsFromEquationBuffer, axis=1)

	def confirmDome(self):

		# lowerLimit = (0.9* HuEquationMean) 
		# upperLimit = (1.1* HuEquationMean) 
		lowerLimit = (0.9* HuEquationMean) - (self.HuImageStdDeviation / 2)
		upperLimit = (1.1* HuEquationMean) + (self.HuImageStdDeviation / 2)

		if ((self.HuImageMean >= lowerLimit).all() and (self.HuImageMean <= upperLimit).all()):
			self.confirmation = True
			print "Dome Confirmed, Landing Command can be issued."
		else:
			self.confirmation = False

def main():
	"""
	Main Function
	"""
	instance = LandingModule()

if __name__ == '__main__':
	main()