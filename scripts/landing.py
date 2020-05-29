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
from btp.srv import identifyDome
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class LandingModule:
	def __init__(self):
		"""
		Initialization for Class
		Initialize ROS Node
		Define Subscriber and Publisher Topics with Callback functions
		Define values of Hyperparametes like HSV Thresholding etc

		Outline general flow of script
		"""
		rospy.init_node('landing_node')
		self.pose_subscriber = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.poseCallback)
		self.image_subscriber = rospy.Subscriber("/iris/tiltCam/image_raw/imagestream", Image, self.imgCallback)
		self.identification_service = rospy.Service("landing_marker_identification_service", identifyDome, self.identifyLandingMarker)

		self.diameter = 4.00
		self.HSV_min = np.array([0, 128, 95])
		self.HSV_max = np.array([179, 255, 136])

		# Focal length is calculated using Image Width(640) and Horizontal Field of View (1.0472) and formula:
		# f = (img_width / 2) / ( tan(deg2rad(h_fieldOfView) / 2) )
		self.img_width = 640
		self.img_height = 400
		self.centreImageX = self.img_width * 0.5
		self.centreImageY = self.img_height * 0.5
		h_fov = 1.0472 #(In radians use np.deg2rad() to convert)
		self.focal_length = (self.img_width * 0.5) / ( math.tan( h_fov * 0.5) ) 
		self.camera_tilt = 0.4 #Enter the downward tilt of the camera in radians

		self.bufferLength = 4
		self.HuMomentsFromImageBuffer = []
		self.HuMomentsFromEquationBuffer = []
		
		self.stack = []
		self.current_pose = np.array([0,0,0])
		# self.processImage()

	def identifyLandingMarker(self, data):
		if data.request == 'identify':
			rospy.loginfo("verification of landing marker in progress")
			self.processImage()
			return "Service callback for identification has ended"
		elif data.request =='save':
			rospy.loginfo("request to save data")
			return self.saveStack()
		return "Invalid data request, service callback has ended"

	def processImage(self):
		"""
		All Image Processing and Data Collection and Matching is done here.
		"""
		self.sourceImage('ROS')

		self.getHSVmask()
		self.getExtremePoints()
		
		if self.extExists:
			self.getCentrePoint()
			self.getDistanceExtremePoints()
			self.getDiameterImagePlane()
			self.getAngleFromFocus()
			
			self.getDistanceLandingMarkerFromFocus()
			self.getHeightDistanceFromFocus()
			self.correctForCameraOrientation()
			self.loadSurfaceCoeff()
			self.getMomentFromEquation()
			self.getMomentFromImage()
			self.loadHuMomentsToBuffer()
			self.getMeanAndStandardDeviation()
			self.confirmDome()
			self.stackData()
			# self.showExtremePoints()
			return "Image Processed"
		else:
			return "No Contour Found"

	def sourceImage(self, topic='default'):
		"""
		Function to define source of image for Hu Moment extraction	
		"""
		if topic == 'default':
			self.image = cv2.imread('/home/fatguru/catkin_ws/src/btp/raw_data/image/20200427-164100/20200427-165151.jpg')
		elif topic == 'ROS':
			self.image = self.sub_image

	def getHSVmask(self):
		"""
		Generate Binary Image from HSV thresholding
		"""
		self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
		self.mask = cv2.inRange(self.hsv, self.HSV_min, self.HSV_max)

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
		if cnts:
			c = max(cnts, key=cv2.contourArea)
	 		self.extMin = tuple(c[c[:, :, 0].argmin()][0])	
	 		self.extMax = tuple(c[c[:, :, 0].argmax()][0])
	 		self.extExists = True
			print "Min Extreme Point:", self.extMin
			print "Max Extreme Point:", self.extMax
		else:
			self.extExists = False

	def showExtremePoints(self):	
 		"""
 		To visualize the obtained extremum points(Image)
 		"""
 		if self.extExists:
 			cv2.circle(self.image, self.extMin, 8, (0, 0, 255), -1)
 			cv2.circle(self.image, self.extMax, 8, (0, 255, 0), -1)
		self.showImage()

	def getCentrePoint(self):
		# From Top Left Corner
		self.centreX = float(self.extMax[0] + self.extMin[0])* 0.5
		self.centreY = float(self.extMax[1] + self.extMin[1])* 0.5
		# From centre
		self.centreX = self.centreX - (self.img_width * 0.5)
		self.centreY = self.centreY - (self.img_height * 0.5)

		print "X Centre:", self.centreX
		print "Y Centre:", self.centreY

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
		
		print "Max From Centre", self.extMaxFromCentre
		print "Min From Centre", self.extMinFromCentre
		
		print "Max From Focus", self.extMaxfromFocus
		print "Min From Focus", self.extMinfromFocus

	def getDiameterImagePlane(self):
		"""
		Simple trignometric calculation to obtain distance between extremum points(Image)
		"""
		self.diameterImagePlane = math.sqrt((self.extMax[0] - self.extMin[0])**2 + (self.extMax[1] - self.extMin[1])**2)
		print "Diameter Image Plane", self.diameterImagePlane
	
	def getAngleFromFocus(self):
		"""
		Calculate the angle between the lines passing through focus intersecting the two extreme points on the diameter
		Using theta = arccos(b**2 + c**2 - a**2 / 2*b*c)
		Angle is obtained in radians
		"""
		self.theta = np.arccos( (self.extMaxfromFocus**2 + self.extMinfromFocus**2 - self.diameterImagePlane**2) / (2* self.extMinfromFocus* self.extMaxfromFocus) )
		print "Angle From Focus", self.theta

	def getDistanceLandingMarkerFromFocus(self):
		"""
		Get the distance of the Landing Marker from Focus
		Equation (17) in Paper 
		"""
		self.zDome = (self.diameter * 0.5) / (np.tan(self.theta* 0.5))
		print "Distance of Centre from Focus:", self.zDome 

	def getHeightDistanceFromFocus(self):
		"""
		Calculate the Height And Distance of the landing marker from the Focus ( i.e. from the camera)
		Equation (19) from the paper
		"""
		self.alpha = np.arctan( (self.centreY) / math.sqrt(self.focal_length**2 + self.centreX**2))
		self.heightFromMarker = self.zDome* np.sin(self.alpha)
		self.distanceFromMarker = self.zDome* np.cos(self.alpha)

		print "Angle alpha: ", self.alpha
		print "Height From Marker: ", self.heightFromMarker
		print "Distance From Marker: ", self.distanceFromMarker
	
	def correctForCameraOrientation(self):
		"""
		Assuming that our principal axis does not run parallel to the ground but at a given angle to it.
		This angle is available from the Gazebo File where the camera position is set
		The height and depth measured from the camera, need to be adjusted to obtain the actual height and depth of the landing marker from 
		the drone. A simple rotation matrix can be used to adjust the frame of reference
		"""
		cx = math.cos(self.camera_tilt)
		sx = math.sin(self.camera_tilt)
		R = np.array([[cx, -sx],[sx, cx]])
		vec = np.dot(R, np.array([[self.distanceFromMarker], [self.heightFromMarker]]))
		self.distanceFromMarker = vec[0]
		self.heightFromMarker = vec[1]

		print "Adjusted Height: ", self.distanceFromMarker
		print "Adjusted Distance", self.heightFromMarker

	def loadSurfaceCoeff(self):
		"""
		Load coefficients of least squares surface obtained
		"""
		self.surface_coeff = np.loadtxt('/home/fatguru/catkin_ws/src/btp/learned_moments/20200519-171152', delimiter=',')

		print ""
		print "Surface coefficients"
		print self.surface_coeff
		print ""

	def getMomentFromEquation(self):
		"""
		Load HuMoment from Equation
		"""
		# x0 = self.distanceFromMarker
		# Compensating for change in frame of reference
		x0 = self.distanceFromMarker
		x1 = self.heightFromMarker
		print "x0", x0
		print "x1", x1
		x0_square = x0**2
		x1_square = x1**2
		x0_x1 = x0*x1
		vec = np.array([[x0, x1, x0_square, x1_square, x0_x1, 1]])
		self.HuMomentsFromEquation = np.dot(vec, self.surface_coeff).T
		self.HuMomentsFromEquation = np.array([self.HuMomentsFromEquation[0][0], self.HuMomentsFromEquation[1][0], self.HuMomentsFromEquation[2][0], self.HuMomentsFromEquation[3][0]]).reshape(4, 1)
		print ""
		print "Moment from Equation:", self.HuMomentsFromEquation.shape
		print self.HuMomentsFromEquation
		print ""

	def getMomentFromImage(self):
		"""
		Compute Hu Moments for binary image
		"""
		moments = cv2.moments(self.mask)
		self.HuMomentsFromImage = cv2.HuMoments(moments)
		
		# Pick only the first 4 Moments
		self.HuMomentsFromImage = self.HuMomentsFromImage[0:4]
		
		print ""
		print "Image", self.HuMomentsFromImage.shape
		print self.HuMomentsFromImage	
		print ""
	
	def loadHuMomentsToBuffer(self):
		if self.HuMomentsFromEquationBuffer == []:
			self.HuMomentsFromEquationBuffer = self.HuMomentsFromEquation
		else:
			self.HuMomentsFromEquationBuffer = np.append(self.HuMomentsFromEquation, self.HuMomentsFromEquationBuffer, axis = 1)
		
		if self.HuMomentsFromEquationBuffer.shape[1] > self.bufferLength:
			self.HuMomentsFromEquationBuffer = np.delete(self.HuMomentsFromEquationBuffer, self.bufferLength - 1, 1)

		if self.HuMomentsFromImageBuffer == []:
			self.HuMomentsFromImageBuffer = self.HuMomentsFromImage
		else:
			self.HuMomentsFromImageBuffer = np.append(self.HuMomentsFromImage, self.HuMomentsFromImageBuffer, axis = 1)
		
		if self.HuMomentsFromImageBuffer.shape[1] > self.bufferLength:
			self.HuMomentsFromImageBuffer = np.delete(self.HuMomentsFromImageBuffer, self.bufferLength - 1, 1)

		print ""
		print "HuMomentBufferImage", self.HuMomentsFromImageBuffer.shape
		print self.HuMomentsFromImageBuffer	
		print ""

		print ""
		print "HuMomentBufferEquation", self.HuMomentsFromEquationBuffer.shape
		print self.HuMomentsFromEquationBuffer	
		print ""

	def getMeanAndStandardDeviation(self):
		
		self.HuImageMean = np.mean(self.HuMomentsFromImageBuffer, axis=1)
		self.HuImageStdDeviation = np.std(self.HuMomentsFromImageBuffer, axis=1)

		self.HuEquationMean = np.mean(self.HuMomentsFromEquationBuffer, axis=1)
		self.HuEquationStdDeviation = np.std(self.HuMomentsFromEquationBuffer, axis=1)

		print ""
		print "Mean from Image", self.HuImageMean.shape
		print self.HuImageMean
		print "Std Deviation Image", self.HuImageStdDeviation.shape
		print self.HuImageStdDeviation
		print ""

		print ""
		print "Mean from Equation", self.HuEquationMean.shape
		print self.HuEquationMean
		print "Std Deviation Equation", self.HuEquationStdDeviation.shape
		print self.HuEquationStdDeviation
		print ""

	def confirmDome(self):

		# lowerLimit = (0.9* HuEquationMean) 
		# upperLimit = (1.1* HuEquationMean) 
		lowerLimit = (0.9* self.HuEquationMean) - (self.HuImageStdDeviation / 2)
		upperLimit = (1.1* self.HuEquationMean) + (self.HuImageStdDeviation / 2)

		print "Lower Limit:", lowerLimit
		print "Upper Limit:", upperLimit

		if ((self.HuImageMean >= lowerLimit).all() and (self.HuImageMean <= upperLimit).all()):
			self.confirmation = True
			print "Dome Confirmed, Landing Command can be issued."
		else:
			print "Dome not confirmed, Cannot land"
			self.confirmation = False

	def stackData(self):
		a = np.ravel([
			self.current_pose[0], 
			self.current_pose[2], 
			self.distanceFromMarker, 
			self.heightFromMarker,

			self.HuMomentsFromEquation[0],
			self.HuMomentsFromImage[0],
			self.HuImageMean[0],
			self.HuImageStdDeviation[0],
			self.HuEquationMean[0],
			self.HuEquationStdDeviation[0],

			self.HuMomentsFromEquation[1],
			self.HuMomentsFromImage[1],
			self.HuImageMean[1],
			self.HuImageStdDeviation[1],
			self.HuEquationMean[1],
			self.HuEquationStdDeviation[1],

			self.HuMomentsFromEquation[2],
			self.HuMomentsFromImage[2],
			self.HuImageMean[2],
			self.HuImageStdDeviation[2],
			self.HuEquationMean[2],
			self.HuEquationStdDeviation[2],

			self.HuMomentsFromEquation[3],
			self.HuMomentsFromImage[3],
			self.HuImageMean[3],
			self.HuImageStdDeviation[3],
			self.HuEquationMean[3],
			self.HuEquationStdDeviation[3],
			]).reshape(1, 28)
		# print "Stack"
		# print self.stack
		# print ""
		if self.stack == []:
			self.stack = a
		else:
			self.stack = np.concatenate((self.stack, a), axis = 0)

	def saveStack(self):
		"""
		Method to save stacked data
		"""
		filename = time.strftime("%Y%m%d-%H%M%S")
		filepath = '/home/fatguru/catkin_ws/src/btp/landing/' + filename
		# rospy.loginfo("Saving surface data file to Path:%s"%filepath)
		if self.stack != []:
			print "Saving Landing data file to Path:%s"%filepath		
			np.savetxt(filepath, self.stack, delimiter=',')
			print "Size of saved stack:", self.stack.shape
			return "Save Successful"
		else:
			print "Save Failed: No Data"
			return "Save Failed: No Data"
	
	def imgCallback(self, ros_data):
		"""
		Callback for image_Subcriber topic
		Saves topic image at self.sub_image
		"""
		np_arr = np.fromstring(ros_data.data, np.uint8).reshape(ros_data.height, ros_data.width, 3)
		raw = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		self.sub_image = np_arr

	def poseCallback(self, data):
		"""
		Callback for local drone position topic
		Stores pose in private data structure
		TODO:
		1] Origin of reference frame of data is Home of Drone
		2] Need to modify so that position is wrt Landing Marker, whose position is defined in temp.world 
		"""
		self.current_pose[0] = data.pose.position.x
		self.current_pose[1] = data.pose.position.y
		self.current_pose[2] = data.pose.position.z

	def showImage(self):
		"""
		Show image obtained on topic
		"""
		cv2.imshow('image', self.image)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

def main():
	"""
	Main Function
	"""
	instance = LandingModule()
	rospy.spin()

if __name__ == '__main__':
	main()