#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from math import copysign, log10

class HuMoment:
	def __init__(self):
		self.HSV_min = np.array([0, 5, 229])
		self.HSV_max = np.array([179, 255, 255])
		self.hu_Moments = np.empty([7, 1])
		self.nrmld_Hu = np.empty([7, 1])

	def getHSVmask(self):
		self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
		self.mask = cv2.inRange(self.hsv, self.HSV_min, self.HSV_max)

	def getHuMoments(self):
		moments = cv2.moments(self.mask)
		self.hu_Moments = cv2.HuMoments(moments)
		
		for i in range(0,7):
  			self.nrmld_Hu[i] = -1* copysign(1.0, self.hu_Moments[i]) * log10(abs(self.hu_Moments[i]))

	def showMask(self):
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

	def sourceImage(self):
		self.image = cv2.imread('/home/fatguru/catkin_ws/src/btp/a.jpg')

	def fetchAndUpdate(self):
		self.sourceImage()
		self.getHSVmask()
		self.getHuMoments()
		self.showMask()
		self.printHuMoments()

def main():
	HuNode = HuMoment()
	HuNode.fetchAndUpdate()

if __name__ == '__main__':
	main()