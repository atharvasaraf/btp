#!/usr/bin/env python
import cv2
import numpy as np
from math import copysign, log10

def gethsv_mask(in_image, MIN_HSV=np.array([0,0,0]), MAX_HSV=np.array([179, 255, 255])):
	hsv = cv2.cvtColor(in_image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, MIN_HSV, MAX_HSV)
	return mask

def get_HuMoments(binary_image):
	moments = cv2.moments(binary_image)
	huMoments = cv2.HuMoments(moments)
	normalized_Hu = huMoments
	for i in range(0,7):
  		normalized_Hu[i] = -1* copysign(1.0, huMoments[i]) * log10(abs(huMoments[i]))
	return normalized_Hu

def test():
	in_image = '/home/fatguru/catkin_ws/src/btp/a.jpg'
	COLOR_MIN = cv2.cv2.Scalar(0, 0, 0.4)
	COLOR_MAX = cv2.Scalar(0.5, 0.5, 1)
	# COLOR_MIN = (0, 0, 0.4)
	# COLOR_MAX = (0.5, 0.5, 1)
	
	frame = cv2.LoadImage(in_image)
	frameHSV = cv2.CreateImage(cv.GetSize(frame), 8, 3)
	cv2.CvtColor(frame, frameHSV, cv2.CV_RGB2HSV)
	frame_threshed = cv2.CreateImage(cv2.GetSize(frameHSV), 8, 1)
	cv2.InRangeS(frameHSV, COLOR_MIN, COLOR_MAX, frame_threshed)
	cv2.imshow(frame_threshed)

def main():
	print cv2.__version__
	in_image = cv2.imread('/home/fatguru/catkin_ws/src/btp/a.jpg')
	# COLOUR_MIN = np.array([ 0, 0, 0.4]) 
	# COLOUR_MAX = np.array([ 0.5, 0.5, 1])
	COLOUR_MIN = np.array([ 0, 5, 229]) 
	COLOUR_MAX = np.array([ 179, 255, 255])
	mask = gethsv_mask(in_image, COLOUR_MIN, COLOUR_MAX)
	huM = get_HuMoments(mask)
	print huM
	# cv2.imshow('mask', mask)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

if __name__ == '__main__':
	main()