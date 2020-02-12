#!/usr/bin/env python
import cv2

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
	test()

if __name__ == '__main__':
	main()