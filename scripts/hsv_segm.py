#!/usr/bin/env python
import cv2
import numpy as np 

def test():
	in_image = cv2.imread('/home/fatguru/Desktop/20200309-212744.jpg')
	COLOUR_MIN = np.array([ 0, 0, 0.4]) 
	COLOUR_MAX = np.array([ 0.5, 0.5, 1])
	hsv = cv2.cvtColor(in_image, cv2.COLOR_BGR2HSV)
	HSV_min = np.array([0, 128, 95])
	HSV_max = np.array([179, 255, 136])
	mask = cv2.inRange(hsv, HSV_min, HSV_max)
	cv2.imshow('mask', mask)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	cv2.imwrite('/home/fatguru/Desktop/20200309-212744_mask.jpg', mask)
def main():
	# print cv2.__version__
	test()

if __name__ == '__main__':
	main()