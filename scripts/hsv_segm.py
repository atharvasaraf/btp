#!/usr/bin/env python
import cv2
import numpy as np 

def test():
	in_image = cv2.imread('./a.jpg')
	COLOUR_MIN = np.array([ 0, 0, 0.4]) 
	COLOUR_MAX = np.array([ 0.5, 0.5, 1])
	hsv = cv2.cvtColor(in_image, cv2.COLOR_BGR2HSV)

	mask = cv2.inRange(hsv, COLOUR_MIN, COLOUR_MAX)
	cv2.imshow('mask', mask)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	
def main():
	print cv2.__version__
	test()

if __name__ == '__main__':
	main()