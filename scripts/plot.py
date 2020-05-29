#!/usr/bin/env python
import time
import math
import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.optimize import least_squares

class PlotterClass:
	def __init__(self):
		self.x_axis = np.linspace(1, 51, 51)
		self.loadData()
		# self.SortAsPerDistance()
		# self.SortAsPerHeight()
		self.getLimits()
		self.plotHuMomentVariation()
		self.plotHuMomentLimit()
		self.plotDistanceHeight()

	def loadData(self):
		self.raw_data = np.loadtxt('/home/fatguru/catkin_ws/src/btp/landing/20200520-211534', delimiter=',')
		
		self.actual_distance = 100 - self.raw_data.T[0]
		self.actual_height = self.raw_data.T[1] 
		self.distanceFromMarker = self.raw_data.T[2] 
		self.heightFromMarker = self.raw_data.T[3]
		
		self.HuMomentsFromEquation1 = self.raw_data.T[4]
		self.HuMomentsFromImage1 = self.raw_data.T[5]
		self.HuImageMean1 = self.raw_data.T[6]
		self.HuImageStdDeviation1 = self.raw_data.T[7]
		self.HuEquationMean1 = self.raw_data.T[8]
		self.HuEquationStdDeviation1 = self.raw_data.T[9]
		
		self.HuMomentsFromEquation2 = self.raw_data.T[10]
		self.HuMomentsFromImage2 = self.raw_data.T[11]
		self.HuImageMean2 = self.raw_data.T[12]
		self.HuImageStdDeviation2 = self.raw_data.T[13]
		self.HuEquationMean2 = self.raw_data.T[14]
		self.HuEquationStdDeviation2 = self.raw_data.T[15]
		
		self.HuMomentsFromEquation3 = self.raw_data.T[16]
		self.HuMomentsFromImage3 = self.raw_data.T[17]
		self.HuImageMean3 = self.raw_data.T[18]
		self.HuImageStdDeviation3 = self.raw_data.T[19]
		self.HuEquationMean3 = self.raw_data.T[20]
		self.HuEquationStdDeviation3 = self.raw_data.T[21]
		
		self.HuMomentsFromEquation4 = self.raw_data.T[22]
		self.HuMomentsFromImage4 = self.raw_data.T[23]
		self.HuImageMean4 = self.raw_data.T[24]
		self.HuImageStdDeviation4 = self.raw_data.T[25]
		self.HuEquationMean4 = self.raw_data.T[26]
		self.HuEquationStdDeviation4 = self.raw_data.T[27]
	
	# def SortAsPerDistance(self):
	# 	self.index_array = np.argsort(self.actual_distance)
	# 	self.x_axis = self.actual_distance[self.index_array]

	# 	self.actual_distance = self.actual_distance[self.index_array] 
	# 	self.actual_height = self.actual_height[self.index_array] 
	# 	self.distanceFromMarker = self.distanceFromMarker[self.index_array] 
	# 	self.heightFromMarker = self.heightFromMarker[self.index_array] 

	# 	self.HuMomentsFromEquation1 = self.HuMomentsFromEquation1[self.index_array]
	# 	self.HuMomentsFromImage1 = self.HuMomentsFromImage1[self.index_array]
	# 	self.HuImageMean1 = self.HuImageMean1[self.index_array]
	# 	self.HuImageStdDeviation1 = self.HuImageStdDeviation1[self.index_array]
	# 	self.HuEquationMean1 = self.HuEquationMean1[self.index_array]
	# 	self.HuEquationStdDeviation1 = self.HuEquationStdDeviation1[self.index_array]

	# 	self.HuMomentsFromEquation2 = self.HuMomentsFromEquation2[self.index_array]
	# 	self.HuMomentsFromImage2 = self.HuMomentsFromImage2[self.index_array]
	# 	self.HuImageMean2 = self.HuImageMean2[self.index_array]
	# 	self.HuImageStdDeviation2 = self.HuImageStdDeviation2[self.index_array]
	# 	self.HuEquationMean2 = self.HuEquationMean2[self.index_array]
	# 	self.HuEquationStdDeviation2 = self.HuEquationStdDeviation2[self.index_array]

	# 	self.HuMomentsFromEquation3 = self.HuMomentsFromEquation3[self.index_array]
	# 	self.HuMomentsFromImage3 = self.HuMomentsFromImage3[self.index_array]
	# 	self.HuImageMean3 = self.HuImageMean3[self.index_array]
	# 	self.HuImageStdDeviation3 = self.HuImageStdDeviation3[self.index_array]
	# 	self.HuEquationMean3 = self.HuEquationMean3[self.index_array]
	# 	self.HuEquationStdDeviation3 = self.HuEquationStdDeviation3[self.index_array]

	# 	self.HuMomentsFromEquation4 = self.HuMomentsFromEquation4[self.index_array]
	# 	self.HuMomentsFromImage4 = self.HuMomentsFromImage4[self.index_array]
	# 	self.HuImageMean4 = self.HuImageMean4[self.index_array]
	# 	self.HuImageStdDeviation4 = self.HuImageStdDeviation4[self.index_array]
	# 	self.HuEquationMean4 = self.HuEquationMean4[self.index_array]
	# 	self.HuEquationStdDeviation4 = self.HuEquationStdDeviation4[self.index_array]

	# def SortAsPerHeight(self):
	# 	self.index_array = np.argsort(self.actual_height)
	# 	self.x_axis = self.actual_height[self.index_array]

	# 	self.actual_distance = self.actual_distance[self.index_array] 
	# 	self.actual_height = self.actual_height[self.index_array] 
	# 	self.distanceFromMarker = self.distanceFromMarker[self.index_array] 
	# 	self.heightFromMarker = self.heightFromMarker[self.index_array] 

	# 	self.HuMomentsFromEquation1 = self.HuMomentsFromEquation1[self.index_array]
	# 	self.HuMomentsFromImage1 = self.HuMomentsFromImage1[self.index_array]
	# 	self.HuImageMean1 = self.HuImageMean1[self.index_array]
	# 	self.HuImageStdDeviation1 = self.HuImageStdDeviation1[self.index_array]
	# 	self.HuEquationMean1 = self.HuEquationMean1[self.index_array]
	# 	self.HuEquationStdDeviation1 = self.HuEquationStdDeviation1[self.index_array]

	# 	self.HuMomentsFromEquation2 = self.HuMomentsFromEquation2[self.index_array]
	# 	self.HuMomentsFromImage2 = self.HuMomentsFromImage2[self.index_array]
	# 	self.HuImageMean2 = self.HuImageMean2[self.index_array]
	# 	self.HuImageStdDeviation2 = self.HuImageStdDeviation2[self.index_array]
	# 	self.HuEquationMean2 = self.HuEquationMean2[self.index_array]
	# 	self.HuEquationStdDeviation2 = self.HuEquationStdDeviation2[self.index_array]

	# 	self.HuMomentsFromEquation3 = self.HuMomentsFromEquation3[self.index_array]
	# 	self.HuMomentsFromImage3 = self.HuMomentsFromImage3[self.index_array]
	# 	self.HuImageMean3 = self.HuImageMean3[self.index_array]
	# 	self.HuImageStdDeviation3 = self.HuImageStdDeviation3[self.index_array]
	# 	self.HuEquationMean3 = self.HuEquationMean3[self.index_array]
	# 	self.HuEquationStdDeviation3 = self.HuEquationStdDeviation3[self.index_array]

	# 	self.HuMomentsFromEquation4 = self.HuMomentsFromEquation4[self.index_array]
	# 	self.HuMomentsFromImage4 = self.HuMomentsFromImage4[self.index_array]
	# 	self.HuImageMean4 = self.HuImageMean4[self.index_array]
	# 	self.HuImageStdDeviation4 = self.HuImageStdDeviation4[self.index_array]
	# 	self.HuEquationMean4 = self.HuEquationMean4[self.index_array]
	# 	self.HuEquationStdDeviation4 = self.HuEquationStdDeviation4[self.index_array]

	def getLimits(self):
		self.lowerlimitHu1 = (0.9* self.HuEquationMean1) - (self.HuImageStdDeviation1 / 2)
		self.lowerlimitHu2 = (0.9* self.HuEquationMean2) - (self.HuImageStdDeviation2 / 2)
		self.lowerlimitHu3 = (0.9* self.HuEquationMean3) - (self.HuImageStdDeviation3 / 2)
		self.lowerlimitHu4 = (0.9* self.HuEquationMean4) - (self.HuImageStdDeviation4 / 2)
		self.upperlimitHu1 = (1.1* self.HuEquationMean1) + (self.HuImageStdDeviation1 / 2)
		self.upperlimitHu2 = (1.1* self.HuEquationMean2) + (self.HuImageStdDeviation2 / 2)
		self.upperlimitHu3 = (1.1* self.HuEquationMean3) + (self.HuImageStdDeviation3 / 2)
		self.upperlimitHu4 = (1.1* self.HuEquationMean4) + (self.HuImageStdDeviation4 / 2)

	def plotHuMomentVariation(self):
		fig, ax = plt.subplots(2,2)
		fig.suptitle("Deviation of Hu-Moments from least square fit")
		ax[0, 0].plot(self.x_axis, self.HuMomentsFromEquation1, '--', label = 'Hu1 (Equation)')
		ax[0, 0].plot(self.x_axis, self.HuMomentsFromImage1, label = 'Hu1 (Image)')
		ax[0, 0].legend()

		ax[0, 1].plot(self.x_axis, self.HuMomentsFromEquation2, '--', label = 'Hu2 (Equation)')
		ax[0, 1].plot(self.x_axis, self.HuMomentsFromImage2, label = 'Hu2 (Image)')
		ax[0, 1].legend()

		ax[1, 0].plot(self.x_axis, self.HuMomentsFromEquation3, '--', label = 'Hu3 (Equation)')
		ax[1, 0].plot(self.x_axis, self.HuMomentsFromImage3, label = 'Hu3 (Image)')
		ax[1, 0].set(xlabel='Instance')
		ax[1, 0].legend()

		ax[1, 1].plot(self.x_axis, self.HuMomentsFromEquation4, '--', label = 'Hu4 (Equation)')
		ax[1, 1].plot(self.x_axis, self.HuMomentsFromImage4, label = 'Hu4 (Image)')
		ax[1, 1].set(xlabel='Instance')
		ax[1, 1].legend()
		plt.show()

	def plotHuMomentLimit(self):
		fig, ax = plt.subplots(2,2)
		fig.suptitle("Dome confirmation criteria limits with respective Hu Moments")

		ax[0, 0].plot(self.x_axis, self.lowerlimitHu1, 'y-.', label = 'Lower limit Hu1')
		ax[0, 0].plot(self.x_axis, self.upperlimitHu1, 'y-.', label = 'upper limit Hu1')		
		ax[0, 0].plot(self.x_axis, self.HuImageMean1, label = 'Hu1 (Running average from Image)')
		# ax[0, 0].plot(np.linspace(1, 51, 51), self.HuEquationMean1, '--',label = 'Hu1 (Running average from Equation)')
		ax[0, 0].set_ylim([0.0003, 0.0008])
		ax[0, 0].legend()

		ax[0, 1].plot(self.x_axis, self.lowerlimitHu2, 'y-.', label = 'Lower limit Hu2')
		ax[0, 1].plot(self.x_axis, self.upperlimitHu2, 'y-.', label = 'upper limit Hu2')		
		ax[0, 1].plot(self.x_axis, self.HuImageMean2, label = 'Hu2 (Running average from Image)')
		# ax[0, 1].plot(np.linspace(1, 51, 51), self.HuEquationMean2, '--',label = 'Hu2 (Running average from Equation)')
		ax[0, 1].legend()

		ax[1, 0].plot(self.x_axis, self.lowerlimitHu3, 'y-.', label = 'Lower limit Hu3')
		ax[1, 0].plot(self.x_axis, self.upperlimitHu3, 'y-.', label = 'upper limit Hu3')		
		ax[1, 0].plot(self.x_axis, self.HuImageMean3, label = 'Hu3 (Running average from Image)')
		# ax[1, 0].plot(np.linspace(1, 51, 51), self.HuEquationMean3, '--',label = 'Hu3 (Running average from Equation)')
		ax[1, 0].set(xlabel='Instance')
		ax[1, 0].legend()

		ax[1, 1].plot(self.x_axis, self.lowerlimitHu4, 'y-.', label = 'Lower limit Hu4')
		ax[1, 1].plot(self.x_axis, self.upperlimitHu4, 'y-.', label = 'upper limit Hu4')		
		ax[1, 1].plot(self.x_axis, self.HuImageMean4, label = 'Hu4 (Running average from Image)')
		# ax[1, 1].plot(np.linspace(1, 51, 51), self.HuEquationMean4, '--',label = 'Hu4 (Running average from Equation)')
		ax[1, 1].set(xlabel='Instance')
		ax[1, 1].legend()
		plt.show()

	def plotDistanceHeight(self):
		fig, ax = plt.subplots(2, 1)
		fig.suptitle("Actual Position vs. Predicted Position")
		ax[0].plot(self.x_axis, self.distanceFromMarker, label = 'Predicted Distance')
		ax[0].plot(self.x_axis, self.actual_distance, '--', label = 'Actual Distance')
		ax[0].set(ylabel = 'Distance (in m)', xlabel='Instance')
		ax[0].legend()

		ax[1].plot(self.x_axis, self.heightFromMarker, label = 'Predicted Height')
		ax[1].plot(self.x_axis, self.actual_height, '--', label = 'Actual Height')
		ax[1].legend()
		ax[1].set(ylabel = 'Height (in m)', xlabel='Instance')
		ax[1].legend()

		plt.show()

def main():
	instance = PlotterClass()

if __name__ == '__main__':
	main()