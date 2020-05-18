#!/usr/bin/env python
"""
The nomenclature pattern for the class objects, functions, and variables may not be consistent despite sincere efforts made for the same

Least square fit for hu-moment data!
"""
import time
import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.optimize import least_squares

class fitData:
	def __init__(self):
		self.raw_data = []
		self.loadData()
		self.augmentData(self.x_raw, self.z_raw)
		self.theta = self.simplefit(self.augX, self.augY)
		self.genPlot(self.theta)
		self.requestSaveData()
		# self.testfit()

	def testfit(self):
		rnd = np.random.RandomState(1)
		x0 = np.linspace(1, 10, 10).reshape((10,1))
		x1 = np.linspace(51, 60, 10).reshape((10,1))
		error = rnd.randn(10).reshape((10,1))
		x0 = x0 + 0.25*error
		x1 = x1 + 0.35*error
		self.augmentData(x0, x1)
		self.augY = np.linspace(52, 70, 10).reshape((10,1))
		self.theta = self.simplefit(self.augX, self.augY)
		self.genPlot(self.theta)
		print th

	def augmentData(self, x0, x1):
		self.x0 = x0
		self.x1 = x1
		x0_square = self.x0 ** 2 
		x1_square = self.x1 ** 2
		x0_x1 = self.x0 * self.x1
		# Normalize matrices if required
		self.augX = np.hstack((self.x0, self.x1, x0_square, x1_square, x0_x1, np.ones((np.shape(self.x0)[0],1))))
		self.normalizehu()
		self.augY = np.hstack((self.hu1, self.hu2, self.hu3, self.hu4))
	
	def normalizehu(self):
		i = self.hu1
		norm = np.linalg.norm(i)	
		if norm != 0:
			self.hu1 =  self.hu1 / norm

		i = self.hu2
		norm = np.linalg.norm(i)	
		if norm != 0:
			self.hu2 =  self.hu2 / norm

		i = self.hu3
		norm = np.linalg.norm(i)	
		if norm != 0:
			self.hu3 =  self.hu3 / norm

		i = self.hu4
		norm = np.linalg.norm(i)	
		if norm != 0:
			self.hu4 =  self.hu4 / norm

	def loadData(self):
		self.raw_data = np.loadtxt('/home/fatguru/catkin_ws/src/btp/data/20200427-170305', delimiter=',')
		self.raw_data = self.raw_data[0:31][:] # Latest data generated with self.raw_data[0:31][:]
		self.x_raw, self.y_raw, self.z_raw, self.hu1, self.hu2, self.hu3, self.hu4, self.hu5, self.hu6, self.hu7 = np.hsplit(self.raw_data, np.array([1,2,3,4,5,6,7,8,9]))
	
		#Add reference frame offset here
		# self.x_raw = self.x_raw + 30
		# self.y_raw = self.y_raw 
	
	def simplefit(self, Xdata, Ydata):
		theta = np.dot(np.dot(np.linalg.inv(np.dot(Xdata.T, Xdata)), Xdata.T), Ydata)
		return theta

	def genPlot(self, theta):
		step_size = 20
		x_temp = np.linspace(np.min(self.x0), np.max(self.x0), step_size).reshape(step_size, 1)
		y_temp = np.linspace(np.min(self.x1), np.max(self.x1), step_size).reshape(step_size, 1)
		foo = np.hstack((x_temp, y_temp, x_temp**2, y_temp**2, x_temp*y_temp, np.ones((step_size,1))))
		outval = np.dot(foo, theta)
		x = np.outer(x_temp, np.ones(step_size))
		y = np.outer(x_temp, np.ones(step_size)).T
		# y = np.outer(y_temp, np.ones(step_size)).T
		
		fig = plt.figure()

		ax = fig.add_subplot(2,2,1,projection = '3d')
		ax.plot_surface(x, y, outval[:, 0].reshape(step_size,1), cmap='viridis', edgecolor='none')
		ax.scatter(self.x0, self.x1, self.augY[:, 0], marker='^', color='r')
		ax.set_title('Hu1')

		ax = fig.add_subplot(2,2,2,projection = '3d')
		ax.plot_surface(x, y, outval[:, 1].reshape(step_size,1), cmap='viridis', edgecolor='none')
		ax.scatter(self.x0, self.x1, self.augY[:, 1], marker='^', color='r')
		ax.set_title('Hu2')

		ax = fig.add_subplot(2,2,3,projection = '3d')
		ax.plot_surface(x, y, outval[:, 2].reshape(step_size,1), cmap='viridis', edgecolor='none')
		ax.scatter(self.x0, self.x1, self.augY[:, 2], marker='^', color='r')
		ax.set_title('Hu3')

		ax = fig.add_subplot(2,2,4,projection = '3d')
		ax.plot_surface(x, y, outval[:, 3].reshape(step_size,1), cmap='viridis', edgecolor='none')
		ax.scatter(self.x0, self.x1, self.augY[:, 3], marker='^', color='r')
		ax.set_title('Hu4')
		
		plt.show()

	def requestSaveData(self):
		blip = raw_input("Please press Y to save generated least square fit surface Data:")
		if blip == 'y':
			self.saveData()
		else:
			print "Exiting"
	def saveData(self):
		"""
		Method to save obtained Least-Square Fit Surface to below path
		"""
		filename = time.strftime("%Y%m%d-%H%M%S")
		filepath = '/home/fatguru/catkin_ws/src/btp/learned_moments/' + filename
		# rospy.loginfo("Saving surface data file to Path:%s"%filepath)
		print "Saving surface data file to Path:%s"%filepath		
		np.savetxt(filepath, self.theta, delimiter=',')
		print "Data saved!"

def main():
	instance = fitData()

if __name__ == '__main__':
	main()