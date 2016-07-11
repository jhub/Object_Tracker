#!/usr/bin/env python
import rospy
import sys
import tf
#import pudb

import numpy as np

from numpy					import zeros,prod,pi,diag
from random					import random, gauss
from math					import hypot
from scipy.stats			import norm



class Particle_Filter(object):

	def __init__(self, d_limits, array_size, g_function,h_function, R, Q):

		self.g 			= g_function
		self.R 			= R
		self.Q 			= Q
		self.h 			= h_function

		self.array_size = array_size
		self.d_limits	= d_limits
		self.tally		= zeros(3)

		self.weights			= zeros([self.array_size,1])

		self.result_points		= zeros([self.array_size,self.d_limits.shape[0]])

		self.rand_all()


	def rand_all(self):

		for i in range(self.d_limits.shape[0]):
			if i == 0:
				self.point_list = np.random.uniform(self.d_limits[i][0],self.d_limits[i][1],self.array_size).reshape((self.array_size, 1))
			else:
				self.point_list = np.hstack((self.point_list,np.random.uniform(self.d_limits[i][0],self.d_limits[i][1],self.array_size).reshape((self.array_size, 1))))


	def upd_points(self, point):
		#pudb.set_trace()
		tot_weight = self.set_weights(point)
		self.normalize_weights(tot_weight)
		self.resample()


	def resample(self):

		sample_interval = float(1)/self.array_size
		offset 			= random() * sample_interval

		last_weight		= self.weights[0]
		index_weight	= 0

		current_weight	= 0

		#resample while while copying nodes with higher weights
		for i in range(self.array_size):
			current_weight 			= offset + i * sample_interval

			while current_weight > last_weight:
				index_weight 	+= 1
				last_weight 	+= self.weights[index_weight]

			self.result_points[i] 	= self.point_list[index_weight]

			i += 1

		self.point_list = self.result_points[:]

		mean = np.mean(self.point_list, axis=0)

		if mean[3] < 0.05:
			self.tally[0] += 1
		elif mean[3] > 0.95:
			self.tally[1] += 1
		else:
			self.tally[2] += 1

		print("Percentage Guessed:")
		print((self.tally[0] + self.tally[1]) /self.tally.sum())
		
		if (self.tally[0] + self.tally[1]) > 0:
			print("Not compromized accuracy:")
			print(self.tally[0] /(self.tally[0] + self.tally[1]))
		else:
			print("DID NOT MAKE GUESSES!!!")


	def set_weights(self, sensor_point):
		tot_weight = 0
		for i in range(len(self.point_list)):
			self.set_weight(self.point_list[i],sensor_point,i)
			tot_weight = tot_weight + self.weights[i]

		return tot_weight


	def normalize_weights(self, tot_weight):
		for i in range(len(self.point_list)):
			self.weights[i] = self.weights[i] / tot_weight


	def set_weight(self,filter_point, sensor_point, index):

		diffs 				= self.residual_fn(sensor_point,self.h(filter_point))
		# multiply the probabilities of each dimention
		self.weights[index] = prod(norm(0,diag(self.Q)).pdf(diffs))


	def residual_fn(self, z, mean):

		residual 	= z - mean

		while residual[2] > pi:
			residual[2] -= 2 * pi

		while residual[2] < -pi:
			residual[2] += 2 * pi

		return residual


	def move_all(self,control,dt):
		for i in range(self.array_size):
			self.point_list[i] = self.move(self.point_list[i],control,self.g,dt)
		return self.point_list


	def move(self,point,control,g,dt):
		mean 				= g(control, point,dt)
		mean				= diag(gauss(mean, self.R)).copy()

		if mean[3] > 1:
			mean[3] = 1
		elif mean[3] < 0:
			mean[3] = 0

		return mean