#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

import numpy as np

from numpy					import zeros,prod,pi,diag
from random					import random, gauss
from math					import hypot
from scipy.stats			import norm

from obj_tr_constants		import x,y,th,v_x, v_z, a_x,a_z, data, pred_beh
from obj_tr_constants 		import g_func_v_p



class Particle_Filter(object):

	def __init__(self, d_limits, array_size, g_function, g_res_fn,h_function, R, Q, init_guess):

		self.g 				= g_function
		self.g_res_fn 		= g_res_fn
		self.h 				= h_function
		self.R 				= R
		self.Q 				= Q

		self.array_size 	= array_size
		self.d_limits		= d_limits

		self.weights			= zeros([self.array_size,1])

		self.result_points		= zeros([self.array_size,self.d_limits.shape[0]])

		self.rand_all(init_guess)


	def rand_all(self, init_guess): 
		for i in range(self.d_limits.shape[0]):
			if i == 0:
				self.point_list = np.random.uniform(self.d_limits[i][0],self.d_limits[i][1],self.array_size).reshape((self.array_size, 1))
			else:
				self.point_list = np.hstack((self.point_list,np.random.uniform(self.d_limits[i][0],self.d_limits[i][1],self.array_size).reshape((self.array_size, 1))))
		
		if init_guess is not None and len(self.point_list) > 0:
			self.point_list[-1] = init_guess


	def upd_points(self, sensor_point):
		#pudb.set_trace()
		tot_weight = self.set_weights(sensor_point)
		self.normalize_weights(tot_weight)
		self.resample()
		return self.get_mean_and_var(self.point_list)


	def set_weights(self, sensor_point):
		tot_weight = 0
		for i in range(len(self.point_list)):
			self.set_weight(self.point_list[i],sensor_point,i)
			tot_weight = tot_weight + self.weights[i]
		return tot_weight


	def set_weight(self,filter_point, sensor_point, index):
		diffs 				= self.g_res_fn(filter_point,self.h(sensor_point))
		# multiply the probabilities of each dimention
		self.weights[index] = prod(norm(0,diag(self.Q)).pdf(diffs))


	def normalize_weights(self, tot_weight):
		for i in range(len(self.point_list)):
			self.weights[i] = self.weights[i] / tot_weight


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


	def get_mean_and_var(self,pointList):
		mean = np.mean(pointList, axis=0)
		cov  = np.cov(pointList.T)
		return (mean, cov)


	def move_all(self,control,dt):
		for i in range(self.array_size):
			self.point_list[i] = self.move(self.point_list[i],control,self.g,dt)


	def move(self,point,control,g,dt):
		mean 				= g(point, control, dt)
		mean				= diag(gauss(mean, self.R)).copy()
		return mean


	def get_pt_list(self): 
		return self.point_list