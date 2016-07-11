#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

import Particle_Filter

from time import time

class compromized_state(object):

	def __init__(self, initial_prob):
		self.c_prob = initial_prob
		#self.PF 	= 

	def update_prob(self, comp_prob, uncomp_prob):
		self.c_prob = comp_prob * self.c_prob / (comp_prob * self.c_prob + uncomp_prob * (1 - self.c_prob))
		self.adjust_prob()

	def adjust_prob(self):
		if self.c_prob < 0.0001:
			self.c_prob = .0001
		elif self.c_prob > 0.9999:
			self.c_prob = 0.9999

	def get_c_prob(self):
		return self.c_prob

	def get_PF_state(self, behv):
		self.PF.update(behv)
