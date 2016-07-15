#!/usr/bin/env python
import rospy
import sys

import numpy 	as np

from math 		import pi

x,y,th,v_x, v_z = 0,1,2,3,4
a_x,a_z 		= 0,1
data, pred_beh 	= 0,1

d_limits				= np.array([[-50,50],[-50,50],[-np.pi,np.pi],[-3,3],[-3,3]]) #x,y,theta,h limits										#x,y,theta,v_x,v_r limits
array_size				= 200																													#How many particles we want

R						= np.array([[.2,.01,.01,.01,.01],[.01,.2,.01,.01,.01],[.01,.01,.1,.01,.01],[.01,.01,.01,.1,.01],[.01,.01,.01,.01,.1]])	#Motion model noise
Q						= np.array([[.2,.01,.02],[.01,.2,.02],[.01,.01,.2]])																							#Measurement noise

'''
Changes state based on the behv given
'''
def g_func_v_p(tmp_state, behv, dt):
	state 		= tmp_state[:]

	state[v_x] 	= behv[a_x]*dt + state[v_x]
	state[v_z] 	= behv[a_z]*dt + state[v_z]

	if state[v_z] != 0:
		state[x] 		= state[x] - (state[v_x] / state[v_z]) * np.sin(state[th]) + (state[v_x] / state[v_z]) * np.sin(state[th] + state[v_z] * dt)
		state[y] 		= state[y] + (state[v_x] / state[v_z]) * np.cos(state[th]) - (state[v_x] / state[v_z]) * np.cos(state[th] + state[v_z] * dt)
		state[th] 		= state[th] + state[v_z] * dt

	else:
		state[x] 		= state[x] + np.cos(state[th]) * dt * state[v_x]
		state[y] 		= state[y] + np.sin(state[th]) * dt * state[v_x]
		state[th] 		= state[th]

	return state


'''
Residual for motion model
'''
def g_res_fn(z, mean):

	residual 	= z[0:3] - mean

	residual[2] = rot_diff(residual[2])

	return residual

def rot_diff(rot_in):
	while rot_in > pi:
		rot_in -= 2 * pi
	while rot_in < -pi:
		rot_in += 2 * pi
	return rot_in

'''
Obtaineds the observation data from our input data
'''
def h_function(mean):
	return mean[0:3]