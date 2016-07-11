#!/usr/bin/env python
import rospy
import sys

import numpy 	as np

x,y,th,v_x, v_z = 0,1,2,3,4
a_x,a_z 		= 0,1
data, pred_beh 	= 0,1

'''
Changes state based on the behv given
'''
def g_func_v_p(state, behv, dt):

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