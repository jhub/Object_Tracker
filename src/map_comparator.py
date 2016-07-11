#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

import numpy 	as np

from scipy 				import spatial, stats
from object_tracker.msg import behv, state, st_beh
from time 				import sleep
from math 				import pi
from bayes 				import compromized_state
from random 			import gauss
from obj_tr_constants	import x,y,th,v_x, v_z, a_x,a_z, data, pred_beh
from obj_tr_constants 	import g_func_v_p


MAP_MAX_NGBR 	= 100

st_var			= np.array([1,1,.4,.8,.4])
beh_var 		= np.array([.2,.2])
rand_len		= 200

'''
Used to obtain the state/behavior lists and the map updated from those lists
'''
def get_full_planned(g_func,map_st_beh, curr_state, beh_list, dt):
	#pudb.set_trace() #For Debugging
	for beh_dur in beh_list:
		insert_noised(map_st_beh, curr_state, beh_dur)
		g_func(curr_state,beh_dur,dt)


def insert_noised(map_st_beh, curr_state, curr_beh):
	global st_var, beh_var#, rand_len
	if not len(map_st_beh[data]):
		map_st_beh[data] 		= np.atleast_2d(curr_state[:] + np.random.normal(0,st_var)).T
		map_st_beh[pred_beh]	= np.atleast_2d(curr_beh[:] + np.random.normal(0,beh_var)).T
	else:
		map_st_beh[data] 		= np.hstack([map_st_beh[data],np.atleast_2d(curr_state[:] + np.random.normal(0,st_var)).T])
		map_st_beh[pred_beh] 	= np.hstack([map_st_beh[pred_beh],np.atleast_2d(curr_beh[:] + np.random.normal(0,beh_var)).T])



'''
Creates a behavior list 
'''
def get_sample_construct(sample_list, dt):
	map_st_beh 		= [[],[]]
	beh_list 	= []

	for bh in sample_list:
		for itr in range(int(bh[1]/dt)):
			beh_list.append(bh[0])
	
	return beh_list


'''
Loads the compromised and uncompromised maps
'''
def load_beh_lists():
	global bc_interval
	un_comp 				= [ [[1,0],1],[[0,0],2],[[0,-pi/4],1],[[0,pi/2],1],[[0,-pi/4],1],[[0,0],4],[[0,pi/4],1],[[0,-pi/2],1],[[0,pi/4],1],[[0,0],2],[[-1,0],1],[[0,0],1] ]
	beh_list_u				= get_sample_construct(un_comp,bc_interval)

	comp 					= [ [[1,0],2],[[0,0],5],[[-1,0],2],[[0,0],1] ]
	beh_list_c				= get_sample_construct(comp,bc_interval)

	return beh_list_u, beh_list_c


def state_to_list(state):
	return [state.x, state.y, state.th, state.v_x, state.v_z]


def beh_to_list(beh):
	return [beh.a_x, beh.a_z]


def behv_callback(behv):
	print behv


def st_beh_callback(st_beh):
	global kd_map_u, kd_map_c, map_st_beh_u, map_st_beh_c, k_list
	global MAP_MAX_NGBR
	#pudb.set_trace() #For Debugging

	beh 			= beh_to_list(st_beh.beh)
	state 			= state_to_list(st_beh.state)
	#state 			= k_list[mac].get_PF_state(beh)
	mac  			= st_beh.MAC

	u_results 		= kd_map_u.query(state,MAP_MAX_NGBR)
	c_results 		= kd_map_c.query(state,MAP_MAX_NGBR)

	u_prob			= compare_sb(state, beh, u_results, map_st_beh_u)
	c_prob			= compare_sb(state, beh, c_results, map_st_beh_c)

	try:
		k_list[mac].update_prob(c_prob, u_prob)
		print "Likelyhood of being compromised is: " + str(k_list[mac].get_c_prob())
		#print "compromised" if comp_prob > .9 else "Not compromised" if comp_prob < .1 else "Determining"
	except Exception:
		print "Not Found Mac"


def compare_sb(state, beh, kd_list, map_st_beh):
	if len(kd_list) > 0 and len(kd_list[1]) > len(map_st_beh[0]): #check dimensions for enough pts (non singular matrix)

		st_kernel 	= stats.gaussian_kde(map_st_beh[data]) 
		bh_kernel	= stats.gaussian_kde(map_st_beh[pred_beh]) 
		state_pdf 	= st_kernel.pdf(state)[0]			#Likelyhood of doing a recorded state
		beh_pdf 	= bh_kernel.pdf(beh)[0]				#Likelyhood of following a behavior prev done

		return state_pdf * beh_pdf
	raise Exception("Need more points")


def print_all(list_in):
	for i in list_in:
		for l in i:
			print tuple(l)

if __name__ == '__main__':
	global bc_interval, kd_map_u, kd_map_c, map_st_beh_u, map_st_beh_c, k_list

	rospy.init_node('camp_comp', anonymous=True)

	map_st_beh_u 			= [[],[]]
	map_st_beh_c 			= [[],[]]
	k_list 					= {}
	bc_interval				= .01

	init_state_u			= [0.0,0.0,0.0,0.0,0.0]
	init_state_c			= [0.0,0.0,0.0,0.0,0.0]
	#pudb.set_trace() #For Debugging
	beh_list_u, beh_list_c 	= load_beh_lists()

	get_full_planned(g_func_v_p,map_st_beh_u,init_state_u,beh_list_u,bc_interval)
	get_full_planned(g_func_v_p,map_st_beh_c,init_state_c,beh_list_c,bc_interval)


	#print_all([map_st_beh_u[0],map_st_beh_c[0]])

	kd_map_u = spatial.KDTree(map_st_beh_u[data].T)
	kd_map_c = spatial.KDTree(map_st_beh_c[data].T)

	k_list["K1"] = compromized_state(0.05)
	rospy.Subscriber("st_beh_k1", st_beh, st_beh_callback)

	rospy.spin()


	#publish_state_beh(state_beh_list, bc_interval)