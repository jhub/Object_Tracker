#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

import numpy 	as np

from visualization_msgs.msg import Marker, MarkerArray
from scipy 					import spatial, stats
from object_tracker.msg 	import behv, state, st_beh
from time 					import sleep
from math 					import pi
from bayes 					import compromized_state
from random 				import gauss
from threading 				import Thread
from obj_tr_constants		import x,y,th,v_x, v_z, a_x,a_z, data, pred_beh
from obj_tr_constants 		import g_func_v_p


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
Loads the compromised and uncompromised maps
'''
def load_beh_lists():
	global bc_interval
	un_comp 				= [ [[1,0],1],[[0,0],2],[[0,-pi/4],1],[[0,pi/2],1],[[0,-pi/4],1],[[0,0],4],[[0,pi/4],1],[[0,-pi/2],1],[[0,pi/4],1],[[0,0],2],[[-1,0],1],[[0,0],1] ]
	beh_list_u				= get_sample_construct(un_comp,bc_interval)

	comp 					= [ [[1,0],2],[[0,0],5],[[-1,0],2],[[0,0],1] ]
	beh_list_c				= get_sample_construct(comp,bc_interval)

	return beh_list_u, beh_list_c


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


def state_to_list(state):
	return [state.x, state.y, state.th, state.v_x, state.v_z]

def beh_to_list(beh):
	return [beh.a_x, beh.a_z]

def get_mac(msg):
	return msg.MAC


'''
Changes the control of the particle filter
'''
def beh_callback(beh):
	global k_list
	#pudb.set_trace() #For Debugging
	mac 	= get_mac(beh)
	beh 	= beh_to_list(beh)
	k_list[mac].upd_PF_behv(beh)


'''
Sensor info coming in [x,y]
'''
def sns_callback(state):
	global k_list
	mac 	= get_mac(state)
	mean 	= k_list[mac].upd_PF_sens(state_to_list(state))
	get_comp_prob(mean, k_list[mac])


def get_comp_prob(state, bayes_obj):
	global k_list, kd_map_u, kd_map_c, map_st_beh_u, map_st_beh_c, MAP_MAX_NGBR

	u_results 		= kd_map_u.query(state,MAP_MAX_NGBR)
	c_results 		= kd_map_c.query(state,MAP_MAX_NGBR)

	u_prob			= compare_sb(state, bayes_obj.PF_behv(), u_results, map_st_beh_u)
	c_prob			= compare_sb(state, bayes_obj.PF_behv(), c_results, map_st_beh_c)

	try:
		bayes_obj.update_prob(c_prob, u_prob)
		print "Likelyhood of being compromised is: " + str(bayes_obj.get_c_prob())
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


'''
Updates the particle filter
'''
def bayes_upd(bayes_obj):
	UPD_FREQUENCY	= rospy.Rate(10)
	upd = 0
	while True:
		pointList = bayes_obj.get_PF_state()
		#pointlist can be used to display, but should not be used to determine prob
		if pointList is not None:
			publish_partcles(pointList) 

		UPD_FREQUENCY.sleep()


def publish_partcles(pointList):

	markerArray = MarkerArray()

	for i in range(pointList.shape[0]):

		point 						= pointList[i]
		marker						= Marker()

		tempQuaternion				= tf.transformations.quaternion_from_euler(0, 0, point[2])

		marker.pose.position.x = point[0]
		marker.pose.position.y = point[1]
		marker.pose.position.z = 0

		marker.scale.x = 0.4
		marker.scale.y = 0.05
		marker.scale.z = 0.01

		marker.color.a = 1.0
		marker.color.r = .1
		marker.color.b = .7

		marker.pose.orientation.x 	= tempQuaternion[0]
		marker.pose.orientation.y 	= tempQuaternion[1]
		marker.pose.orientation.z 	= tempQuaternion[2]
		marker.pose.orientation.w 	= tempQuaternion[3]

		marker.id 				= i
		marker.header.frame_id 	= "/my_frame"
		marker.header.stamp 	= rospy.Time.now()
		marker.action 			= marker.ADD

		markerArray.markers.append(marker)

	particle_pub.publish(markerArray)


def print_all(list_in):
	for i in list_in:
		for l in i:
			print tuple(l)


if __name__ == '__main__':
	global bc_interval, kd_map_u, kd_map_c, map_st_beh_u, map_st_beh_c, k_list, particle_pub
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

	#This is incoming conections, emulated hardcoded
	init_pos 		= np.array([0.0,0.0,0.0,0.0,0.0])
	k_list["K1"] 	= compromized_state(0.05, init_pos)

	rospy.Subscriber("beh_k1", behv, beh_callback)
	rospy.Subscriber("state_k1", state, sns_callback)
	particle_pub = rospy.Publisher('pose_cloud', MarkerArray, queue_size=100)

	for MAC in k_list:
		listener_thread 			= Thread(target = bayes_upd, args = (k_list[MAC], ))
		listener_thread.setDaemon(True)
		listener_thread.start()

	print "Ready"

	rospy.spin()


	#publish_state_beh(state_beh_list, bc_interval)