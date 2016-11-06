#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

import numpy as np

from object_tracker.msg import behv, state, st_beh
from nav_msgs.msg		import Odometry
from turtlesim.msg 		import Pose
from time 				import sleep
from math 				import pi
from obj_tr_constants	import bc_interval
from obj_tr_constants	import g_func_v_p

x,y,th,v_x, v_th = 0,1,2,3,4
a_x,a_th 		= 0,1
data, pred_beh 	= 0,1
st_ind, ctl_ind = 0,1

'''
Used to obtain the state/behavior lists and the map updated from those lists
'''
def get_full_planned(g_func,map_beh, curr_state, beh_dur_list, dt):
	state_beh_list = []
	for beh_dur in beh_dur_list:
		state_beh_list.append([curr_state[:], beh_dur[:]])
		curr_state = g_func(curr_state,beh_dur,dt)
	return state_beh_list


'''
Creates a behavior list 
'''
def get_sample_construct(sample_list, dt):
	map_beh 		= [[],[]]
	beh_list 	= []

	for bh in sample_list:
		for itr in range(int(bh[1]/dt)):
			beh_list.append(bh[0])
	
	return beh_list


def load_beh_lists(comp_map):
	un_comp 				= [ [[.5,0],2],[[1,0],2],[[1,-pi/8],2],[[1,pi/4],2],[[1,-pi/8],2],[[1,0],4],[[1,pi/8],2],[[1,-pi/4],2],[[1,pi/8],2],[[1,0],2],[[.5,0],2] ]
	beh_list_u				= get_sample_construct(un_comp,bc_interval)

	comp 					= [ [[.5,0],2],[[1,0],18],[[.5,0],2] ]
	beh_list_c				= get_sample_construct(comp,bc_interval)

	return beh_list_u if comp_map else beh_list_c


def publish_state_beh(state_beh_list, bc_interval):
	for st_bh in state_beh_list:
		if not rospy.is_shutdown():
			s 		= state()
			s.x 	= st_bh[st_ind][x] 	
			s.y		= st_bh[st_ind][y] 	
			s.th 	= st_bh[st_ind][th] 	
			s.v_x	= st_bh[st_ind][v_x] 
			s.v_th	= st_bh[st_ind][v_th]
			s.MAC 	= "K1" 
			print s

			pub_state.publish(s)

			b 		= behv()
			b.a_x 	= st_bh[ctl_ind][a_x] 
			b.a_th	= st_bh[ctl_ind][a_th]
			b.MAC 	= "K1" 

			pub_behv.publish(b)

			#sb 			= st_beh()
			#sb.MAC 		= "K1"
			#sb.state 		= s 
			#sb.beh 		= b

			#pub_st_beh.publish(sb)
			
			sleep(bc_interval)

		else : break


def insert_noise(pose, q):
	x 		= pose.x
	y 		= pose.y

	#insert noise here

def turt_pose_cbak(pose):
	odom_msg = Odometry()

	quaternion = tf.transformations.quaternion_from_euler(0, 0, pose.theta)

#	x,var_x,y,var_y,theta, theta_var = insert_noise(pose, quaternion)


	#pudb.set_trace() #For Debugging

	odom_msg.pose.pose.position.x 		= pose.x
	odom_msg.pose.pose.position.y 		= pose.y
	odom_msg.pose.pose.position.z 		= 0.0
	odom_msg.pose.pose.orientation.x 	= quaternion[0]
	odom_msg.pose.pose.orientation.y 	= quaternion[1]
	odom_msg.pose.pose.orientation.z 	= quaternion[2]
	odom_msg.pose.pose.orientation.w 	= quaternion[3]


	pub_state.publish(odom_msg)


if __name__ == '__main__':

	rospy.init_node('obj_trck', anonymous=True)

	rospy.Subscriber("/turtle2/pose", Pose, turt_pose_cbak)

	pub_state 				= rospy.Publisher('/turtle2/noisedOdom', Odometry, queue_size=10)
	# pub_behv				= rospy.Publisher('beh_k1', behv, queue_size=10)
	# pub_st_beh 				= rospy.Publisher('st_beh_k1', st_beh, queue_size=10)

	# init_state 				= [0.0,0.0,0.0]
	# map_beh 				= [[],[]]
	#pudb.set_trace() #For Debugging
	# comp_map 				= True
	# beh_list				= load_beh_lists(comp_map)

	# state_beh_list			= get_full_planned(g_func_v_p,map_beh,init_state,beh_list,bc_interval)

	# for s in state_beh_list:
	# 	print s

	# publish_state_beh(state_beh_list, bc_interval)

	rospy.spin()