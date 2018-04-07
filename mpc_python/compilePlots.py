import matplotlib.pyplot as plt
import numpy as np
from math import *
import time
import pickle
import argparse
from copy import copy
import scipy.io as sio

from robot import Acrobot
from mpc import MPC

if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	parser.add_argument('--mode', type=str, default='test')
	parser.add_argument('--model', type=str, default='./models/acrobot_results')
	parser.add_argument('--type', type=str, default='waypoints')

	args = parser.parse_args()
	
	MPC_trajectory_N_5_file = "./models/MPC_trajectory_N_5"
	trajN5_params, trajN5_xHistory, trajN5_uHistory, trajN5_xRefHistory = pickle.load(open(MPC_trajectory_N_5_file, 'rb'))

	MPC_trajectory_N_10_file = "./models/MPC_trajectory_N_10"
	trajN10_params, trajN10_xHistory, trajN10_uHistory, trajN10_xRefHistory = pickle.load(open(MPC_trajectory_N_10_file, 'rb'))

	MPC_waypoints_N_5_file = "./models/MPC_waypoints_N_5"
	waypN5_params, waypN5_xHistory, waypN5_uHistory, waypN5_xRefHistory = pickle.load(open(MPC_waypoints_N_5_file, 'rb'))

	MPC_waypoints_N_10_file = "./models/MPC_waypoints_N_10"
	waypN10_params, waypN10_xHistory, waypN10_uHistory, waypN10_xRefHistory = pickle.load(open(MPC_waypoints_N_10_file, 'rb'))
	

	# Displaying Graphs
	t = np.linspace(0, trajN5_params['Duration'], trajN5_params['Duration']/trajN5_params['Ts']+1)
	f, axarr = plt.subplots(2, 2)
	L = t.size
	axarr[0, 0].plot(t, trajN5_xHistory[:L,0], 'b', label='N = 5')
	axarr[0, 0].plot(t, trajN10_xHistory[:L,0], 'g', label='N = 10')
	axarr[0, 0].plot(t, trajN10_xRefHistory[:L,0],'r-', label='Reference')
	axarr[0, 0].set_title('Joint 1 Position', fontsize=20)
	axarr[0, 0].set_xlabel('time', fontsize=20)
	axarr[0, 0].set_ylabel('rad', fontsize=20)
	axarr[0, 0].tick_params(labelsize=12)
	
	axarr[0, 1].plot(t, trajN5_xHistory[:L,1], 'b', label='N = 5')
	axarr[0, 1].plot(t, trajN10_xHistory[:L,1], 'g', label='N = 10')
	axarr[0, 1].plot(t, trajN10_xRefHistory[:L,1],'r-', label='Reference')
	axarr[0, 1].set_title('Joint 2 Position', fontsize=20)
	axarr[0, 1].set_xlabel('time', fontsize=20)
	axarr[0, 1].set_ylabel('rad', fontsize=20)
	axarr[0, 1].tick_params(labelsize=12)

	axarr[1, 0].plot(t, trajN5_xHistory[:L,2], 'b', label='N = 5')
	axarr[1, 0].plot(t, trajN10_xHistory[:L,2], 'g', label='N = 10')
	axarr[1, 0].plot(t, trajN10_xRefHistory[:L,2],'r-', label='Reference')
	axarr[1, 0].set_title('Joint 1 Velocity', fontsize=20)
	axarr[1, 0].set_xlabel('time', fontsize=20)
	axarr[1, 0].set_ylabel('rad/s', fontsize=20)
	axarr[1, 0].tick_params(labelsize=12)

	axarr[1, 1].plot(t, trajN5_xHistory[:L,3], 'b', label='N = 5')
	axarr[1, 1].plot(t, trajN10_xHistory[:L,3], 'g', label='N = 10')
	axarr[1, 1].plot(t, trajN10_xRefHistory[:L,3],'r-', label='Reference')
	axarr[1, 1].set_title('Joint 2 Velocity', fontsize=20)
	axarr[1, 1].set_xlabel('time', fontsize=20)
	axarr[1, 1].set_ylabel('rad/s', fontsize=20)
	axarr[1, 1].tick_params(labelsize=12)
	plt.legend(prop={'size':15})

	plt.figure()
	plt.plot(t, trajN5_uHistory[:L], 'b', label='N = 5')
	plt.plot(t, trajN10_uHistory[:L], 'g', label='N = 10')
	plt.title('Actuator Torque', fontsize=20)
	plt.xlabel('time', fontsize=20)
	plt.ylabel('N-m', fontsize=20)
	plt.tick_params(labelsize=12)
	plt.legend(prop={'size':25})
	plt.show()
	# t = np.linspace(0, waypN5_params['Duration'], waypN5_params['Duration']/waypN5_params['Ts']+1)
	# f, axarr = plt.subplots(2, 2)
	# L = t.size
	# axarr[0, 0].plot(t, waypN5_xHistory[:L,0], 'b', label='N = 5')
	# axarr[0, 0].plot(t, waypN10_xHistory[:L,0], 'g', label='N = 10')
	# axarr[0, 0].plot(t, waypN10_xRefHistory[:L,0],'r-', label='Reference')
	# axarr[0, 0].set_title('Joint 1 Position', fontsize=20)
	# axarr[0, 0].set_xlabel('time', fontsize=20)
	# axarr[0, 0].set_ylabel('rad', fontsize=20)
	# axarr[0, 0].tick_params(labelsize=12)
	
	# axarr[0, 1].plot(t, waypN5_xHistory[:L,1], 'b', label='N = 5')
	# axarr[0, 1].plot(t, waypN10_xHistory[:L,1], 'g', label='N = 10')
	# axarr[0, 1].plot(t, waypN10_xRefHistory[:L,1],'r-', label='Reference')
	# axarr[0, 1].set_title('Joint 2 Position', fontsize=20)
	# axarr[0, 1].set_xlabel('time', fontsize=20)
	# axarr[0, 1].set_ylabel('rad', fontsize=20)
	# axarr[0, 1].tick_params(labelsize=12)

	# axarr[1, 0].plot(t, waypN5_xHistory[:L,2], 'b', label='N = 5')
	# axarr[1, 0].plot(t, waypN10_xHistory[:L,2], 'g', label='N = 10')
	# axarr[1, 0].plot(t, waypN10_xRefHistory[:L,2],'r-', label='Reference')
	# axarr[1, 0].set_title('Joint 1 Velocity', fontsize=20)
	# axarr[1, 0].set_xlabel('time', fontsize=20)
	# axarr[1, 0].set_ylabel('rad/s', fontsize=20)
	# axarr[1, 0].tick_params(labelsize=12)

	# axarr[1, 1].plot(t, waypN5_xHistory[:L,3], 'b', label='N = 5')
	# axarr[1, 1].plot(t, waypN10_xHistory[:L,3], 'g', label='N = 10')
	# axarr[1, 1].plot(t, waypN10_xRefHistory[:L,3],'r-', label='Reference')
	# axarr[1, 1].set_title('Joint 2 Velocity', fontsize=20)
	# axarr[1, 1].set_xlabel('time', fontsize=20)
	# axarr[1, 1].set_ylabel('rad/s', fontsize=20)
	# axarr[1, 1].tick_params(labelsize=12)
	# plt.legend(prop={'size':15})

	# plt.figure()
	# plt.plot(t, waypN5_uHistory[:L], 'b', label='N = 5')
	# plt.plot(t, waypN10_uHistory[:L], 'g', label='N = 10')
	# plt.title('Actuator Torque', fontsize=20)
	# plt.xlabel('time', fontsize=20)
	# plt.ylabel('N-m', fontsize=20)
	# plt.tick_params(labelsize=12)
	# plt.legend(prop={'size':25})
	# plt.show()