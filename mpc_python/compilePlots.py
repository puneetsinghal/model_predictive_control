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
	# t = np.linspace(0, trajN5_params['Duration'], trajN5_params['Duration']/trajN5_params['Ts']+1)
	# f, axarr = plt.subplots(2, 2)
	# axarr[0, 0].plot(t, trajN5_xHistory[:,0], 'b', label='N = 5')
	# axarr[0, 0].plot(t, trajN10_xHistory[:,0], 'g', label='N = 10')
	# axarr[0, 0].plot(t, trajN10_xRefHistory[:,0],'r-', label='Reference')
	# axarr[0, 0].set_title('Joint 1 Position')
	# axarr[0, 0].set_xlabel('time')
	# axarr[0, 0].set_ylabel('rad')

	# axarr[0, 1].plot(t, trajN5_xHistory[:,1], 'b', label='N = 5')
	# axarr[0, 1].plot(t, trajN10_xHistory[:,1], 'g', label='N = 10')
	# axarr[0, 1].plot(t, trajN10_xRefHistory[:,1],'r-', label='Reference')
	# axarr[0, 1].set_title('Joint 2 Position')
	# axarr[0, 1].set_xlabel('time')
	# axarr[0, 1].set_ylabel('rad')

	# axarr[1, 0].plot(t, trajN5_xHistory[:,2], 'b', label='N = 5')
	# axarr[1, 0].plot(t, trajN10_xHistory[:,2], 'g', label='N = 10')
	# axarr[1, 0].plot(t, trajN10_xRefHistory[:,2],'r-', label='Reference')
	# axarr[1, 0].set_title('Joint 1 Velocity')
	# axarr[1, 0].set_xlabel('time')
	# axarr[1, 0].set_ylabel('rad/s')

	# axarr[1, 1].plot(t, trajN5_xHistory[:,3], 'b', label='N = 5')
	# axarr[1, 1].plot(t, trajN10_xHistory[:,3], 'g', label='N = 10')
	# axarr[1, 1].plot(t, trajN10_xRefHistory[:,3],'r-', label='Reference')
	# axarr[1, 1].set_title('Joint 2 Velocity')
	# axarr[1, 1].set_xlabel('time')
	# axarr[1, 1].set_ylabel('rad/s')
	# plt.legend()

	# plt.figure()
	# plt.plot(t, trajN5_uHistory, 'b', label='N = 5')
	# plt.plot(t, trajN10_uHistory, 'g', label='N = 10')
	# plt.title('Actuator Torque')
	# plt.xlabel('time')
	# plt.ylabel('N-m')
	# plt.legend()
	# plt.show()

	t = np.linspace(0, waypN5_params['Duration'], waypN5_params['Duration']/waypN5_params['Ts']+1)
	f, axarr = plt.subplots(2, 2)
	L = t.size
	axarr[0, 0].plot(t, waypN5_xHistory[:L,0], 'b', label='N = 5')
	axarr[0, 0].plot(t, waypN10_xHistory[:L,0], 'g', label='N = 10')
	axarr[0, 0].plot(t, waypN10_xRefHistory[:L,0],'r-', label='Reference')
	axarr[0, 0].set_title('Joint 1 Position')
	axarr[0, 0].set_xlabel('time')
	axarr[0, 0].set_ylabel('rad')

	axarr[0, 1].plot(t, waypN5_xHistory[:L,1], 'b', label='N = 5')
	axarr[0, 1].plot(t, waypN10_xHistory[:L,1], 'g', label='N = 10')
	axarr[0, 1].plot(t, waypN10_xRefHistory[:L,1],'r-', label='Reference')
	axarr[0, 1].set_title('Joint 2 Position')
	axarr[0, 1].set_xlabel('time')
	axarr[0, 1].set_ylabel('rad')

	axarr[1, 0].plot(t, waypN5_xHistory[:L,2], 'b', label='N = 5')
	axarr[1, 0].plot(t, waypN10_xHistory[:L,2], 'g', label='N = 10')
	axarr[1, 0].plot(t, waypN10_xRefHistory[:L,2],'r-', label='Reference')
	axarr[1, 0].set_title('Joint 1 Velocity')
	axarr[1, 0].set_xlabel('time')
	axarr[1, 0].set_ylabel('rad/s')

	axarr[1, 1].plot(t, waypN5_xHistory[:L,3], 'b', label='N = 5')
	axarr[1, 1].plot(t, waypN10_xHistory[:L,3], 'g', label='N = 10')
	axarr[1, 1].plot(t, waypN10_xRefHistory[:L,3],'r-', label='Reference')
	axarr[1, 1].set_title('Joint 2 Velocity')
	axarr[1, 1].set_xlabel('time')
	axarr[1, 1].set_ylabel('rad/s')
	plt.legend()

	plt.figure()
	plt.plot(t, waypN5_uHistory[:L], 'b', label='N = 5')
	plt.plot(t, waypN10_uHistory[:L], 'g', label='N = 10')
	plt.title('Actuator Torque')
	plt.xlabel('time')
	plt.ylabel('N-m')
	plt.legend()
	plt.show()