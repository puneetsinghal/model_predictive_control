# Author:
#     Puneet Singhal: puneetatpks@gmail.com
#     Aditya Raghu Chavvali:

# For detailed information on the minimize function visit:
# https://docs.scipy.org/doc/scipy/reference/tutorial/optimize.html
import sys
sys.path.insert(0, './..')
import matplotlib.pyplot as plt
import numpy as np
from math import *
import time
import pickle
import argparse
from copy import copy
import scipy.io as sio

from robot import Acrobot
from robot import PlanarRR

from mpc import MPC
from IPython import embed

if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	parser.add_argument('--mode', type=str, default='test')
	parser.add_argument('--model', type=str, default='planarRR_trajectory.dat')
	parser.add_argument('--type', type=str, default='waypoints')

	args = parser.parse_args()
	t = time.time()

	params = {}
	params['m1'] = 1.
	params['m2'] = 1.
	params['l1'] = 0.5
	params['l2'] = 0.5
	params['g'] = -9.81
	params['I1'] = 0.1
	params['I2'] = 0.1
	params['x0'] = np.array([0., 0., 0., 0.])		# current state space
	params['u0'] = np.array([0., 0.])				# initial force

	LB = -100.									# input force Lower Bound 
	UP = 100.									# input force Upper Bound
	
	if (args.type == 'waypoints'):
		params['Ts'] = 0.05						# time iteration
		params['N'] = 5							# Event horizon = 10
		params['Duration'] = 2.0				# max time 
		params['numIterations'] = int(params['Duration']/params['Ts'])
		# params['waypoints'] = np.array([[0, 0.7022, 0.5790, 0.0513, -1.0845, -1.0845, 1.3254, 2.1820, 2.6058, 3.1416], 
		#                         [0, -1.2763, -2.2228, -2.0331, 0.0265, 2.4347, 4.1818, 4.4895, 3.9700, 3.1416],
		#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
		#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

		params['waypoints'] = np.array([[0, 0.7022, 0.5790, 0.0513, -3.0, -3.0, 1.3254, 2.1820, 2.6058, 3.1416, 3.1416, 3.1416], 
								[0, -1.2763, -2.2228, -2.0331, 0.0265, 0.0265, 4.1818, 4.4895, 3.9700, 3.1416, 3.1416, 3.1416],
								[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
								[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
		
		params['waypoints'][1,:] = params['waypoints'][1,:] - params['waypoints'][0,:]
		params['waypoints'][3,:] = params['waypoints'][3,:] - params['waypoints'][2,:]
		params['trajectory']= None
		params['Q'] = np.diag([100.,10.,0.1, 0.1])
		params['R'] = np.diag([0.01, 0.01])
		bnds = ((LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP))#, (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP))

	else:
		params['Ts'] = 0.01				# time iteration
		params['N'] = 5				# Event horizon
		params['waypoints'] = None
		params['Duration'] = 12			# max time 
		params['numIterations'] = int(params['Duration']/params['Ts'])

		joints = pickle.load(open('../Trajectory/planarRR_trajectory', 'rb'))
		# embed()
		params['trajectory'] = (np.array(joints).T)[:4,:]
		print(params['trajectory'].shape)
		params['x0'] = params['trajectory'][:,0]
		# params['Q'] = np.diag([100.,10.,0.1, 0.1])
		params['Q'] = np.diag([1., 1., 100., 100.])
		params['R'] = np.diag([0.01, 0.01])
		bnds = ((LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP))

	print('total number of iterations are: {}'.format(params['numIterations']))
	uopt = np.zeros([params['N'], 2]).reshape(params['N']*2,)
	u0 = params['u0']
	x = params['x0']
	
	# arrays to save data
	uHistory = np.zeros((params['numIterations']+1, 2))
	uHistory[0,:] = u0 						# force history
	xHistory = np.zeros((params['numIterations']+1, 4))
	xHistory[0, :] = params['x0']#[:,0]      	# position history
	# xRefHistory = np.zeros((params['numIterations']+1, 4))

	robot = PlanarRR(params)
	print("Robot object created")

	controller = MPC(robot, params, bnds, args.type)
	print("Controller object created")

	# cons = ({'type': 'ineq', 'fun': Contraints, 'args':(x, robot,)})
	# contraint jacobian can also be added to possibly speed up 

	if (args.mode == 'train'):
		for ct in range(params['numIterations']):
			t = time.time()
			print('iteration # {} of {}'.format(ct, params['numIterations']))

			# optimize
			results = controller.optimize(uopt, x, u0)

			# prepare variable for next run and save the output in history arrays
			uopt = results.x
			print(uopt.shape)
			u0 = uopt.reshape(params['N'], 2)[0]
			x = controller.IntegrationEstimation(x, u0, 30)
			print("State Achieved: {}".format(x.T))
			print("Time taken: {}".format((time.time()-t)))
			uHistory[ct+1,:] = u0
			xHistory[ct+1,:] = x#[:,0]

		print(time.time()-t)
		controller.xRefHistory+= [controller.xRefHistory[ct]]
		xRefHistory = np.array(controller.xRefHistory)
		filename = './models/acrobot_results_' + str(params['N'])
		pickle.dump([params, xHistory, uHistory, xRefHistory], open(filename, 'wb'))
	else:
		filename = args.model
		params, xHistory, uHistory, xRefHistory = pickle.load(open(filename, 'rb'))   
	
	# Displaying Graphs
	t = np.linspace(0, params['Duration'], params['Duration']/params['Ts']+1)
	# xHistory = np.array(xHistory)
	# print(xRefHistory)
	# print(xRefHistory.shape)
	f, axarr = plt.subplots(2, 2)
	axarr[0, 0].plot(t, xHistory[:,0], 'r')
	axarr[0, 0].plot(t, xRefHistory[:,0], 'g')
	axarr[0, 0].set_title('Joint 1 Position')
	axarr[0, 0].set_xlabel('time', fontsize=15)
	axarr[0, 0].set_ylabel('theta_1', fontsize=15)
	axarr[0, 0].tick_params(labelsize=15)
	axarr[0, 0].grid(True)

	axarr[0, 1].plot(t, xHistory[:,1], 'r')
	axarr[0, 1].plot(t, xRefHistory[:,1], 'g')
	axarr[0, 1].set_title('Joint 2 Position')
	axarr[0, 1].set_xlabel('time', fontsize=15)
	axarr[0, 1].set_ylabel('theta_2', fontsize=15)
	axarr[0, 1].tick_params(labelsize=15)
	axarr[0, 1].grid(True)

	axarr[1, 0].plot(t, xHistory[:,2], 'r')
	axarr[1, 0].plot(t, xRefHistory[:,2], 'g')
	axarr[1, 0].set_title('Joint 1 Velocity')
	axarr[1, 0].set_xlabel('time', fontsize=15)
	axarr[1, 0].set_ylabel('ang_vel_1', fontsize=15)
	axarr[1, 0].tick_params(labelsize=15)
	axarr[1, 0].grid(True)

	axarr[1, 1].plot(t, xHistory[:,3], 'r')
	axarr[1, 1].plot(t, xRefHistory[:,3], 'g')
	axarr[1, 1].set_title('Joint 2 Velocity')
	axarr[1, 1].set_xlabel('time', fontsize=15)
	axarr[1, 1].set_ylabel('ang_vel_2', fontsize=15)
	axarr[1, 1].tick_params(labelsize=15)
	axarr[1, 1].grid(True)

	fig, ax = plt.subplots()
	ax.plot(t, uHistory[:, 0], 'b', label='torque_1')
	ax.plot(t, uHistory[:, 1], 'k', label='torque_2')
	legend = ax.legend(loc='upper right', shadow=True)
	ax.set_xlabel('time', fontsize=15)
	ax.set_ylabel('inputs', fontsize=15)
	ax.tick_params(labelsize=15)
	ax.grid(True)
	# embed()
	# try:
	# 	robot.animate(xHistory)
	# except:
	# 	print("Problem with creating animation. Please check the function definition")
	plt.show()

