# Author:
#     Puneet Singhal: puneetatpks@gmail.com
#     Aditya Raghu Chavvali: adikiran007@gmail.com

# For detailed information on the minimize function visit:
# https://docs.scipy.org/doc/scipy/reference/tutorial/optimize.html

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
import numpy as np
from math import *
import time
import pickle
import argparse
from copy import copy
import scipy.io as sio

from robot import *
from mpc import MPC

if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	parser.add_argument('--mode', type=str, default='train')
	parser.add_argument('--model', type=str, default='./models/acrobot_results')
	parser.add_argument('--type', type=str, default='waypoints')

	args = parser.parse_args()
	t = time.time()


	params = {}
	params['m1'] = 1.
	params['m2'] = 1.
	params['m3'] = 1.
	params['l1'] = 0.5
	params['l2'] = 0.5
	params['l3'] = 0.5
	params['g'] = -9.81
	params['I1'] = 0.0
	params['I2'] = 0.0
	params['I3'] = 0.0
	params['x0'] = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])		# current state space - edited for 5DOF
	params['u0'] = np.array([0.,0.,0.,0.,0.])							# initial force - edited

	#LB = np.array([-100., -100., -100., -100.,-100.])		# input force Lower Bound - modified for 5 DOF
	#UP = np.array([100., 100., 100., 100.,100.])			# input force Upper Bound - modified for 5DOF
	
	LB = -100.0		# input force Lower Bound - modified for 5 DOF
	UP = 100.			# input force Upper Bound - modified for 5DOF
	 
    
	if (args.type == 'waypoints'):
		params['Ts'] = 0.05						# time iteration
		params['N'] = 5							# Event horizon = 10
		params['Duration'] = 2.0				# max time 
		params['numIterations'] = int(params['Duration']/params['Ts'])
		# params['waypoints'] = np.array([[0, 0.7022, 0.5790, 0.0513, -1.0845, -1.0845, 1.3254, 2.1820, 2.6058, 3.1416], 
		#                         [0, -1.2763, -2.2228, -2.0331, 0.0265, 2.4347, 4.1818, 4.4895, 3.9700, 3.1416],
		#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
		#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

		#params['waypoints'] = np.array([[0, 0.7022, 0.5790, 0.0513, -3.0, -3.0, 1.3254, 2.1820, 2.6058, 3.1416, 3.1416, 3.1416], 
		#						[0, -1.2763, -2.2228, -2.0331, 0.0265, 0.0265, 4.1818, 4.4895, 3.9700, 3.1416, 3.1416, 3.1416],
		#						[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
		#						[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
		params['waypoints'] = np.array ([[ 1.70143204, 1.70143204, 1.70143204, 0.64113454, 0.64113454, 0.64113454, 1.68242039, 1.68242039, 1.68242039],
									[-0.87753143,-1.06585094,-0.87753143,-0.62944913,-0.86225178,-0.62944913,-0.53075948,-0.53075948,-0.53075948],
									[ 0.91630422, 1.02217488, 0.91630422, 1.34251619, 1.4312865, 1.34251619, 1.50072625, 1.50072625, 1.50072625],
									[-1.347757,-1.05356684,-1.347757,-1.16962734,-0.84805437,-1.16962734,-1.11010693,-1.11010693,-1.11010693],
									[0.13063571, 0.13063571, 0.13063571, 0.64113454, 0.64113454, -0.92966179, 0.11162407, 0.11162407,0.11162407],
									[0,0,0,0,0,0,0,0,0],
									[0,0,0,0,0,0,0,0,0],
									[0,0,0,0,0,0,0,0,0],
									[0,0,0,0,0,0,0,0,0],
									[0,0,0,0,0,0,0,0,0]])
		
		#params['waypoints'][1,:] = params['waypoints'][1,:] - params['waypoints'][0,:]
		#params['waypoints'][3,:] = params['waypoints'][3,:] - params['waypoints'][2,:]
		params['waypoints'][1,:] = params['waypoints'][1,:] - params['waypoints'][0,:]
		params['waypoints'][2,:] = params['waypoints'][2,:] - params['waypoints'][1,:]
		params['waypoints'][3,:] = params['waypoints'][3,:] - params['waypoints'][2,:]
		params['waypoints'][4,:] = params['waypoints'][4,:] - params['waypoints'][3,:]
		
		params['waypoints'][6,:] = params['waypoints'][6,:] - params['waypoints'][5,:]
		params['waypoints'][7,:] = params['waypoints'][7,:] - params['waypoints'][6,:]
		params['waypoints'][8,:] = params['waypoints'][8,:] - params['waypoints'][7,:]
		params['waypoints'][9,:] = params['waypoints'][9,:] - params['waypoints'][8,:]

		params['trajectory']= None
		params['Q'] = np.diag([100.,10.,10., 10.,0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
		params['R'] = np.diag([0.01, 0.01, 0.01, 0.01,0.01])
		bnds = ((LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP))

	else:
		params['Ts'] = 0.02				# time iteration
		params['N'] = 20				# Event horizon
		params['waypoints'] = None
		params['Duration'] = 2.		# max time 
		params['numIterations'] = int(params['Duration']/params['Ts'])

		data = sio.loadmat('acrobot_trajectory.mat')
		params['trajectory']= data['z']
		params['trajectory'][1,:] = params['trajectory'][1,:] - params['trajectory'][0,:]
		params['trajectory'][2,:] = params['trajectory'][2,:] - params['trajectory'][1,:]
		params['trajectory'][3,:] = params['trajectory'][3,:] - params['trajectory'][2,:]
		params['trajectory'][4,:] = params['trajectory'][4,:] - params['trajectory'][3,:]
		
		params['trajectory'][6,:] = params['trajectory'][6,:] - params['trajectory'][5,:]
		params['trajectory'][7,:] = params['trajectory'][7,:] - params['trajectory'][6,:]
		params['trajectory'][8,:] = params['trajectory'][8,:] - params['trajectory'][7,:]
		params['trajectory'][9,:] = params['trajectory'][9,:] - params['trajectory'][8,:]
		# print(params['trajectory'].shape)
		# params['Q'] = np.diag([100.,10.,0.1, 0.1])
		params['Q'] = np.diag([1., 1., 1., 1.])
		params['R'] = 0.01
		bnds = ((LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP))

	print('total number of iterations are: {}'.format(params['numIterations']))
	uopt = np.zeros(params['N']) 
	u0 = params['u0']
	x = params['x0']
	
	# arrays to save data

	

	uHistory = np.zeros((params['numIterations']+1,5))
	uHistory[0,:] = u0             # force history
	xHistory = np.zeros((params['numIterations']+1, 10))
	xHistory[0,:] = params['x0']#[:,0]      # position history
	xRefHistory = np.zeros((params['numIterations']+1, 10))
    
	##################### KALYAN THIS HAS TO BE CHANGED##################
	robot = KdcArm(params)
	print("Robot object created")

	#print(robot.dynamics( [1., 0., 0., 0., 0., 0., 0., 0., 0., 0.] , [.0,.0,.0,.0,.0]))

	

	controller = MPC(robot, params, bnds, args.type)
	print("Controller object created")    

    # cons = ({'type': 'ineq', 'fun': Contraints, 'args':(x, robot,)})
	# contraint jacobian can also be added to possibly speed up 

	if (args.mode == 'train'):
		for ct in range(params['numIterations']):
			t = time.time()
			print('iteration # {} of {}'.format(ct, params['numIterations']))
			# optimize
			for i in range(len(u0)):            
				results = controller.optimize(uopt, x, u0[i])
				# prepare variable for next run and save the output in history arrays
				uopt = results.x  
				u0[i] = uopt[0] # -- This must be a vector, loop and accumulate
			x = controller.IntegrationEstimation(x, u0, 30)
			#print("State Achieved: {}".format(x.T))
			#print("Time taken: {}".format((time.time()-t)))
			uHistory[ct+1] = u0
			xHistory[ct+1,:] = x#[:,0]

		#print(time.time()-t)       
		controller.xRefHistory+= [controller.xRefHistory[ct]]
		xRefHistory = np.array(controller.xRefHistory)
		filename = './models/acrobot_results'
		pickle.dump([params, xHistory, uHistory, xRefHistory], open(filename, 'wb'))
	else:
		filename = args.model
		params, xHistory, uHistory, xRefHistory = pickle.load(open(filename, 'rb'))   
	'''	
	######################## TO BE CHANGED - X IS NOW 10 DIMENSIONAL COPY -PASTE THE SAME ###################
	# Displaying Graphs
	t = np.linspace(0, params['Duration'], params['Duration']/params['Ts']+1)
	# xHistory = np.array(xHistory)
	# print(xRefHistory)
	# print(xRefHistory.shape)
	f, axarr = plt.subplots(2, 2)
	axarr[0, 0].plot(t, xHistory[:,0])
	axarr[0, 0].plot(t, xRefHistory[:,0])
	axarr[0, 0].set_title('Joint 1 Position')
	axarr[0, 0].set_xlabel('time')
	axarr[0, 0].set_ylabel('theta_1')

	axarr[0, 1].plot(t, xHistory[:,1])
	axarr[0, 1].plot(t, xRefHistory[:,1])
	axarr[0, 1].set_title('Joint 2 Position')
	axarr[0, 1].set_xlabel('time')
	axarr[0, 1].set_ylabel('theta_2')

	axarr[1, 0].plot(t, xHistory[:,2])
	axarr[1, 0].plot(t, xRefHistory[:,2])
	axarr[1, 0].set_title('Joint 1 Velocity')
	axarr[1, 0].set_xlabel('time')
	axarr[1, 0].set_ylabel('ang_vel_1')

	axarr[1, 1].plot(t, xHistory[:,3])
	axarr[1, 1].plot(t, xRefHistory[:,3])
	axarr[1, 1].set_title('Joint 2 Velocity')
	axarr[1, 1].set_xlabel('time')
	axarr[1, 1].set_ylabel('ang_vel_2')

	plt.figure()
	plt.plot(t, uHistory)
	plt.legend('Actuator Torque')
	plt.xlabel('time')
	plt.ylabel('inputs')

	robot.animate(xHistory)
	plt.show()
	'''

