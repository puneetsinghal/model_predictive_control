from scipy.integrate import ode
import scipy.io as sio
import argparse
import numpy as np
import tensorflow as tf
from copy import copy   
from collections import deque
from IPython import embed
import matplotlib.pyplot as plt
from RNN import RNNNetwork
from data_processor import DB_Processor
from robot import Acrobot
from robot import PlanarRR


def IntegrationEstimation(xk, uk, Ts, robot, M = 5):
	# Runge-Kutta 4th order (M = 5 optimization problem, M = 30 updating state space)
	# Better ODE solvers can be used here 
	delta = Ts/M
	xk1 = xk
	for ct in range(M):
		k1 = robot.dynamics(xk1, uk)
		k2 = robot.dynamics(xk1 + k1*delta/2, uk)
		k3 = robot.dynamics(xk1 + k2*delta/2, uk)
		k4 = robot.dynamics(xk1 + k3*delta, uk)
		xk1 = xk1 + delta*(k1/6 + k2/3 + k3/3 + k4/6)
	return xk1


def Purturb(x, percent):
	x_p = x * (1 + percent * 2*(np.random.rand(1) - 0.5))
	return x_p 

def PurturbParams(purturbed_params, percentage):
	purturbed_params = {}
	purturbed_params['m1'] = Purturb(params['m1'], percentage)
	purturbed_params['m2'] = Purturb(params['m2'], percentage)
	purturbed_params['l1'] = Purturb(params['l1'], percentage)
	purturbed_params['l2'] = Purturb(params['l2'], percentage) 

	purturbed_params['I1'] = Purturb(params['I1'], percentage)
	purturbed_params['I2'] = Purturb(params['I2'], percentage)

	purturbed_params['x0'] = params['x0'] # current state space
	purturbed_params['Ts'] = params['Ts']             # time iteration
	purturbed_params['g'] = params['g']
	purturbed_params['N'] = params['N']             # Event horizon = 10
	purturbed_params['u0'] = params['u0']# initial force
	return purturbed_params


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--model',  type=str,	default=None)
	parser.add_argument('--data',   type=str,	default='data_for_rnn_5000.dat')
	args = parser.parse_args()

	# Set up analytical model
	params 			= {}
	params['m1']    = 1.
	params['m2']    = 1.
	params['l1']    = 0.5
	params['l2']    = 0.5 
	params['g']     = -9.81
	params['I1']    = 0.1
	params['I2']    = 0.1
	
	params['Ts']    = 0.01             # time iteration
	params['N']     = 1              # Event horizon = 10
	Ts              = params['Ts']
	
	model_robot     = PlanarRR(params)

	# Set up uncertainty real model
	# percentage = 0.05  # the parameters are +- 5 percent uncertain
	# purturbed_params = PurturbParams(params, percentage)
	# real_robot = PlanarRR(purturbed_params)
	
	"""
	# Simulate responses on real robot
	for i in range(int(10.0/Ts)):
		# Decide the optimal usage
		# Use 0 usage for now
		uk = np.array(0).reshape(1,1)
		if i == 0:
			uHistory = uk
		else:
			uHistory = np.append(uHistory, uk, axis=1)
		
		xk = IntegrationEstimation(xk, uk, Ts, real_robot, 30) 
		xHistory = np.append(xHistory, xk, axis=1)
	
	print(uHistory)
	np.save('xHistory', xHistory)
	np.save('uHistory', uHistory)
	"""
	# xHistory = np.load('xHistory.npy')
	# uHistory = np.load('uHistory.npy')
	# data_points = uHistory.shape[1]
	
	# Set up neural network model
	model_path      = args.model
	log_path        = './logs/test'

	# network = mpcNetwork.Network(lrn_rate, total_epoch, batch_size, network_type, model_path,
			# log_path)
	# Network parameters
	time_steps          = 5
	batch_size          = 1024
	input_state_size    = 6 # [sita1 w1 sita2 w2 torque]_t
	output_state_size   = 4 # [sita1 w1 sita2 w2]_t+1
	hidden_state_size   = 8
	num_epoch           = 100000
	lrn_rate            = 1e-3
	dropout_prob        = 0.95

	# input_epoch, output_epoch, num_batch = \
	#             DB_Processor.gen_epoch(args.data, batch_size, time_steps, self.input_state_size, self.output_state_size)
	sess = tf.Session()
	
	rawData         = DB_Processor.loadRawData(args.data, input_state_size, output_state_size)
	rawInputData    = rawData[:, :-1, :]
	rawOutputData   = rawData[:, 1:, :]

	network         = RNNNetwork(lrn_rate, input_state_size, hidden_state_size, output_state_size)
	network.load_model_weights(sess, model_path) 
	# See response on our model
	errorHistory = []
	errorHistory_net = []

	for i in range(rawData.shape[0]):
		# xk                              = np.array(rawInputData[i,0,0])
		input_state                     = deque(maxlen=time_steps)
		input_state_with_prediction     = deque(maxlen=time_steps)
		print("running batch number: {}".format(i))
		for t in range(rawData.shape[1] - params['N']):
			xk                              = (rawInputData[i,t,0])
			uk                              = (rawInputData[i,t,1])
			input_state.append(np.array([xk + uk])[0])
			
			input_state_with_prediction = copy(input_state)
			error = 0.

			for pred_step in range(params['N']):                   
				if (pred_step > 0):
					nextState 			= np.zeros([1,6])
					nextState[0,0:4] 	= copy(x_pred_net)
					uk 					= np.array(rawData[i, t + pred_step, 1])
					nextState[0,4:] 	= uk
					input_state_with_prediction.append(nextState[0])
				
				# Neural Model
				# embed()
				feed_dict 	= {network.network_batch_size:1, network.prev_state:np.array(input_state_with_prediction)[np.newaxis,:,:],
								network.dropout_prob:1.0}
				x_pred_net 	= sess.run([network.final_prediciton], feed_dict= feed_dict)[0]	
				
				xk1 		= np.array(rawData[i,t + pred_step + 1, 0])
				error 		+= np.mean(np.square(xk1-x_pred_net))
				# error_net = np.square(xHistory[:,i+j+1].reshape(4,1) - x_pred_net.reshape(4,1))

			errorHistory.append(error)
				# errorHistory_net.append(error_net)
	
	errorHistory = np.array(errorHistory)
	print(np.mean(errorHistory))
	np.save('errorHistory', errorHistory)
	plt.figure()
	plt.plot(errorHistory)
	plt.show()
	# print(np.mean(np.sqrt(errorHistory), axis=1, keepdims=True))  
	# print(np.mean(np.sqrt(errorHistory_net), axis=1, keepdims=True))  
	  
	sess.close()