from scipy.integrate import ode
from robot import Acrobot
import numpy as np
from collections import deque
from RNN import RNNNetwork
import tensorflow as tf
import argparse
#from IPython import embed
import matplotlib.pyplot as plt



def IntegrationEstimation(xk, uk, Ts, robot,M = 5):
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


def Energy(params, xk):
    KE = 0.5*params['m1']*(params['l1']**2)/4*xk[2,0]**2 + \
            params['m2']*(0.5*params['l1']**2*xk[2,0]**2 + \
            0.5*(params['l2']**2)/4*(xk[2,0] + xk[3,0])**2 + \
            params['l1']*params['l2']/2*xk[2,0]*(xk[2,0] + xk[3,0]))*np.cos(xk[1,0])+ \
            0.5*params['I1']*xk[2,0]**2 + 0.5*params['I2']*xk[3,0]**2

    PE = params['m1']*params['l1']/2*params['g']*(np.cos(xk[0,0])) + \
            params['m2']*params['g']*(params['l1']*np.cos(xk[0,0])+ \
            params['l2']/2*np.cos(xk[0,0] + xk[1,0]))

    return KE + PE

def testAnalyticalModel(params):
	robot = Acrobot(params)

	# results = ode45(@(t,x)acrobotDynamicsCT(t, x, u, params), linspace(0,4,4/params.Ts), x0);

	# N = size(results.x,2);
	energy = np.zeros((int(10.0/params['Ts']),1))
	xk = params['x0']
	uk = 0
	xHistory = [xk[:,0].tolist()]

	for i in range(int(10.0/params['Ts'])):
		xk = IntegrationEstimation(xk, uk, params['Ts'], robot, 30)
		energy[i] = Energy(params, xk)
		xHistory += [xk[:,0].tolist()]
    
	#robot.animate(np.array(xHistory))
	plt.plot(energy)
	plt.show()

def testRNNModel(params, modelName):
	robot = Acrobot(params)
	network = RNNNetwork(lrn_rate=0.0001, input_state_size=5, hidden_state_size=16, output_state_size=4)
	
	energy = np.zeros((int(10.0/params['Ts']),1))
	xk = np.array([0.4, 0, 0., 0., 0.]).reshape((1,5))
	xHistory = [xk[0,:4].tolist()]

	input_state = deque(maxlen=5)
	input_state.append(xk[0])
	nextState = np.zeros([1,5])
	with tf.Session() as sess:
		network.load_model_weights(sess, modelName)
		for i in range(5000):
			# embed()
			feed_dict = {network.network_batch_size:1, network.prev_state:np.array(input_state)[np.newaxis,:,:],
							network.dropout_prob:1.0}
			predicted_x = sess.run([network.final_prediciton], feed_dict= feed_dict)[0]
			nextState[0,0:4] = predicted_x
			# nextState[0,4] = 0.1
			input_state.append(nextState[0])
			xHistory += [nextState[0,0:4].tolist()]

	robot.animate(np.array(xHistory))
	embed()

if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	parser.add_argument('--model', type=str, default=None)
	args = parser.parse_args()

	params = {}
	params['m1'] = 1.
	params['m2'] = 1.
	params['l1'] = 0.5
	params['l2'] = 0.5 
	params['g'] = -9.81
	params['I1'] = 0.1
	params['I2'] = 0.1

	params['x0'] = np.array([1.0, 0, 0, 0]).reshape((4,1)) # current state space
	params['Ts'] = 0.01             # time iteration
	params['N'] = 10              # Event horizon = 10
	params['u0'] = 0               # initial force

	# testAnalyticalModel(params)

	testRNNModel(params)




