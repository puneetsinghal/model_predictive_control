import numpy as np 
from robot import *
from IPython import embed
import pickle
from math import pi

def IntegrationEstimation(xk, uk, Ts, M = 5):
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

def totuple(a):
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a

params = {}
params['m1'] = 1.
params['m2'] = 1.
params['l1'] = 0.5
params['l2'] = 0.5 
params['g'] = -9.81
params['I1'] = 0.1
params['I2'] = 0.1

params['Ts'] = 0.01             # time iteration
params['N'] = 10              # Event horizon = 10

robot = PlanarRR(params)

T = 0.5
N = 100
PIK = 'data_for_rnn_' + str(int(N*T/params['Ts'])) + '.dat'
data = []

with open(PIK, "wb") as f:

	for j in range(N):

		params['x0'] = pi*np.array(np.random.uniform(-1,1,4))
		params['u0'] = np.array(np.random.uniform(-1,1,2))

		# energy = np.zeros((int(10.0/params['Ts']),1))
		xk = params['x0']
		uk = params['u0']
		xHistory = [xk.tolist()]
		subData = []
		for i in range(int(T/params['Ts'])):
			xk = IntegrationEstimation(xk, uk, params['Ts'], 30)
			subData.append((xk[:].tolist(),uk[:].tolist()))
			# subData.append([totuple(xk[:]),totuple(uk[:])])
		data.append(subData)
		print(len(data))
	# embed()
	pickle.dump(data, f)

# with open(PIK, "rb") as f:
#     print (pickle.load(f)[9000])
