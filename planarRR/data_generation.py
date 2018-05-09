import numpy as np 
from robot import Acrobot
from IPython import embed
import pickle

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

robot = Acrobot(params)

PIK = "data_for_rnn_actual.dat"
save = list()

with open(PIK, "wb") as f:

	for j in range(500):

		params['x0'] = np.array(np.random.uniform(-1,1,4))
		params['u0'] = np.array(np.random.uniform(-2,2,2))

		energy = np.zeros((int(10.0/params['Ts']),1))
		xk = params['x0']
		uk = params['u0']
		xHistory = [xk.tolist()]

		for i in range(int(10.0/params['Ts'])):
			xk = IntegrationEstimation(xk, uk, params['Ts'], 30)

			KE = 0.5*params['m1']*(params['l1']**2)/4*xk[2]**2 + params['m2']*(0.5*params['l1']**2*xk[2]**2 + 0.5*(params['l2']**2)/4*(xk[2] + xk[3])**2 + params['l1']*params['l2']/2*xk[2]*(xk[2] + xk[3]))*np.cos(xk[1])+ 0.5*params['I1']*xk[2]**2 + 0.5*params['I2']*xk[3]**2

			PE = params['m1']*params['l1']/2*params['g']*(np.cos(xk[0])) + params['m2']*params['g']*(params['l1']*np.cos(xk[0])+ params['l2']/2*np.cos(xk[0] + xk[1]))

			energy[i] = KE + PE
			save.append((xk[:].tolist(),uk[:].tolist()))


		print(len(save))

	pickle.dump(save, f)
			# xHistory += [xk[:].tolist()]


# with open(PIK, "rb") as f:
#     print (pickle.load(f)[9000])
