from scipy.integrate import ode
from robot import Acrobot
import numpy as np

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



if __name__ == '__main__':
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

	robot = Acrobot(params)

	# results = ode45(@(t,x)acrobotDynamicsCT(t, x, u, params), linspace(0,4,4/params.Ts), x0);

	# N = size(results.x,2);
	energy = np.zeros((int(10.0/params['Ts']),1))
	xk = params['x0']
	uk = 0
	xHistory = [xk[:,0].tolist()]

	for i in range(int(10.0/params['Ts'])):
		xk = IntegrationEstimation(xk, uk, params['Ts'], 30)
		energy[i] = Energy(params, xk)
		xHistory += [xk[:,0].tolist()]
        robot.animate(np.array(xHistory))




