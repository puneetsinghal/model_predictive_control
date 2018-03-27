
from scipy.optimize import minimize
import numdifftools as nd
import numpy as np
from math import *
from copy import copy
from IPython import embed

class MPC(object):
	"""docstring for MPC"""
	def __init__(self, robot, params, bnds):
		self.Q = params['Q']
		self.R = params['R']
		self.robot = robot
		self.Ts = params['Ts']
		self.N = params['N']
		self.bnds = bnds
		self.x = params['x0']
		self.u0 = params['u0']
		self.xref = params['x0']
		self.cons = ({'type': 'ineq', 'fun': self.Contraints, 'args':()})
		# self.jac = ({'type': 'ineq', 'fun': self.Jacobian, 'args':()})

	def IntegrationEstimation(self, xk, uk, M = 5):
		# Runge-Kutta 4th order (M = 5 optimization problem, M = 30 updating state space)
		# Better ODE solvers can be used here 
		delta = self.Ts/M
		xk1 = xk
		for ct in range(M):
			k1 = self.robot.dynamics(xk1, uk)
			k2 = self.robot.dynamics(xk1 + k1*delta/2, uk)
			k3 = self.robot.dynamics(xk1 + k2*delta/2, uk)
			k4 = self.robot.dynamics(xk1 + k3*delta, uk)
			xk1 = xk1 + delta*(k1/6 + k2/3 + k3/3 + k4/6)
		return xk1

	def CostFunction(self, u):
	    # Cost function taken from MatLab's nMPC example code 
		J = 0.
		xk = self.x

		for ct in range(len(u)):
			uk = u[ct]

			xk1 = self.IntegrationEstimation(xk, uk);

			# for i in range(len(xk1)):
			J += np.matmul(np.matmul((xk1-self.xref).T,self.Q),(xk1-self.xref))

			if ct==0:
				J += (uk-self.u0)*self.R*(uk-self.u0)
			else:
				J += (uk-u[ct-1])*self.R*(uk-u[ct-1])

			xk = copy(xk1)
		return J

	def Contraints(self, u):
		phiMin = -10
		phiMax = 10
		xk = self.x
		c = np.zeros(self.N*2)

		for ct in range(len(u)):
			uk = u[ct]

			xk1 = self.IntegrationEstimation(xk, uk, 5)

			# -phi + phiMin > 0
			c[2*ct] = xk1[0]-phiMin
			# phi - phiMax > 0
			c[2*ct+1] = -xk1[0]+phiMax

			#update plant state and input for next step
			xk = xk1
		return c

	def Jacobian(self, u):
		gradient = np.zeros(self.N)
		# embed()
		delta = 0.001
		c0 = self.CostFunction(u)
		# print("entered")
		for i in range(self.N):
			y = copy(u)
			# embed()
			y[i] += delta
			cPlus = self.CostFunction(y)
			y[i] -= 2.*delta
			cMinus = self.CostFunction(y)
			
			gradient[i] = (cPlus - cMinus)/(2.*delta)
			# embed()

		# y = nd.Jacobian(self.CostFunction)
		# print(y(u))
		# embed()
		return gradient

	def optimize(self, uopt, x, u0, xref):
		self.x = x
		self.u0 = u0
		self.xref = xref
		return minimize(self.CostFunction, uopt, args=(), method='SLSQP',jac=self.Jacobian, bounds=self.bnds, constraints=self.cons)