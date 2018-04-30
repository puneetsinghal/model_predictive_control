
from scipy.optimize import minimize
#import numdifftools as nd
import numpy as np
from math import *
from copy import copy
from IPython import embed

class MPC(object):
	"""docstring for MPC"""
	def __init__(self, robot, params, bnds, method = 'waypoints'):
		self.Q = params['Q']
		self.R = params['R']
		self.robot = robot
		self.Ts = params['Ts']
		self.N = params['N']
		self.startTime = 0.
		self.Duration = params['Duration']
		self.bnds = bnds
		self.x = params['x0']
		self.u0 = params['u0']
		self.xref = params['x0']
		self.cons = ({'type': 'ineq', 'fun': self.Contraints, 'args':()})
		self.waypoints = params['waypoints']
		self.currentTime = 0.
		self.method = method
		self.xRefHistory = []

		# expanding trajectory by (N-1) points with final state.
		# This is to generate reference values for last iteration without using if condition (possibly will save time)
		if(method=='trajectory'):
			self.numPoints = (params['trajectory']).shape[1]
			trajectoryLength = self.numPoints+self.N-1
			self.trajectory = np.zeros((10,trajectoryLength))  # Changed this !
			self.trajectory[:,0:self.numPoints] = params['trajectory']
			xf = self.trajectory[:,self.numPoints-1]
			for i in range(self.N-1):
				self.trajectory[:,self.numPoints+i] = xf
		else:            
			self.numSegment = (self.waypoints).shape[1] - 1 # subtracting one to get segments - HARDCODED
			#self.numSegment =  8    
		# self.setupFigure()
		# self.jac = ({'type': 'ineq', 'fun': self.Jacobian, 'args':()})

	# def setupFigure(self):
	# 	self.fig = plt.figure()
	# 	self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-1.5, 1.5))
	# 	self.ax.grid()
	# 	self.line, = self.ax.plot([], [], 'o-', lw=2)

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

	def CostFunctionGoal(self, u):
	    # Cost function taken from MatLab's nMPC example code 
		J = 0.
		xk = self.x
		print(u)
		for ct in range(len(u)):
			uk = u[ct]

			xk1 = self.IntegrationEstimation(xk, uk);

			# for i in range(len(xk1)):
			J += np.matmul(np.matmul((xk1-self.xref),self.Q),(xk1-self.xref))
			if ct==0:
				J += (uk-self.u0)*self.R*(uk-self.u0)                
			else:
				J += (uk-u[ct-1])*self.R*(uk-u[ct-1])

			xk = copy(xk1)
            
		return J[0,0]

	def CostFunctionTrajectory(self, u):
	    # Cost function taken from MatLab's nMPC example code 
		J = 0.
		xk = self.x
		trajIndex = int((self.currentTime - self.startTime)/self.Ts)
		for ct in range(len(u)):
			uk = u[ct]

			xk1 = self.IntegrationEstimation(xk, uk);

			# for i in range(len(xk1)):
			J += np.matmul(np.matmul((xk1-self.trajectory[:,trajIndex]),self.Q),(xk1-self.trajectory[:,trajIndex]))

			if ct==0:
				J += (uk-self.u0)*self.R*(uk-self.u0)
			else:
				J += (uk-u[ct-1])*self.R*(uk-u[ct-1])

			xk = copy(xk1)
			# if(trajIndex<self.numPoints):
			trajIndex += 1
		return J

	def Contraints(self, u):
		phiMin = -10
		phiMax = 10
		xk = copy(self.x)
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
		c0 = self.CostFunctionGoal(u)
		# print("entered")
		for i in range(self.N):
			y = copy(u)
			# embed()
			y[i] += delta
			cPlus = self.CostFunctionGoal(y)
			y[i] -= 2.*delta
			cMinus = self.CostFunctionGoal(y)
			
			gradient[i] = (cPlus - cMinus)/(2.*delta)
		return gradient

	# def Jacobian(self, u):
	# 	xk = np.zeros((self.N, 4))
	# 	zk = np.zeros((self.N, 4))
	# 	gradient = np.zeros(len(u))
	# 	xk[0,:] = copy(self.x)
	# 	zk[0,:] = copy(self.x)

	# 	for k in range(0,self.N-1):		
	# 		xk[k+1,:] = self.robot.dynamics(xk[k,:],u[k])
	# 		zk[k,:] = xk[k+1,:]

	# 	lamda = 1000*np.matmul(self.Q, xk[self.N-1,:])
	# 	for k in range(self.N-1,-1,-1):
	# 		A,B,C = self.linear(xk[k,:],u[k])
	# 		# embed()
	# 		# print((np.dot(B,lamda)))
	# 		gradient[k] = self.R*u[k] + np.dot(B,lamda)
	# 		lamda = np.matmul(np.matmul(C.T,self.Q),zk[k,:].T) + np.matmul(A.T,lamda)
	# 	print(1000.*gradient)
	# 	return (1000.*gradient)

	def linear(self,x0,u0):
		#Simple Linerization:
		delta = 0.001
		c0 = self.robot.dynamics(x0,u0)
		A = np.zeros((10,10))
		B = np.zeros((10,5))
		C = np.identity(5)
		a = copy(x0)
		b = copy(u0)

		for i in range(10):
			a = copy(x0)
			a[i] = a[i] + delta
			ca = self.robot.dynamics(a,u0)
			# embed()
			A[:,i] = (ca - c0)/delta

		b += delta
		cb = self.robot.dynamics(x0,b)
		B = ((cb-c0)/delta)

		return A, B, C

	def optimize(self, uopt, x, u0):
		self.x = copy(x)
		self.u0 = u0
		if(self.method=='waypoints'):
			# fetch the xref from waypoints
			waypointIndex = min(int(self.numSegment*self.currentTime/self.Duration) + 1, self.numSegment)          
			self.xref = copy(self.waypoints[:,waypointIndex])
			self.xRefHistory += [(self.xref).tolist()]
			print("reference value: {}".format(self.xref.T))                    
			results = minimize(self.CostFunctionGoal, uopt, args=(), method='SLSQP', jac=self.Jacobian, bounds=self.bnds, constraints=self.cons)
		else:
			trajIndex = min(int((self.currentTime - self.startTime)/self.Ts), self.numPoints-1)
			self.xref = copy(self.trajectory[:,trajIndex])
			self.xRefHistory += [(self.xref).tolist()]
			print("reference value: {}".format(self.xref.T))  
			results = minimize(self.CostFunctionTrajectory, uopt, args=(), method='SLSQP', bounds=self.bnds, constraints=self.cons)
		
		self.currentTime += self.Ts
		return results
	# def animate(self, z):
	# 	# length = p.l1+p.l2;
	# 	# axis equal; axis(length*[-1,1,-1,1]); axis off;

		
	# 	time_template = 'time = %.1fs'
	# 	time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

	# 	def init():
	# 		line.set_data([], [])
	# 		time_text.set_text('')
	# 		return line, time_text

	# 	def animate(i):
	# 		thisx = [0, x1[i], x2[i]]
	# 		thisy = [0, y1[i], y2[i]]

	# 		line.set_data(thisx, thisy)
	# 		time_text.set_text(time_template%(i*self.Ts))
	# 		return line, time_text

	# 	ani = animation.FuncAnimation(fig, animate, np.arange(1, len(z)), interval=100, blit=True, init_func=init)

	# 	# ani.save('double_pendulum.mp4', fps=15)
	# 	plt.show()