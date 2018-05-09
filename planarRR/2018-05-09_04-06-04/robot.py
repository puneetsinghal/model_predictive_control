import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import sin, cos, acos, atan2, pi

class Acrobot(object):
	"""docstring for ClassName"""
	def __init__(self, params):
		self.m1 = params['m1']
		self.m2 = params['m2']
		self.l1 = params['l1']
		self.l2 = params['l2']
		self.g = params['g']
		self.I1 = params['I1']
		self.I2 = params['I2']
		self.N = params['N']
		self.Ts = params['Ts']

	def dynamics(self, x, u):

		theta_1 = x[0]
		theta_2 = x[1]
		dtheta_1 = x[2]
		dtheta_2 = x[3]

		sine_2 = np.sin(theta_2)
		cosine_2 = np.cos(theta_2)

		try:
			M = np.zeros((2,2))

			M[0,0] = self.m1*(self.l1/2)**2 + self.m2*(self.l1**2 + (self.l2/2)**2 + 2*self.l1*(self.l2/2)*cosine_2) + self.I1 + self.I2
			M[0,1] = self.m2*((self.l2/2)**2 + self.l1*(self.l2/2)*cosine_2) + self.I2
			M[1,0] = M[0,1]
			M[1,1] = self.m2*(self.l2/2)**2 + self.I2

			# M[0,0] = self.m2*(self.l1**2 + 2*self.l1*(self.l2/2)*cosine_2) + self.I1 + self.I2
			# M[0,1] = self.m2*(self.l1*(self.l2/2)*cosine_2) + self.I2
			# M[1,0] = M[0,1]
			# M[1,1] = self.I2
			Cor = np.zeros((2,2))
			Cor[0,0] = -2*self.m2* self.l1*(self.l2/2)*sine_2*dtheta_2
			Cor[0,1] = -self.m2*self.l1*(self.l2/2)*sine_2*dtheta_2
			Cor[1,0] = self.m2*self.l1*(self.l2/2)*sine_2*dtheta_1

			G_vector = np.zeros((2,1))
			G_vector[0,0] = (self.m1*(self.l1/2) + self.m2*self.l1)*self.g*np.cos(np.pi/2+theta_1) + self.m2* (self.l2/2)*self.g*np.cos(np.pi/2+theta_1 + theta_2)
			G_vector[1,0] = self.m2*(self.l2/2)*self.g*np.cos(np.pi/2 + theta_1 + theta_2)

			B = np.array([0, 1]).reshape((2,1))
			# B = np.identity(2)
			xk1 = np.zeros(4)
			xk1[0] = dtheta_1
			xk1[1] = dtheta_2
			# print(np.linalg.pinv(M))
			# print(B*u)
			# print(np.matmul(Cor,np.array([dtheta_1, dtheta_2]).reshape((2,1))))
			# print(G)
			# print((B*u - np.matmul(Cor, np.array([dtheta_1, dtheta_2]).reshape((2,1))) - G))
			# print(np.matmul(np.linalg.pinv(M),(B*u - np.matmul(Cor, np.array([dtheta_1, dtheta_2])))))
			temp = np.matmul(np.linalg.pinv(M), (B*u - np.matmul(Cor,np.array([dtheta_1, dtheta_2]).reshape((2,1))) - G_vector))
			xk1[2:4] = temp[:,0]
		except:
			# pass
			print(x)
			
		return xk1

	def kinematics(self, z):
		q1 = z[0]
		q2 = z[1]
		dq1 = z[2]
		dq2 = z[3]

		p1 = self.l1*np.array([np.sin(q1),-np.cos(q1)])
		p2 = p1 + self.l2*np.array([np.sin(q1+q2), -np.cos(q1+q2)])
		return p1, p2

	def animate(self, z):
		# length = p.l1+p.l2;
		# axis equal; axis(length*[-1,1,-1,1]); axis off;
		
		x1 = self.l1*np.sin(z[:,0])
		y1 = -self.l1*np.cos(z[:,0])

		x2 = self.l2*np.sin(z[:,0] + z[:,1]) + x1
		y2 = -self.l2*np.cos(z[:,0] + z[:,1]) + y1

		fig = plt.figure()

		ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-1.5, 1.5))
		ax.grid()
		
		line, = ax.plot([], [], 'o-', lw=2)
		time_template = 'time = %.1fs'
		time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

		def init():
			line.set_data([], [])
			time_text.set_text('')
			return line, time_text

		def animate(i):
			thisx = [0, x1[i], x2[i]]
			thisy = [0, y1[i], y2[i]]

			line.set_data(thisx, thisy)
			time_text.set_text(time_template%(i*self.Ts))
			return line, time_text

		ani = animation.FuncAnimation(fig, animate, np.arange(1, len(z)), interval=100, blit=True, init_func=init)

		# ani.save('double_pendulum.mp4', fps=15)
		plt.show()


class PlanarRR(object):
	def __init__(self, params):
		self.m1 	= params['m1']
		self.m2 	= params['m2']
		self.l1 	= params['l1']
		self.l2 	= params['l2']
		self.g 		= 0*params['g']
		self.I1 	= params['I1']
		self.I2 	= params['I2']

	def dynamics(self, x, u):

		theta_1, theta_2, dtheta_1, dtheta_2 = x
		# embed()
		sine_2 = sin(theta_2)
		cosine_2 = cos(theta_2)

		M = np.zeros((2,2))
		M[0,0] = self.m1*(self.l1/2)**2 + self.m2*(self.l1**2 + (self.l2/2)**2 + 2*self.l1*(self.l2/2)*cosine_2) + self.I1 + self.I2
		M[0,1] = self.m2*((self.l2/2)**2 + self.l1*(self.l2/2)*cosine_2) + self.I2
		M[1,0] = M[0,1]
		M[1,1] = self.m2*(self.l2/2)**2 + self.I2

		Cor = np.zeros((2,2))
		Cor[0,0] = -2*self.m2* self.l1*(self.l2/2)*sine_2*dtheta_2
		Cor[0,1] = -self.m2*self.l1*(self.l2/2)*sine_2*dtheta_2
		Cor[1,0] = self.m2*self.l1*(self.l2/2)*sine_2*dtheta_1

		G_vector = np.zeros((2,1))
		G_vector[0,0] = (self.m1*(self.l1/2) + self.m2*self.l1)*self.g*np.cos(np.pi/2+theta_1) + self.m2* (self.l2/2)*self.g*np.cos(np.pi/2+theta_1 + theta_2)
		G_vector[1,0] = self.m2*(self.l2/2)*self.g*np.cos(np.pi/2 + theta_1 + theta_2)

		# B = np.array([0, 1]).reshape((2,1))
		B = np.identity(2)
		M_inv = np.linalg.pinv(M)

		f = np.array([dtheta_1, dtheta_2])
		f = np.hstack((f, np.matmul(M_inv, - np.matmul(Cor,np.array([dtheta_1, dtheta_2]).reshape((2,1))) - G_vector).T[0]))

		# embed()
		g = np.zeros([2, 2])
		g = np.hstack((g, np.matmul(M_inv, B)))

		xdot = f + np.matmul(g.T, u)
		return xdot

	@staticmethod
	def inverseKinematics(x, y, l1=1., l2=1.):

		c2 = (x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2)

		theta_2 = acos(c2)

		alpha = atan2(l2*sin(theta_2), (x**2 + y**2 + l1**2 - l2**2)/(2*l1))

		theta_1 = atan2(y, x) - alpha

		return np.array([theta_1, theta_2])
	
	@staticmethod
	def forwardKinematics(z, l1=1., l2=1.):
		q1 = z[0]
		q2 = z[1]

		p1 = l1*np.array([cos(q1), sin(q1)])
		p2 = p1 + l2*np.array([cos(q1+q2), sin(q1+q2)])
		return p1, p2

	def animate(self, z):
		# length = p.l1+p.l2;
		# axis equal; axis(length*[-1,1,-1,1]); axis off;
		
		x1 = self.l1*np.cos(z[:,0])
		y1 = self.l1*np.sin(z[:,0])

		x2 = self.l2*np.cos(z[:,0] + z[:,1]) + x1
		y2 = self.l2*np.sin(z[:,0] + z[:,1]) + y1

		fig = plt.figure()

		ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-1.5, 1.5))
		ax.grid()
		
		line, = ax.plot([], [], 'o-', lw=2)
		time_template = 'time = %.1fs'
		time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

		def init():
			line.set_data([], [])
			time_text.set_text('')
			return line, time_text

		def animate(i):
			thisx = [0, x1[i], x2[i]]
			thisy = [0, y1[i], y2[i]]

			line.set_data(thisx, thisy)
			time_text.set_text(time_template%(i*self.Ts))
			return line, time_text

		ani = animation.FuncAnimation(fig, animate, np.arange(1, len(z)), interval=100, blit=True, init_func=init)

		# ani.save('double_pendulum.mp4', fps=15)
		plt.show()

	# def generateSystemData(self, control_generator):
	# 	T= self.T
	# 	M = self.M

	# 	U = np.zeros([T*M, self.numInputs])
	# 	W = np.zeros([T*M, self.disturbanceSize])
	# 	X = np.zeros([T*M, self.DIM])
	# 	x = np.zeros([T, self.DIM])
	# 	# embed()

	# 	SM = np.random.randint(0, 101, [self.numInputs, M])
	# 	for k in range(M):

	# 		# x0 = np.random.uniform(-1, 1, self.DIM)
	# 		x0 = np.zeros(self.DIM)
	# 		j = SM[:,k]
			
	# 		u      = 0.5 *np.vstack((control_generator[:,j[0]], control_generator[:,j[1]]))
	# 		w      = 0.01 * np.random.uniform(-1, 1, [self.disturbanceSize, T])
	# 		x[0,:] = x0

	# 		r = SCI_INT.ode(self.dynamicsTrain).set_integrator("dopri5") 
	# 		r.set_initial_value(x0, 0)
	# 		for i in range(1, T):
	# 			# embed()
	# 			x[i, :] = r.integrate(r.t+self.dt)    # get one more value, add it to the array
	# 			if not r.successful():
	# 				raise RuntimeError("Could not integrate")
	# 		# embed()
	# 		U[k*T:(k+1)*T,:] = u.T
	# 		W[k*T:(k+1)*T,:] = w.T
	# 		X[k*T:(k+1)*T,:] = x
	# 		print(k)
		
	# 	# angles, velocities = np.hsplit(X, 2)
	# 	# sines = np.sin(angles)
	# 	# cosines = np.cos(angles)
	# 	# angles = np.arctan2(sines, cosines)
	# 	# X = np.hstack((angles, velocities))
		
	# 	file = h5py.File('PlanarRR_systemData3.h5', 'w') 
	# 	file.create_dataset('U', data=U)
	# 	file.create_dataset('W', data=W)
	# 	file.create_dataset('X', data=X)
	# 	file.close()

	# 	print("Data generation completed")
	# 	return U, W, X

	# def findControl(self, xHistory):

	# 	# angles, velocities = np.hsplit(xHistory, 2)
	# 	# sines = np.sin(angles)
	# 	# cosines = np.cos(angles)
	# 	# angles = np.arctan2(sines, cosines)
	# 	# xHistory = np.hstack((angles, velocities))
	# 	# input("enter: ")
	# 	uHistory = np.zeros([xHistory.shape[0], self.numInputs])
	# 	for i in range(xHistory.shape[0]):
	# 		x = xHistory[i,:]
	# 		g = self.g_function(x).T
	# 		feed_dict={self.nn.X_t:x.reshape(1,self.DIM), self.nn.X_tPlus:x.reshape(1,self.DIM), self.nn.dropout_prob:1.0}
	# 		grad = self.sess.run(self.nn.value_grad_t,feed_dict=feed_dict)
	# 		# ut = -0.5*np.matmul(g.reshape(self.numInputs,self.DIM), grad.reshape(self.DIM,1))[:,0]
	# 		ut   = -0.5*np.matmul(g, grad.reshape(self.DIM,1)).reshape(self.numInputs,)
	# 		# print(ut)
	# 		uHistory[i, :] = ut
	# 	# embed()
	# 	return uHistory