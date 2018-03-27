import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


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

		theta_1 = x[0][0]
		theta_2 = x[1][0]
		dtheta_1 = x[2][0]
		dtheta_2 = x[3][0]

		try:
			M = np.zeros((2,2))

			M[0,0] = self.m1*(self.l1/2)**2 + self.m2*(self.l1**2 + (self.l2/2)**2 + 2*self.l1*(self.l2/2)*np.cos(theta_2)) + self.I1 + self.I2
			M[0,1] = self.m2*((self.l2/2)**2 + self.l1*(self.l2/2)*np.cos(theta_2)) + self.I2
			M[1,0] = M[0,1]
			M[1,1] = self.m2*(self.l2/2)**2 + self.I2

			Cor = np.zeros((2,2))
			Cor[0,0] = -2*self.m2* self.l1*(self.l2/2)*np.sin(theta_2)*dtheta_2
			Cor[0,1] = -self.m2*self.l1*(self.l2/2)*np.sin(theta_2)*dtheta_2
			Cor[1,0] = self.m2*self.l1*(self.l2/2)*np.sin(theta_2)*dtheta_1

			G_vector = np.zeros((2,1))
			G_vector[0,0] = (self.m1*(self.l1/2) + self.m2*self.l1)*self.g*np.cos(np.pi/2+theta_1) + self.m2* (self.l2/2)*self.g*np.cos(np.pi/2+theta_1 + theta_2)
			G_vector[1,0] = self.m2*(self.l2/2)*self.g*np.cos(np.pi/2 + theta_1 + theta_2)

			B = np.array([0, 1]).reshape((2,1))

			xk1 = np.zeros((4,1))
			xk1[0,0] = dtheta_1
			xk1[1,0] = dtheta_2
			# print(np.linalg.pinv(M))
			# print(B*u)
			# print(np.matmul(Cor,np.array([dtheta_1, dtheta_2]).reshape((2,1))))
			# print(G)
			# print((B*u - np.matmul(Cor, np.array([dtheta_1, dtheta_2]).reshape((2,1))) - G))
			# print(np.matmul(np.linalg.pinv(M),(B*u - np.matmul(Cor, np.array([dtheta_1, dtheta_2])))))
			xk1[2:4,:] = np.matmul(np.linalg.pinv(M), (B*u - np.matmul(Cor,np.array([dtheta_1, dtheta_2]).reshape((2,1))) - G_vector))
		except:
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

		ani.save('double_pendulum.mp4', fps=15)
		plt.show()


class Furuta(object):
	"""docstring for Furuta"""
	def __init__(self, params):
		pass
	
	def dynamics(self, x, u):
    
	    # m_cart = 1.0   # cart mass
	    # m_pend = 1.0   # pendulum mass
	    # g = 9.81       # gravity of earth
	    # l = 1          # pendulum length
	    # r = 0.5        # length of actuator

	    # phi = x[0]
	    # dphi = x[1]
	    # theta = x[2]
	    # dtheta = x[3]

	    # dxdt = np.zeros(4)

	    # dxdt[0] = dphi
	    # dxdt[1] = (2*F*l + g*l*m_pend*r*sin(2*theta) - 2*dphi*dtheta*l**3*m_pend*sin(2*theta) - 2*dtheta**2*l**2*m_pend*r*sin(theta) + 2*dphi**2*m_pend*r**3*sin(theta)*(sin(theta)**2 - 1) + 2*dphi*dtheta*l*m_pend*r**2*sin(2*theta) - 2*dphi**2*l**2*m_pend*r*sin(theta)*(sin(theta)**2 - 1))/(2*l**3*m_pend + 2*l*m_cart*r**2 - 2*l**3*m_pend*cos(theta)**2)
	    # dxdt[2] = dtheta
	    # dxdt[3] = (2*F*l*r*cos(theta) + dphi**2*l**4*m_pend*sin(2*theta) - dphi**2*m_cart*r**4*sin(2*theta) + 2*g*l**3*m_pend*sin(theta) - 2*g*l**3*m_pend*(sin(theta) - sin(theta)**3) + 2*g*l*m_cart*r**2*sin(theta) + 2*g*l*m_pend*r**2*(sin(theta) - sin(theta)**3) + dphi**2*l**2*m_cart*r**2*sin(2*theta) - dphi**2*l**2*m_pend*r**2*sin(2*theta) - dtheta**2*l**2*m_pend*r**2*sin(2*theta) - 2*dphi**2*l**4*m_pend*cos(theta)**3*sin(theta) - 2*dphi**2*m_pend*r**4*cos(theta)**3*sin(theta) + 4*dphi**2*l**2*m_pend*r**2*cos(theta)**3*sin(theta) - 4*dphi*dtheta*l*m_pend*r**3*sin(theta)*(sin(theta)**2 - 1) + 4*dphi*dtheta*l**3*m_pend*r*sin(theta)*(sin(theta)**2 - 1))/(2*l**4*m_pend + 2*l**2*m_cart*r**2 - 2*l**4*m_pend*cos(theta)**2)

	    # return dxdt
	    pass
	
