import math
import numpy as np
from collections import namedtuple
from numpy.linalg import inv


_OFFSET_ = np.array([[0.,	 0.,	0.,		0.,		0.,     0.],
				   [0,-0.006, 0.031, -0.031, -0.038,    0.],
				   [0,  0.07,  0.28,   0.28,  0.055, 0.032]])

_L1_ = 0.28
_L2_ = 0.28

def inverseKinematics(x, y, z, theta): # end effector is always pointing down
	if z < -0.03:
		raise ValueError('Point is inside of the table')

	angles = np.zeros((5,1))

	combined_x_offset = 0.044

	tool_z_offset = 0.032
	module5_z_offset = 0.055;
	module1_z_offset = 0.;
	module2_z_offset = 0.07;

	z_mod = (-module2_z_offset - module1_z_offset + z + tool_z_offset
		 + module5_z_offset)

	r = np.sqrt(x**2+y**2+z_mod**2)
	if r > 0.572*0.95:
		raise ValueError('Point is outside of the reachable workspace')

	r1 = np.sqrt(x**2+y**2)
	if r1 > combined_x_offset:
		d = np.sqrt(r1**2 - combined_x_offset**2)
	else:
		raise ValueError('Radius of arm is too close to the base')

	angles[0] = (np.pi/2 + np.arccos((d**2 - combined_x_offset**2 + r1**2)
				/ (2*d*r1))- np.arcsin(x/r1))

	r2 = np.sqrt(z_mod**2 + d**2)

	angles[1] = -(-np.arcsin(z_mod/r2) + np.pi/2
				 - np.arccos((_L1_**2-_L2_**2+r2**2)/(2*_L1_*r2)))
	angles[2] = np.pi-np.arccos((_L1_**2+_L2_**2-r2**2)/(2*_L1_*_L2_))

	angles[3] = -(np.pi + angles[1] - angles[2])
	angles[4] = -(np.pi/2 - angles[0]) + theta

	return angles

def forwardKinematics(angles):
	Transform = Tz(angles[0],_OFFSET_[:,0])
	Transform = np.dot(Transform,Ty(-angles[1],_OFFSET_[:,1]))
	Transform = np.dot(Transform,Ty(angles[2],_OFFSET_[:,2]))
	Transform = np.dot(Transform,Ty(-angles[3],_OFFSET_[:,3]))
	Transform = np.dot(Transform,Tz(angles[4],_OFFSET_[:,4]))
	return np.around(np.dot(Transform,T(_OFFSET_[:,5])),decimals=5)

def Ty(theta, trans):
	return np.array([[math.cos(theta),  0., math.sin(theta), 	trans[0]],
					 [		0.,  	  1., 		0.,			trans[1]],
					 [-math.sin(theta), 0., math.cos(theta), 	trans[2]],
					 [		0., 	  0., 		0., 		 	  1]])
def Tz(theta,trans):
	return np.array([[math.cos(theta), -math.sin(theta), 0., trans[0]],
					 [math.sin(theta),  math.cos(theta), 0., trans[1]],
					 [ 		0., 				0.,  1., trans[2]],
					 [		0., 				0.,  0., 	   1.]])
def T(trans):
	return np.array([[1., 0.,  0., trans[0]],
					 [0., 1.,  0., trans[1]],
					 [0., 0.,  1., trans[2]],
					 [0., 0.,  0., 	   1.]])
