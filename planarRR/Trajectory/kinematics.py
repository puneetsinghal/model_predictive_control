from math import sin, cos, acos, atan2
import numpy as np
from collections import namedtuple
from numpy.linalg import inv


_OFFSET_ = np.array([[0.,	 0.,	0.,		0.,		0.,     0.],
				   [0,-0.006, 0.031, -0.031, -0.038,    0.],
				   [0,  0.07,  0.28,   0.28,  0.055, 0.032]])

_L1_ = 0.28
_L2_ = 0.28

def inverseKinematics(x, y, l1, l2):
	
	c2 = (x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2)

	theta_2 = acos(c2)

	alpha = atan2(l2*sin(theta_2), (x**2 + y**2 - l1**2)/(2*l1))

	theta_1 = atan2(y, x) - alpha

	return np.array([theta_1, theta_2])
