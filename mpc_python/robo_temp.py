import numpy as np
#import matplotlib.pyplot as plt
#import matplotlib.animation as animation

#import sys
#import rospy
#from arm_feedback.srv import *


class  KdcArm(object):
	"""docstring for  KdcArm"""
	def __init__(self, params):
		pass
	
	def dynamics(self, x, u):
		return np.zeros(10)

	