import math
import numpy as np
from collections import namedtuple
from numpy.linalg import inv
import sys
sys.path.insert(0, './..')
from robot import PlanarRR

import kinematics as KMTCS

def differentiate(vector,dt):
	diff_vec = np.zeros(vector.shape)
	diff_vec[0] = (vector[0]-vector[1])/dt
	diff_vec[-1] = (vector[-2]-vector[-1])/dt
	for i in range(1,vector.size-1):
		diff_vec[i] = (vector[i+1]-vector[i-1])/(2*dt)

	return diff_vec

def minJerkSetup(x0,t0,tf):
	A = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
					[0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
					[0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
					[1, tf, tf**2, tf**3, tf**4, tf**5],
					[0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
					[0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])
	constants = np.dot(inv(A),x0)
	return constants

def minJerkStep(t,constants):
	pos = np.dot(np.array([1, t, t**2, t**3, t**4, t**5]),constants)
	vel = np.dot(np.array([0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4]),constants)
	accel = np.dot(np.array([0, 0, 2, 6*t, 12*t**2, 20*t**3]),constants)
	return pos,vel,accel

def minJerkSetup_now(initial_angles,tf,waypoints,t_array=None):
	num_waypoints = waypoints.shape[1]
	try:
		if t_array == None:
			del_t = float(tf)/float(num_waypoints)
			t_array = del_t*np.ones((num_waypoints,1))
		elif not t_array.size == num_waypoints:
			raise ValueError('Time array length is incorrect')
		elif not tf == np.sum(t_array):
			raise ValueError('Time array must add up to final time')
	except:
		if not t_array.size == num_waypoints:
			raise ValueError('Time array length is incorrect')
		elif not tf == np.sum(t_array):
			raise ValueError('Time array must add up to final time')

	joint_constants = namedtuple('joint_constants','J1 J2')
	joint_const = joint_constants(np.zeros((6,num_waypoints)),
									np.zeros((6,num_waypoints)))
	
	x0 = np.zeros((2,6)) 
	# x0: 5 Joints and 6 values {q_initial, dq_initial, ddq_initial, q_final, dq_final, ddq_final}
	# dq and ddq are assumed to zeros at each waypoints
	# q_initial and q_final are changed in the for-loop below

	if initial_angles.ndim == 2:
		if initial_angles.shape[0] == 2:
			initial_angles = initial_angles.T
	x0[:,0] = initial_angles
	x0[:,3] = PlanarRR.inverseKinematics(waypoints[0,0],
										waypoints[1,0])

	t0 = np.zeros((num_waypoints,1))
	tf = np.zeros((num_waypoints,1))
	tf[0] = t_array[0]

	for i in range(num_waypoints):
		if i > 0:
			x0[:,0] = x0[:,3]
			x0[:,3] = PlanarRR.inverseKinematics(waypoints[0,i],
												waypoints[1,i]).T
			t0[i] = tf[i-1]
			tf[i] = t0[i]+t_array[i]
			
		joint_const.J1[:,i] = minJerkSetup(x0[0],t0[i],t0[i]+t_array[i])
		joint_const.J2[:,i] = minJerkSetup(x0[1],t0[i],t0[i]+t_array[i])
	
	return joint_const

def minJerkStep_now(time,tf,waypoints,joint_const,t_array=None):
	num_waypoints = waypoints.shape[1]
	try:
		if t_array == None:
			del_t = float(tf)/float(num_waypoints)
			t_array = del_t*np.ones((num_waypoints,1))
			k = int(np.floor(time/(del_t)))
		else:
			sum_time = 0.
			k = 0
			for i in range(t_array.size-1):
				sum_time = sum_time + t_array[i]
				if time < sum_time:
					break
				k = k+1
	except:
		sum_time = 0.
		k = 0
		for i in range(t_array.size-1):
			sum_time = sum_time + t_array[i]
			if time < sum_time:
				break
			k = k+1
	# print(time, k)
	if not t_array.size == num_waypoints:
		raise ValueError('Time array is length is incorrect')
	if not tf == np.sum(t_array):
		raise ValueError('Time array must add up to final time')
	pos = np.zeros((2,1))
	vel = np.zeros((2,1))
	accel = np.zeros((2,1))

	for j in range(2):
		pos[j],vel[j],accel[j] = minJerkStep(time,joint_const[j][:,k])

	return pos,vel,accel