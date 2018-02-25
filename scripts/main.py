
#Standard Python Libraries
import sys
import csv
import math
from datetime import datetime
import time
import argparse
from collections import namedtuple

#Third Party Software
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import GPy

import os

#Python Script
import TrajectoryGenerartor as TG
import kinematics as KMTCS

if __name__ == '__main__':
    #Parsing inputs for plotting, trajectory generation, and saving options
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--plot", type=str, default="none")
    parser.add_argument("-t", "--traj", type=str, default="pnp")
    parser.add_argument("-m", "--mode", type=str, default="minjerk")
    args = parser.parse_args()
    plotting = args.plot.lower()
    traj_type = args.traj.lower()
    traj_mode = args.mode.lower()
    #Creating a start position namedtuple
    position = namedtuple('Position', ['c_x', 'c_y','c_z','theta'])

    #Setup control struct
    control_info = {}
    if traj_type == "circle":
        control_info['type'] = "Circle"
        position.c_x = 0.0
        position.c_y = 0.395
        position.c_z = 0.0
        position.theta = 0
        control_info['tf'] = 10.
    else:
        if 	traj_mode == "minjerk":
            control_info['type'] = "MinJerk"
            position.c_x = 0.0
            position.c_y = 0.395
            position.c_z = 0.1
            position.theta = 0
            control_info['tf'] = 16.
        else:
            control_info['type'] = "Cubic"
            position.c_x = 0.0
            position.c_y = 0.395
            position.c_z = 0.1
            position.theta = 0
            control_info['tf'] = 16.

    ###### Training the Model ######

    start_time = time.clock()
    time_from_start = 0.
    final_time=control_info['tf']

    #MinJerk trajectory type specifies waypoints to move to with zero velocity and accelration
    #boundary conditions
    if traj_type == "pnp":
        waypoints = np.array([[-0.02, -0.02, -0.02, 0.365,  0.365, 0.365,   0.0,   0.0],
            [ 0.49,  0.49,  0.49, 0.2175,  0.2175, 0.2175, 0.395, 0.395],
            [  0.1,   -0.02,   0.1,  0.1,   -0.02,  0.1,   0.1,   0.1],
            [   0.,   0.0,    0.,np.pi/2,np.pi/2,  0.0,   0.0,   0.0]])
        #Specify time interval for each segment
        time_array = np.array([   2.,    2.,    2.,   2.,    2.,   2.,  2.,   2.])
        #Compute the total time needed
        tf = np.sum(time_array)
        #Use the inverseKinematics function to generate the starting joint configuration
        initial_angles = KMTCS.inverseKinematics(0.0, 0.395, 0.1, 0.0)

        if traj_mode == "minjerk":
            #Compute the minimum jerk constants
            joint_const = TG.minJerkSetup_now(initial_angles,tf,waypoints,t_array=time_array)
        else:
            joint_const = None

    #Loop for the duration of the control interval
    if(args.plot == "all"):
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        plt.ion()
        fig.show()
        fig.canvas.draw()

        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        while control_info['tf']-time_from_start> 0:
            time_from_start += 0.1
            #Compute the next time step for the minimum jerk trajectory
            if control_info['type'] == "MinJerk":
                pos_v,vel_v,accel_v = TG.minJerkStep_now(time_from_start,tf,waypoints,joint_const,t_array=time_array)

            if control_info['type'] == "Circle":
                pos_v,vel_v,accel_v = TG.generateJointTrajectory_now(time_from_start)

            if control_info['type'] == "Cubic":
                # cubic spline 
                a = None
            # print(pos_v)
            H = KMTCS.forwardKinematics(pos_v)
            ax.scatter(H[0,3], H[1,3], H[2,3], 'g*')
            fig.canvas.draw()
            plt.pause(0.01)
    else:
        while control_info['tf']-time_from_start> 0:
            time_from_start += 0.1
            #Compute the next time step for the minimum jerk trajectory
            if control_info['type'] == "MinJerk":
                pos_v,vel_v,accel_v = TG.minJerkStep_now(time_from_start,tf,waypoints,joint_const,t_array=time_array)

            if control_info['type'] == "Circle":
                pos_v,vel_v,accel_v = TG.generateJointTrajectory_now(time_from_start)
