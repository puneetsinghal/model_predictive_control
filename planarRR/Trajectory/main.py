
#Standard Python Libraries
import sys
sys.path.insert(0, './..')
import os
from IPython import embed
import pickle

#Third Party Software
import numpy as np
import matplotlib.pyplot as plt


#Python Script
import TrajectoryGenerartor as TG
from robot import PlanarRR

if __name__ == '__main__':

    #MinJerk trajectory type specifies waypoints to move to with zero velocity and accelration
    #boundary conditions
    waypoints = np.array([[1, -1, -1.], [1, 1, 0]])
    
    #Specify time interval for each segmentt
    time_array = np.array([4., 4., 4.])
    #Compute the total time needed
    tf = np.sum(time_array)

    #Use the inverseKinematics function to generate the starting joint configuration
    initial_angles = PlanarRR.inverseKinematics(1., 0.)
    #Compute the minimum jerk constants
    joint_const = TG.minJerkSetup_now(initial_angles,tf,waypoints,t_array=time_array)
   
    #Loop for the duration of the control interval
    traj_joint = []
    X = []
    time_from_start = 0.
    dt = 0.01
    while tf-time_from_start >= 0:        
        #Compute the next time step for the minimum jerk trajectory
        pos_v, vel_v, accel_v = TG.minJerkStep_now(time_from_start, tf, waypoints, joint_const, t_array=time_array)
        # embed()
        traj_joint.append(pos_v[:,0].tolist() + vel_v[:,0].tolist())
        p1, p2 = PlanarRR.forwardKinematics(pos_v)
        X.append(p2.tolist())
        time_from_start += dt
    
    pickle.dump(traj_joint,open('planarRR_trajectory','wb'))

    traj_joint = np.array(traj_joint)
    X = np.array(X)
    
    t = np.linspace(0, tf, tf/dt+1, True)
    # xHistory = np.array(xHistory)
    # print(xRefHistory)
    # print(xRefHistory.shape)
    f, axarr = plt.subplots(2, 1)
    axarr[0].plot(t, traj_joint[:,0])
    axarr[0].set_title('Joint 1 Position')
    axarr[0].set_xlabel('time')
    axarr[0].set_ylabel('theta_1')

    plt.plot(t, traj_joint[:,1])
    axarr[1].set_title('Joint 2 Position')
    axarr[1].set_xlabel('time')
    axarr[1].set_ylabel('theta_2')

    plt.figure()
    plt.plot(X[:,0], X[:,1])
    plt.title('reference trajectory')
    plt.xlabel('x-position')
    plt.ylabel('y-position')
    plt.show()
    embed()
