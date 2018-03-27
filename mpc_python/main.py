# Author:
#     Puneet Singhal: puneetatpks@gmail.com
#     Aditya Raghu Chavvali:

# For detailed information on the minimize function visit:
# https://docs.scipy.org/doc/scipy/reference/tutorial/optimize.html

import matplotlib.pyplot as plt
import numpy as np
from math import *
import time
import pickle
import argparse
from copy import copy

from robot import Acrobot
from mpc import MPC

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='test')
    parser.add_argument('--suffix', type=str, default='')

    args = parser.parse_args()
    t = time.time()


    params = {}
    params['m1'] = 1.
    params['m2'] = 1.
    params['l1'] = 0.5
    params['l2'] = 0.5 
    params['g'] = -9.81
    params['I1'] = 0.1
    params['I2'] = 0.1
    params['x0'] = np.array([0, 0, 0, 0]).reshape((4,1)) # current state space
    params['Ts'] = 0.05             # time iteration
    params['N'] = 10              # Event horizon = 10
    params['u0'] = 0               # initial force
    params['Duration'] = 2.5        # max time 
    params['numIterations'] = int(params['Duration']/params['Ts'])
    print('total number of iterations are: {}'.format(params['numIterations']))

    params['xMidPoints'] = np.array([[0, 0.7022, 0.5790, 0.0513, -1.0845, -1.0845, 1.3254, 2.1820, 2.6058, 3.1416], 
                            [0, -1.2763, -2.2228, -2.0331, 0.0265, 2.4347, 4.1818, 4.4895, 3.9700, 3.1416],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    # params['xMidPoints'] = np.array([[0, 0.7022, 0.5790, 0.0513, -2., -2., 1.3254, 2.1820, 2.6058, 3.1416], 
    #                         [0, -1.2763, -2.2228, -2.0331, 0.0265, 0.0265, 4.1818, 4.4895, 3.9700, 3.1416],
    #                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
    #                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    
    params['numsegment'] = params['xMidPoints'].shape[1] - 1 # subtracting one to get segments
    params['Q'] = np.diag([100.,10.,0.1, 0.1])
    params['R'] = 0.01

    uopt = np.zeros(params['N']) 
    u0 = params['u0']
    x = params['x0']
    
    # arrays to save data
    uHistory = [u0]             # force history
    xHistory = [params['x0'][:,0].tolist()]       # position history
    xRefHistory = []

    robot = Acrobot(params)
    print("Robot object created")

    LB = -100            # input force Lower Bound 
    UP = 100             # input force Upper Bound
    bnds = ((LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP))
    controller = MPC(robot, params, bnds)
    print("Controller object created")

    # cons = ({'type': 'ineq', 'fun': Contraints, 'args':(x, robot,)})
    # contraint jacobian can also be added to possibly speed up 
    
    if (args.mode == 'train'):
        for ct in range(params['numIterations']):
            print('iteration # {} of {}'.format(ct, params['numIterations']))
            
            # fetch the xref from waypoints
            index = int(params['numsegment']*ct*params['Ts']/params['Duration']) + 1 
            xref = copy(params['xMidPoints'][:,index])
            xref = xref.reshape((4,1))
            xref[1,0] -= xref[0,0]
            xref[3,0] -= xref[2,0]
            xRefHistory += [xref[:,0].tolist()]
            print("reference value: {}, {}".format(index, xref.T))

            # optimize
            results = controller.optimize(uopt, x, u0, xref)

            # prepare variable for next run and save the output in history arrays
            uopt = results.x
            u0 = uopt[0]
            x = controller.IntegrationEstimation(x, u0, 30)
            print("next state: {}".format(x.T))
            
            uHistory.append(u0)
            xHistory += [x[:,0].tolist()]

        print(time.time()-t)

        filename = './acrobot_results' + args.suffix
        pickle.dump([params, xHistory, uHistory, xRefHistory], open(filename, 'wb'))
    else:
        filename = './acrobot_results' + args.suffix
        params, xHistory, uHistory, xRefHistory = pickle.load(open(filename, 'rb'))   
    
    # Displaying Graphs
    t = np.linspace(0, params['Duration'], params['Duration']/params['Ts']+1)
    xRefHistory += [[pi,0,0,0]]
    xRefHistory = np.array(xRefHistory)
    xHistory = np.array(xHistory)

    f, axarr = plt.subplots(2, 2)
    axarr[0, 0].plot(t, xHistory[:,0])
    axarr[0, 0].plot(t, xRefHistory[:,0])
    axarr[0, 0].set_title('Joint 1 Position')
    axarr[0, 0].set_xlabel('time')
    axarr[0, 0].set_ylabel('theta_1')

    axarr[0, 1].plot(t, xHistory[:,1])
    axarr[0, 1].plot(t, xRefHistory[:,1])
    axarr[0, 1].set_title('Joint 2 Position')
    axarr[0, 1].set_xlabel('time')
    axarr[0, 1].set_ylabel('theta_2')

    axarr[1, 0].plot(t, xHistory[:,2])
    axarr[1, 0].plot(t, xRefHistory[:,2])
    axarr[1, 0].set_title('Joint 1 Velocity')
    axarr[1, 0].set_xlabel('time')
    axarr[1, 0].set_ylabel('ang_vel_1')

    axarr[1, 1].plot(t, xHistory[:,3])
    axarr[1, 1].plot(t, xRefHistory[:,3])
    axarr[1, 1].set_title('Joint 2 Velocity')
    axarr[1, 1].set_xlabel('time')
    axarr[1, 1].set_ylabel('ang_vel_2')

    plt.figure()
    plt.plot(t, uHistory)
    plt.legend('Actuator Torque')
    plt.xlabel('time')
    plt.ylabel('inputs')

    robot.animate(xHistory)
    plt.show()

