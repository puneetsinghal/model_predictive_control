# Evan Harber eharber@andrew.cmu.edu
# For detailed information on the minimize function visit:
# https://docs.scipy.org/doc/scipy/reference/tutorial/optimize.html

from scipy.optimize import minimize
import matplotlib.pyplot as plt
import numpy as np
from math import *
import time
from robot import Acrobot
import pickle
import argparse
from copy import copy


print('Start Control')

def IntegrationEstimation(xk, uk, robot, M = 5):
    # Runge-Kutta 4th order (M = 5 optimization problem, M = 30 updating state space)
    # Better ODE solvers can be used here 
    delta = robot.Ts/M
    xk1 = xk
    for ct in range(M):
        k1 = robot.dynamics(xk1, uk)
        k2 = robot.dynamics(xk1 + k1*delta/2, uk)
        k3 = robot.dynamics(xk1 + k2*delta/2, uk)
        k4 = robot.dynamics(xk1 + k3*delta, uk)
        xk1 = xk1 + delta*(k1/6 + k2/3 + k3/3 + k4/6)
    return xk1

def CostFunction(u, xk, u0, xref, robot):
    # Cost function taken from MatLab's nMPC example code 
    Q = np.diag([100.,10.,0.1, 0.1])
    R = 0.01
    J = 0.

    for ct in range(len(u)):
        uk = u[ct]

        xk1 = IntegrationEstimation(xk, uk, robot);

        # for i in range(len(xk1)):
        J += np.matmul(np.matmul((xk1-xref).T,Q),(xk1-xref))
 
        if ct==0:
            J += (uk-u0)*R*(uk-u0)
        else:
            J += (uk-u[ct-1])*R*(uk-u[ct-1])
        
        xk = copy(xk1)
    return J

def Contraints(u, xk, robot):
    phiMin = -10
    phiMax = 10

    c = np.zeros(robot.N*2)

    for ct in range(len(u)):
        uk = u[ct]

        xk1 = IntegrationEstimation(xk, uk, robot, 5)

        # -phi + phiMin > 0
        c[2*ct] = xk1[0]-phiMin
        # phi - phiMax > 0
        c[2*ct+1] = -xk1[0]+phiMax

        #update plant state and input for next step
        xk = xk1
    return c

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='test')
    parser.add_argument('--suffix', type=str, default='')

    args = parser.parse_args()

    params = {}
    params['m1'] = 1.
    params['m2'] = 1.
    params['l1'] = 0.5
    params['l2'] = 0.5 
    params['g'] = -9.81
    params['I1'] = 0.1
    params['I2'] = 0.1

    t = time.time()

    params['x0'] = np.array([0, 0, 0, 0]).reshape((4,1)) # current state space
    params['Ts'] = 0.05             # time iteration
    params['N'] = 10              # Event horizon = 10
    params['u0'] = 0               # initial force
    params['Duration'] = 2.5        # max time 
    params['numIterations'] = int(params['Duration']/params['Ts'])
    print('total number of iterations are: {}'.format(params['numIterations']))
    
    # xref1 = np.array([-pi/2, 0, 0, 0]).reshape((4,1)) # reference state space
    # xf = np.array([pi, 0, 0, 0]).reshape((4,1))
    
    params['xMidPoints'] = np.array([[0, 0.7022, 0.5790, 0.0513, -1.0845, -1.0845, 1.3254, 2.1820, 2.6058, 3.1416], 
                            [0, -1.2763, -2.2228, -2.0331, 0.0265, 2.4347, 4.1818, 4.4895, 3.9700, 3.1416],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    # params['xMidPoints'] = np.array([[0, 0.7022, 0.5790, 0.0513, -2., -2., 1.3254, 2.1820, 2.6058, 3.1416], 
    #                         [0, -1.2763, -2.2228, -2.0331, 0.0265, 0.0265, 4.1818, 4.4895, 3.9700, 3.1416],
    #                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
    #                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    
    params['numsegment'] = params['xMidPoints'].shape[1] - 1 # subtracting one to get segments

    uopt = np.zeros(params['N']) 
    u0 = params['u0']
    x = params['x0']
    uHistory = [u0]             # force history
    xHistory = [params['x0'][:,0].tolist()]       # position history
    xRefHistory = []

    LB = -100            # input force Lower Bound 
    UP = 100             # input force Upper Bound

    robot = Acrobot(params)
    bnds = ((LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP))
    cons = ({'type': 'ineq', 'fun': Contraints, 'args':(x, robot,)})
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
            res = minimize(CostFunction, uopt, args=(x, u0, xref, robot), method='SLSQP', bounds=bnds, constraints=cons)
            # jac=func_deriv arg that could possibly speed up time
            
            # prepare variable for next run and save the output in history arrays
            uopt = res.x
            u0 = uopt[0]
            x = IntegrationEstimation(x, u0, robot, 30)
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

