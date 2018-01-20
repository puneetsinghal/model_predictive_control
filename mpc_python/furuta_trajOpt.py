# Evan Harber eharber@andrew.cmu.edu
# For detailed information on the minimize function visit:
# https://docs.scipy.org/doc/scipy/reference/tutorial/optimize.html

from scipy.optimize import minimize
import matplotlib.pyplot as plt
import numpy as np
from math import *
import time

print('Start Control')

def Dynamics(x, F):
    
    m_cart = 1.0   # cart mass
    m_pend = 1.0   # pendulum mass
    g = 9.81       # gravity of earth
    l = 1          # pendulum length
    r = 0.5        # length of actuator

    phi = x[0]
    dphi = x[1]
    theta = x[2]
    dtheta = x[3]

    dxdt = np.zeros(4)

    dxdt[0] = dphi
    dxdt[1] = (2*F*l + g*l*m_pend*r*sin(2*theta) - 2*dphi*dtheta*l**3*m_pend*sin(2*theta) - 2*dtheta**2*l**2*m_pend*r*sin(theta) + 2*dphi**2*m_pend*r**3*sin(theta)*(sin(theta)**2 - 1) + 2*dphi*dtheta*l*m_pend*r**2*sin(2*theta) - 2*dphi**2*l**2*m_pend*r*sin(theta)*(sin(theta)**2 - 1))/(2*l**3*m_pend + 2*l*m_cart*r**2 - 2*l**3*m_pend*cos(theta)**2)
    dxdt[2] = dtheta
    dxdt[3] = (2*F*l*r*cos(theta) + dphi**2*l**4*m_pend*sin(2*theta) - dphi**2*m_cart*r**4*sin(2*theta) + 2*g*l**3*m_pend*sin(theta) - 2*g*l**3*m_pend*(sin(theta) - sin(theta)**3) + 2*g*l*m_cart*r**2*sin(theta) + 2*g*l*m_pend*r**2*(sin(theta) - sin(theta)**3) + dphi**2*l**2*m_cart*r**2*sin(2*theta) - dphi**2*l**2*m_pend*r**2*sin(2*theta) - dtheta**2*l**2*m_pend*r**2*sin(2*theta) - 2*dphi**2*l**4*m_pend*cos(theta)**3*sin(theta) - 2*dphi**2*m_pend*r**4*cos(theta)**3*sin(theta) + 4*dphi**2*l**2*m_pend*r**2*cos(theta)**3*sin(theta) - 4*dphi*dtheta*l*m_pend*r**3*sin(theta)*(sin(theta)**2 - 1) + 4*dphi*dtheta*l**3*m_pend*r*sin(theta)*(sin(theta)**2 - 1))/(2*l**4*m_pend + 2*l**2*m_cart*r**2 - 2*l**4*m_pend*cos(theta)**2)

    return dxdt

def IntegrationEstimation(xk, uk, Ts, M = 5):
    # Runge-Kutta 4th order (M = 5 optimization problem, M = 30 updating state space)
    # Better ODE solvers can be used here 
    delta = Ts/M
    xk1 = xk
    for ct in range(M):
        k1 = Dynamics(xk1,uk)
        k2 = Dynamics(xk1 + k1*delta/2, uk)
        k3 = Dynamics(xk1 + k2*delta/2, uk)
        k4 = Dynamics(xk1 + k3*delta, uk)
        xk1 = xk1 + (k1/6 + k2/3 + k3/3 + k4/6)*delta
    return xk1

def CostFunction(u, xk, u0, xref):
    # Cost function taken from MatLab's nMPC example code 
    Q = np.diag([10,1,10,1])
    R = 0.01
    J = 0

    for ct in range(len(u)):
        uk = u[ct]

        xk1 = IntegrationEstimation(xk, uk, Ts);

        # for i in range(len(xk1)):
        J += np.matmul(np.matmul((xk1-xref).T,Q),(xk1-xref).T)
 
        if ct==0:
            J += (uk-u0)*R*(uk-u0)
        else:
            J += (uk-u[ct-1])*R*(uk-u[ct-1])
        
        xk = xk1
    return J

def Contraints(u, xk):
    phiMin = -10
    phiMax = 10

    c = np.zeros(N*2)

    for ct in range(len(u)):
        uk = u[ct]

        xk1 = IntegrationEstimation(xk, uk, Ts, 5)

        # -phi + phiMin > 0
        c[2*ct] = xk1[0]-phiMin
        # phi - phiMax > 0
        c[2*ct+1] = -xk1[0]+phiMax

        #update plant state and input for next step
        xk = xk1
    return c

if __name__ == '__main__':

    t = time.time()
    data = {}
    # data['x'] = [0, 0, -pi, 0]   # current state space
    x = [0, 0, -pi, 0]   # current state space
    Ts = 0.1             # time iteration
    N = 10               # Event horizon = 10
    xref1 = [0, 0, 0, 0] # reference state space
    xref2 = [pi, 0, 0, 0]
    u0 = 0               # initial force
    Duration = 20        # max time 

    uopt = np.zeros(N) 

    uHistory = [u0]      # force history
    x0History = [x[0]]   # position history
    x1History = [x[1]] 
    x2History = [x[2]] 
    x3History = [x[3]] 

    LB = -100            # input force Lower Bound 
    UP = 100             # input force Upper Bound

    bnds = ((LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP))
    cons = ({'type': 'ineq', 'fun': Contraints, 'args':(x,)})
    # contraint jacobian can also be added to possibly speed up 
    print('total number of loops are: {}'.format(int(Duration/Ts)))
    for ct in range(int(Duration/Ts)):
        print('loop # {}'.format(ct))
        if ct*Ts<10:
            xref = xref1
        else:
            xref = xref2

        res = minimize(CostFunction, uopt, args=(x, u0, xref,), method='SLSQP', bounds=bnds, constraints=cons)
        # jac=func_deriv arg that could possibly speed up time
        uopt = res.x
        u0 = uopt[0]
        x = IntegrationEstimation(x, u0, Ts, 30)

        uHistory.append(u0)
        x0History.append(x[0])
        x1History.append(x[1])
        x2History.append(x[2])
        x3History.append(x[3])

    print(time.time()-t)

    # Displaying Graphs 
    x = np.linspace(0, Duration, Duration/Ts+1)

    f, axarr = plt.subplots(2, 2)
    axarr[0, 0].plot(x, x0History)
    axarr[0, 0].set_title('Actuator Position')
    axarr[0, 0].set_xlabel('time')
    axarr[0, 0].set_ylabel('phi')

    axarr[0, 1].plot(x, x1History)
    axarr[0, 1].set_title('Actuator Velocity')
    axarr[0, 1].set_xlabel('time')
    axarr[0, 1].set_ylabel('dphi')

    axarr[1, 0].plot(x, x2History)
    axarr[1, 0].set_title('Pendulum Position')
    axarr[1, 0].set_xlabel('time')
    axarr[1, 0].set_ylabel('theta')

    axarr[1, 1].plot(x, x3History)
    axarr[1, 1].set_title('Pendulum Velocity')
    axarr[1, 1].set_xlabel('time')
    axarr[1, 1].set_ylabel('dtheta')

    plt.show()

