from scipy.integrate import ode
import scipy.io as sio
from robot import Acrobot
import numpy as np
import mpcNetwork
import tensorflow as tf

def IntegrationEstimation(xk, uk, Ts, M = 5):
    # Runge-Kutta 4th order (M = 5 optimization problem, M = 30 updating state space)
    # Better ODE solvers can be used here 
    delta = Ts/M
    xk1 = xk
    for ct in range(M):
        k1 = robot.dynamics(xk1, uk)
        k2 = robot.dynamics(xk1 + k1*delta/2, uk)
        k3 = robot.dynamics(xk1 + k2*delta/2, uk)
        k4 = robot.dynamics(xk1 + k3*delta, uk)
        xk1 = xk1 + delta*(k1/6 + k2/3 + k3/3 + k4/6)
    return xk1

if __name__ == '__main__':
    # Set up analytical model
    params = {}
    params['m1'] = 1.
    params['m2'] = 1.
    params['l1'] = 0.5
    params['l2'] = 0.5 
    params['g'] = -9.81
    params['I1'] = 0.1
    params['I2'] = 0.1

    params['x0'] = np.array([1, 0, 0, 0]).reshape((4,1)) # current state space
    params['Ts'] = 0.01             # time iteration
    params['N'] = 10              # Event horizon = 10
    params['u0'] = 0               # initial force
    robot = Acrobot(params)


    # Set up neural network model
    model_path = './models/model.cpkt'
    log_path = './logs/test'
    train_file = 'train_data.mat'
    test_file = 'test_data.mat'
    network_type = 'Feedforward'
    load_model = True
    lrn_rate = 0.0001
    total_epoch = 10000
    batch_size = 256

    network = mpcNetwork.Network(lrn_rate, total_epoch, batch_size, network_type, model_path,
            log_path)
        
    # N = size(results.x,2);
    sim_xk = params['x0']
    sim_uk = 0
    sim_xHistory = [sim_xk[:,0].tolist()]
    print(sim_xk.shape)

    net_xk = np.reshape(sim_xk,[1,4]) 
    net_uk = np.array([[0]])
    net_xHistory = [net_xk[0,:].tolist()]
    
    with tf.Session() as test_sess:
        network.load_model_weights(test_sess)

        for i in range(int(10.0/params['Ts'])):
            sim_xk = IntegrationEstimation(sim_xk, sim_uk, params['Ts'], 30)
            sim_xHistory += [sim_xk[:,0].tolist()]
            
            net_xk = network.predict_next_state(test_sess, net_xk, net_uk)            
            net_xHistory += [net_xk[0,:].tolist()] 
        print(np.array(sim_xHistory).shape)
        print(np.array(net_xHistory).shape)
    robot.animate(np.array(net_xHistory))






