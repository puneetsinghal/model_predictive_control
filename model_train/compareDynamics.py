from scipy.integrate import ode
import scipy.io as sio
from robot import Acrobot
import numpy as np
import mpcNetwork
import tensorflow as tf

def IntegrationEstimation(xk, uk, Ts, robot, M = 5):
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


def Purturb(x, percent):
    x_p = x * (1 + percent * 2*(np.random.rand(1) - 0.5))
    return x_p 

def PurturbParams(purturbed_params, percentage):
    purturbed_params = {}
    purturbed_params['m1'] = Purturb(params['m1'], percentage)
    purturbed_params['m2'] = Purturb(params['m2'], percentage)
    purturbed_params['l1'] = Purturb(params['l1'], percentage)
    purturbed_params['l2'] = Purturb(params['l2'], percentage) 

    purturbed_params['I1'] = Purturb(params['I1'], percentage)
    purturbed_params['I2'] = Purturb(params['I2'], percentage)

    purturbed_params['x0'] = params['x0'] # current state space
    purturbed_params['Ts'] = params['Ts']             # time iteration
    purturbed_params['g'] = params['g']
    purturbed_params['N'] = params['N']             # Event horizon = 10
    purturbed_params['u0'] = params['u0']# initial force
    return purturbed_params


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
    params['u0'] = np.array(0).reshape(1,1)               # initial force
    model_robot = Acrobot(params)

    
    # Set up uncertainty real model
    percentage = 0.05  # the parameters are +- 5 percent uncertain
    purturbed_params = PurturbParams(params, percentage)
    real_robot = Acrobot(purturbed_params)
    
    T = 10.0
    Ts = params['Ts']
    xHistory = params['x0'] 
    uHistory = []
    xk = params['x0']
    uk = 0
    
    """
    # Simulate responses on real robot
    for i in range(int(10.0/Ts)):
        # Decide the optimal usage
        # Use 0 usage for now
        uk = np.array(0).reshape(1,1)
        if i == 0:
            uHistory = uk
        else:
            uHistory = np.append(uHistory, uk, axis=1)
        
        xk = IntegrationEstimation(xk, uk, Ts, real_robot, 30) 
        xHistory = np.append(xHistory, xk, axis=1)
    
    print(uHistory)
    np.save('xHistory', xHistory)
    np.save('uHistory', uHistory)
    """
    xHistory = np.load('xHistory.npy')
    uHistory = np.load('uHistory.npy')
    
    
    data_points = uHistory.shape[1]
        
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
    
    sess = tf.Session()
    network.load_model_weights(sess) 
    # See response on our model
    steps = 5 # Unroll 5 time steps
    for i in range(data_points):
        xk = xHistory[:,i]
        #print(xk)
        x_pred = xk.reshape(4,1)
        x_pred_net = xk.reshape(1,4)
        # Unroll and predict
        for j in range(steps):
            if i+j <= data_points-1:
                
                uk = uHistory[:, i+j].reshape(1,1)
                uk = np.array(0).reshape(1,1)
                
                # Model
                x_pred = IntegrationEstimation(x_pred, uk, Ts, model_robot, 30)
                error = np.square(xHistory[:,i+j+1].reshape(4,1) - x_pred)
                
                # Neural Model
                x_pred_net = network.predict_next_state(sess, x_pred_net, uk)            
                error_net = np.square(xHistory[:,i+j+1].reshape(4,1) - x_pred_net.reshape(4,1))

                if i+j == 0:
                    errorHistory = error 
                    errorHistory_net = error_net 
                else:
                    errorHistory = np.append(errorHistory, error, axis=1)
                    errorHistory_net = np.append(errorHistory_net, error_net, axis=1)
        # xk = IntegrationEstimation(xk, uk, Ts, model_robot, 30)
        #predictHistory = np.append(predictHistory, xk, axis=1)
    np.save('errorHistory', errorHistory)
    print(np.mean(np.sqrt(errorHistory), axis=1, keepdims=True))  
    print(np.mean(np.sqrt(errorHistory_net), axis=1, keepdims=True))  
      
    sess.close() 

    """
    #robot.animate(np.array(xHistory))

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

    net_xk = np.reshape(sim_xk,[1,4]) 
    net_uk = np.array([[0]])
    net_xHistory = [net_xk[0,:].tolist()]
    
    with tf.Session() as test_sess:
        network.load_model_weights(test_sess)

        for i in range(int(10.0/params['Ts'])):
            
            net_xk = network.predict_next_state(test_sess, net_xk, net_uk)            
            net_xHistory += [net_xk[0,:].tolist()] 
    robot.animate(np.array(net_xHistory))
    """




