from scipy.integrate import ode
import scipy.io as sio
from robot import Acrobot
import numpy as np
import mpcNetwork
import tensorflow as tf
import pickle
import matplotlib.pyplot as plt

def generateData(data): 
    #Uploading the data into xk_data and uk_data
    xk_data = np.empty([len(data),4])
    uk_data = np.empty([len(data),1])
    for row in range(len(data)):
        x = data[row][0]
        u = data[row][1]
        xk_data[row,:] = np.array(x)
        uk_data[row] = np.array(u)

    xk_data = xk_data.T
    uk_data = uk_data.T
    data = np.concatenate((xk_data, uk_data))
    print('data',data.shape)
    #input_data = data[:,:-1]
    #output_data = data[:4,1:]
    points = xk_data.shape[1]
    episodes = points // 1000
    print('episodes', episodes)
    xHistory = []
    uHistory = []
    for episode in range(episodes):
        xhistory_k = data[:4,1000*episode:1000*(episode+1)]
        uhistory_k = np.array([data[4,1000*episode:1000*(episode+1)]])
        
        xHistory.append(xhistory_k)
        uHistory.append(uhistory_k) 
    
    return xHistory, uHistory

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
    # Set up purturbed model
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
    
    # Set up uncertainty real model
    percentage = 0.05  # the parameters are +- 5 percent uncertain
    purturbed_params = PurturbParams(params, percentage)
    compare_robot_5 = Acrobot(purturbed_params)
    
    # Set up uncertainty real model
    percentage = 0.20  # the parameters are +- 5 percent uncertain
    purturbed_params = PurturbParams(params, percentage)
    compare_robot_10 = Acrobot(purturbed_params)
    
    T = 10.0
    Ts = params['Ts']
    xHistory = params['x0'] 
    uHistory = []
    xk = params['x0']
    uk = 0
    
    
    # Set up DNN model
    model_path = './dnn_models/model.cpkt'
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



    # Load simulated data 
    data_file = 'acrobot_small_data_10000.dat' 
    pickle_data = pickle.load(open(data_file, 'rb'))
    xHistory_all, uHistory_all = generateData(pickle_data)
    # Get prediction error of purturbed plant
    data_points = xHistory_all[0].shape[1]
    episodes = len(xHistory_all)
    episodes = 1
    data_points = 100
    steps = 5 # steps to unroll
    purturb_error_5 = np.zeros((4,steps))
    purturb_error_10 = np.zeros((4,steps))
    dnn_error = np.zeros((4,steps))
    
    for episode in range(episodes):
        print(episode)
        xHistory = xHistory_all[episode]
        uHistory = uHistory_all[episode]
        for i in range(data_points):
            xk = xHistory[:,i]
            uk = uHistory[0,i]
            x_pred_5 = xk.reshape(4,1)
            x_pred_10 = xk.reshape(4,1)
            x_pred_net = xk.reshape(1,4)
            # Unroll and predict
            for j in range(steps):
                if i+j <= data_points-2:
                    
                    uk = uHistory[:, i+j].reshape(1,1)
                    uk = np.array(0).reshape(1,1)
                    
                    # Model
                    x_pred_5 = IntegrationEstimation(x_pred_5, uk, Ts, compare_robot_5, 30)
                    error = np.square(xHistory[:,i+j+1].reshape(4,1) - x_pred_5)
                    purturb_error_5[:, j] += error[:,0]
                    
                    # Model
                    x_pred_10 = IntegrationEstimation(x_pred_10, uk, Ts, compare_robot_10, 30)
                    error = np.square(xHistory[:,i+j+1].reshape(4,1) - x_pred_10)
                    purturb_error_10[:, j] += error[:,0]
                    
                    # Neural Model
                    x_pred_net = network.predict_next_state(sess, x_pred_net, uk)            
                    error_net = np.square(xHistory[:,i+j+1].reshape(4,1) - x_pred_net.reshape(4,1))
                    dnn_error[:, j] += error_net[:,0]
                    """
                    if i+j == 0:
                        errorHistory = error 
                        #errorHistory_net = error_net 
                    else:
                        errorHistory = np.append(errorHistory, error, axis=1)
                        #errorHistory_net = np.append(errorHistory_net, error_net, axis=1)
                    """
           # xk = IntegrationEstimation(xk, uk, Ts, model_robot, 30)
            #predictHistory = np.append(predictHistory, xk, axis=1)
    sess.close()
     
    plt.plot(np.arange(steps)+1,purturb_error_5[0,:], label='uncertain 5 percent')
    #plt.plot(np.arange(steps)+1,purturb_error_10[0,:], label='uncertain 10 percent')
    plt.plot(np.arange(steps)+1, dnn_error[0,:], label='neural network')
    plt.legend()
    plt.show()
    #print(np.mean(np.sqrt(errorHistory), axis=1, keepdims=True))  
    #print(np.mean(np.sqrt(errorHistory_net), axis=1, keepdims=True))  
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
    xHistory = np.load('xHistory.npy')V
    uHistory = np.load('uHistory.npy')
    """ 
    """ 
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
