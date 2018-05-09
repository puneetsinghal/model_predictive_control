import tensorflow as tf
import numpy as np
import scipy.io as sio
import mpcNetwork

if __name__ == '__main__':
    # Input [sita1 sita2 w1 w2 tao] of t
    # Output [sita1 sita2 w1 w2] of t+1
    # Path to certain directories
    model_path = './dnn_models/model.cpkt'
    log_path = './logs/test'
    train_file = 'train_data.mat'
    test_file = 'test_data.mat'
    network_type = 'Feedforward'
    load_model = False
    # Network training parameters
    lrn_rate = 0.0001
    total_epoch = 1000000
    batch_size = 256
    
    input_data, output_data = mpcNetwork.load_matfile(train_file)
    print(input_data.shape)
    mean = np.mean(input_data, axis=0, keepdims=True)
    std = np.std(input_data, axis=0, keepdims=True)
    print(mean)
    print(std)


    network = mpcNetwork.Network(lrn_rate, total_epoch, batch_size, network_type, model_path, log_path) 
    
    
    with tf.Session() as sess:
        network.train(sess, train_file, load_model)
        #network.test(sess, test_file, load_model)

    """ 
    # Test the network as the model
    test_cycle = 100
    x0 = np.array([[0.1,0,0,0]])
    u0 = np.array([[0]])
    print(x0.shape) 
    x_prev = x0
    u_prev = u0
     
    # Test the prediction
    with tf.Session() as test_sess:
        network.load_model_weights(test_sess)
        
        for t in range(test_cycle):
            next_state = network.predict_next_state(test_sess, x_prev, u_prev)
            #print(next_state[0])
            x_prev = next_state
    """


