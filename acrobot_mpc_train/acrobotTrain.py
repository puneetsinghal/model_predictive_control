import tensorflow as tf
import numpy as np
import scipy.io as sio
import mpcNetwork

if __name__ == '__main__':
    # Build the computation graph
    # Input [sita1 sita2 w1 w2 tao] of t
    # Output [sita1 sita2 w1 w2] of t+1
    # Path to certain directories
    model_path = './models/model.cpkt'
    log_path = './logs/test'
    train_file = 'train_data.mat'
    test_file = 'test_data.mat'
    network_type = 'Feedforward' 
    # Network training parameters
    lrn_rate = 0.0001
    total_epoch = 100000
    batch_size = 256
   
    network = mpcNetwork.Network(lrn_rate, total_epoch, batch_size, network_type, model_path, log_path) 
    

    with tf.Session() as sess:
        network.train(sess, train_file, model_path)
        network.test(sess, test_file)



