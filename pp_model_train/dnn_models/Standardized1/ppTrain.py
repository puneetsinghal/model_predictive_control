import tensorflow as tf
import numpy as np
import scipy.io as sio
import mpcNetwork
import pickle
import os
from mpcNetwork import generateData
from shutil import copyfile
# Pick and place robot with 2 DOF

if __name__ == '__main__':
    # Input [sita1 sita2 w1 w2 tao1 tao2] of t
    # Output [sita1 sita2 w1 w2] of t+1
    # Path to certain directories

    load_model = False
    # Network training parameters
    lrn_rate = 1e-4
    total_epoch = 1000000
    batch_size = 256
    
    #network_type = 'Feedforward'
    network_type = 'Standardized'
    # no. of frames the input data takes
    frames = 1
    task_name = network_type + str(frames)
    path = './dnn_models/' + task_name + '/'
    if not os.path.exists(path):
        os.makedirs(path)
    copyfile('ppTrain.py', path + 'ppTrain.py')
    copyfile('mpcNetwork.py', path + 'mpcNetwork.py')
    model_path = path + '/model.cpkt'
    log_path = path + '/logs'
    train_file = 'data_for_rnn_small.dat'
    
    

    pickle_data = pickle.load(open(train_file,'rb'))
    input_data, output_data = generateData(pickle_data)
    network = mpcNetwork.Network(lrn_rate, total_epoch, batch_size, network_type, model_path, log_path) 
    
    with tf.Session() as sess:
        network.train(sess, train_file, load_model)
