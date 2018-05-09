import tensorflow as tf
import numpy as np
import scipy.io as sio
import sys
import shutil
import os
import pickle
from matplotlib import pyplot as plt
import argparse
from logger import Logger
from RNN import RNNNetwork

def make_log_dir(log_parent_dir):
	import datetime, os
	current_timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
	log_dir = log_parent_dir + current_timestamp
	if not os.path.exists(log_dir):
		os.makedirs(log_dir)
	return log_dir

def main(args):
	# Network parameters
	time_steps = 5
	batch_size = 1024
	input_state_size = 5 # [sita1 w1 sita2 w2 torque]_t
	output_state_size = 4 # [sita1 w1 sita2 w2]_t+1
	hidden_state_size = 8
	num_epoch = 100000
	lrn_rate = 1e-4
	dropout_prob = 0.85
	
	# Training data file 	
	dataFileName = args.data

	network = RNNNetwork(lrn_rate, input_state_size, hidden_state_size, output_state_size)
	if(args.mode=='train'):
		if(args.data == None):
			dataFileName = 'acrobot_large_data_500000.dat'
		
		log_dir 	= make_log_dir('./')

		# copy python scripts to log_dir. This helps preserved the parameters used to train the model.
		shutil.copyfile('RNN.py', (log_dir + '/RNN.py'))  
		shutil.copyfile('main.py', (log_dir + '/main.py'))  
		shutil.copyfile('logger.py', (log_dir + '/logger.py')) 
		shutil.copyfile('testDynamics.py', (log_dir + '/testDynamics.py'))
		shutil.copyfile('robot.py', (log_dir + '/robot.py'))
		shutil.copyfile('data_processor.py', (log_dir + '/data_processor.py'))

		with tf.Session() as sess:
			network.train(sess, num_epoch, dropout_prob, dataFileName, batch_size, time_steps, log_dir, args.model)

	elif(args.mode=='test'):
		# use 'acrobot_small_test_data_10000.dat' for testing as default if datafile is not given in arguments
		if(args.data == None):
			dataFileName = 'acrobot_small_test_data_10000.dat'
		modelName = args.model
		with tf.Session() as sess:
			network.test(sess, num_epoch, 1.0, dataFileName, batch_size, time_steps, modelName)
	else:
		modelName = args.model

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--mode', type=str, default='test')
	parser.add_argument('--model', type=str, default=None)
	parser.add_argument('--data', type=str, default=None)

	args = parser.parse_args()

	main(args)



