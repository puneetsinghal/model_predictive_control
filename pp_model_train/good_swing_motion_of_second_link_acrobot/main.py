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
	return current_timestamp

def main(args):
	# Network parameters
	time_steps = 5
	batch_size = 1024
	input_state_size = 5 # [sita1 w1 sita2 w2 torque]_t
	output_state_size = 4 # [sita1 w1 sita2 w2]_t+1
	hidden_state_size = 16
	num_epoch = 100000
	lrn_rate = 1e-4
	
	# Training data file 	
	dataFileName = args.data

	# Model path
	modelPath = './model/'
	

	network = RNNNetwork(lrn_rate, input_state_size, hidden_state_size, output_state_size)
	if(args.mode=='train'):
		log_dir 	= make_log_dir('')
		if not os.path.exists(log_dir):
			os.makedirs(log_dir)

		shutil.copyfile('RNN.py', (log_dir + '/RNN.py'))  
		shutil.copyfile('main.py', (log_dir + '/main.py'))  
		shutil.copyfile('logger.py', (log_dir + '/logger.py')) 
		shutil.copyfile('testDynamics.py', (log_dir + '/testDynamics.py'))

		with tf.Session() as sess:
			network.train(sess, num_epoch, dataFileName, batch_size, time_steps, log_dir)
	elif(args.mode=='test'):
		modelName = args.model
		with tf.Session() as sess:
			network.test(sess, num_epoch, dataFileName, batch_size, time_steps, modelName)
	else:
		modelName = args.model

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--mode', type=str, default='test')
	parser.add_argument('--model', type=str, default=None)
	parser.add_argument('--data', type=str, default=None)

	args = parser.parse_args()

	main(args)



