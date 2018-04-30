import tensorflow as tf
import numpy as np
import scipy.io as sio
import sys
import os
import pickle
from matplotlib import pyplot as plt
import argparse
from logger import Logger
from RNN import RNNNetwork

def loadRawData(fileName):
	data=pickle.load(open(fileName,'rb'))

	#Uploading the data into xk_data and uk_data
	X_and_U = np.zeros([10, 1000, 5])

	for row in range(len(data)):
		firstIndex = row//1000
		secondIndex = row%1000
		X_and_U[firstIndex, secondIndex, :4] = data[row][0]
		X_and_U[firstIndex, secondIndex, 4:] = data[row][1]

	return X_and_U

def main(args):
	# Network parameters
	time_steps = 3
	batch_size = 1024
	input_state_size = 5 # [sita1 w1 sita2 w2 torque]_t
	output_state_size = 4 # [sita1 w1 sita2 w2]_t+1
	hidden_state_size = 8
	num_epoch = 100000
	lrn_rate = 1e-4
	
	# Training data file 	
	dataFileName = args.data

	# Model path
	modelPath = './model/'
	if not os.path.exists(modelPath):
		os.makedirs(modelPath)

	network = RNNNetwork(lrn_rate, input_state_size, hidden_state_size, output_state_size)
	if(args.mode=='train'):
		with tf.Session() as sess:
			network.train(sess, num_epoch, dataFileName, batch_size, time_steps)
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



