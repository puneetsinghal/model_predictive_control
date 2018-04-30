import tensorflow as tf
import numpy as np
from matplotlib import pyplot as plt
from logger import Logger
from data_processor import DB_Processor

def make_log_dir(log_parent_dir):
	import datetime, os
	current_timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
	return current_timestamp

class RNNNetwork(object):
	def __init__(self, lrn_rate, input_state_size, hidden_state_size, output_state_size):
		self.lrn_rate = lrn_rate
		self.input_state_size = input_state_size
		self.output_state_size = output_state_size
		# self.model_path = model_path

		# Build network
		# Inputs
		"""
		# Current state x_t (for
		self.current_state = tf.placheholder(tf.float32, [None, None, input_state_size])
		"""
		# History
		self.prev_state = tf.placeholder(tf.float32, [None, None, input_state_size])
		# The actual outout that is used to compare with prediction
		self.future_state = tf.placeholder(tf.float32, [None, None, output_state_size])
		# Network batch size as place holder so can input 1 during runtime
		self.network_batch_size = tf.placeholder(tf.int32, [])
		
		# LSTM cell
		cell = tf.contrib.rnn.LSTMCell(hidden_state_size, state_is_tuple=True, activation=tf.sigmoid)
		init_states = cell.zero_state(self.network_batch_size, tf.float32)
		rnn_outputs, final_state = tf.nn.dynamic_rnn(cell, self.prev_state, initial_state=init_states)
		outputs = tf.reshape(rnn_outputs, [-1, hidden_state_size])
		
		# Combine LSTM output with control usage and current state
		W = tf.get_variable('W', [hidden_state_size, output_state_size])
		b = tf.get_variable('b', [output_state_size])
		predictions = tf.matmul(outputs, W) + b
		
		self.loss = tf.losses.mean_squared_error(predictions,
							tf.reshape(self.future_state, [-1, output_state_size]))
	   
		self.optimizer = tf.train.AdamOptimizer(learning_rate=self.lrn_rate)
		self.trainer = self.optimizer.minimize(self.loss)
		
		self.final_prediciton = tf.matmul(final_state[1], W) + b
	
		self.saver = tf.train.Saver()

	def train(self, sess, num_epoch, data_file, batch_size, time_steps, model=None):
		input_epoch, output_epoch, num_batch = DB_Processor.gen_epoch(data_file, batch_size, time_steps, self.input_state_size, self.output_state_size)
		
		log_dir 	= make_log_dir('')
		self.logger 		= Logger('./' + log_dir + '/train_log/')
		self.model_path 	= './' + log_dir + '/model'

		save_interval = 100
		if model is None:
			# Initialize network
			sess.run(tf.global_variables_initializer())
		else:
			self.load_model_weights(sess, model)
		for epoch in range(num_epoch):
			# for batch in range(num_batch):
			batch = np.random.choice(num_batch, batch_size)
			feed_dict = {self.network_batch_size:batch_size, 
							self.prev_state:input_epoch[batch], self.future_state:output_epoch[batch]}
			train_loss, _  = sess.run([self.loss, self.trainer], feed_dict= feed_dict)
				
			if epoch % save_interval == 0:
				print("epoch", epoch, "loss", train_loss)
				self.save_model_weights(sess, self.model_path + '_' + str(epoch) + '.cpkt')
				self.logger.log_scalar(tag='Loss',value=train_loss, step=epoch)
		
		# Finish training, save final model
		self.save_model_weights(sess, self.model_path + '_final.cpkt')

	def test(self, sess, num_epoch, data_file, batch_size, time_steps, modelPath=None):
		# batch_size = None
		input_epoch, output_epoch, num_batch = DB_Processor.gen_epoch(data_file, batch_size, time_steps, self.input_state_size, self.output_state_size)

		self.load_model_weights(sess, modelPath)

		numTestData = 10000

		testLoss = np.zeros(numTestData)

		for i in range(numTestData):
			batch = np.random.choice(num_batch, 100)

			feed_dict = {self.network_batch_size:100, 
							self.prev_state:input_epoch[batch], self.future_state:output_epoch[batch]}
			testLoss[i] = sess.run([self.loss], feed_dict= feed_dict)[0]
		
		print("average error is {}".format(np.mean(testLoss)))
		plt.figure()
		plt.plot(testLoss)
		plt.show()

	def predict(self, sess, prev_state):
		feed_dict = {self.prev_state:prev_state, self.network_batch_size:1}
		return sess.run([self.final_prediction], feed_dict=feed_dict)
	
	def save_model_weights(self, sess, file_path):
		self.saver.save(sess, file_path)
	
	def load_model_weights(self, sess, modelPath):
		# self.saver.restore(sess, file_path)
		self.saver = tf.train.import_meta_graph(modelPath + 'model_final.cpkt.meta')
		# savePath = './model/' #+ args.model + '_NN/'
		self.saver.restore(sess, tf.train.latest_checkpoint(modelPath))
		print("Model restored.")
