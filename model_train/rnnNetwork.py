import tensorflow as tf
import numpy as np
import scipy.io as sio
import sys
import os

def load_matfile(matfile):
    # Load matlab mat file
    data = sio.loadmat(matfile)
    input_data = data['Input_data']
    output_data = data['Output_data']
    return input_data, output_data

def gen_epoch(data_file, batch_size, time_steps, input_state_size, output_state_size):
    input_data, output_data = load_matfile(data_file)
    data_size = input_data.shape[0]    
    num_batch = data_size // (batch_size*time_steps)
    
    num_examples = data_size // time_steps
    
    input_examples = np.zeros((num_examples, time_steps, input_state_size))
    output_examples = np.zeros((num_examples, time_steps, output_state_size))
    for i in range(num_examples):
        input_examples[i] = input_data[i*time_steps : (i+1)*time_steps]
        output_examples[i] = output_data[i*time_steps : (i+1)*time_steps]

    input_epoch = np.zeros((num_batch, batch_size, time_steps, input_state_size))
    output_epoch = np.zeros((num_batch, batch_size, time_steps, output_state_size))
   
    for i in range(num_batch):
        input_epoch[i] = input_examples[i*batch_size : (i+1)*batch_size]
        output_epoch[i] = output_examples[i*batch_size : (i+1)*batch_size]

    return input_epoch, output_epoch, num_batch


class RNNNetwork():
    def __init__(self, lrn_rate, input_state_size, hidden_state_size, output_state_size, model_path):
        self.lrn_rate = lrn_rate
        self.input_state_size = input_state_size
        self.output_state_size = output_state_size
        self.model_path = model_path
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
        cell = tf.contrib.rnn.LSTMCell(hidden_state_size, state_is_tuple=True)
        init_states = cell.zero_state(self.network_batch_size, tf.float32)
        rnn_outputs, final_state = tf.nn.dynamic_rnn(cell, self.prev_state, initial_state=init_states)
        outputs = tf.reshape(rnn_outputs, [-1, hidden_state_size])
        
        # Combine LSTM output with control usage and current state
        W = tf.get_variable('W', [hidden_state_size, output_state_size])
        b = tf.get_variable('b', [output_state_size])
        predictions = tf.matmul(outputs, W) + b
        
        """
        state = tf.reshape(self.current_state, [-1, input_state_size])
        W2 = tf.get_variable('W2', [input_state_size, output_state_size])
        b2 = tf.get_variable('b2', [output_state_size])
        predictions = (tf.matmul(outputs, W1) + b1) + (tf.matmul(state, W2) + b2)
        """

        
        self.loss = tf.losses.mean_squared_error(
                predictions, tf.reshape(self.future_state, [-1, output_state_size]))
       
        self.train_step = tf.train.AdamOptimizer(learning_rate=self.lrn_rate).minimize(self.loss)
        
        self.final_prediciton = tf.matmul(final_state[1], W) + b
    
        self.saver = tf.train.Saver()

    def train(self, sess, num_epoch, data_file, batch_size, time_steps, model=None):
        input_epoch, output_epoch, num_batch = gen_epoch(data_file, batch_size, 
                time_steps, self.input_state_size, self.output_state_size)
        
        save_interval = 100
        if model is None:
            # Initialize network
            sess.run(tf.global_variables_initializer())
        else:
            self.load_model_weights(sess, model)
        for epoch in range(num_epoch):
            for batch in range(num_batch):
                train_loss, _  = sess.run([self.loss, self.train_step], feed_dict={ 
                    self.network_batch_size:batch_size,
                    self.prev_state:input_epoch[batch], self.future_state:output_epoch[batch]})
            if epoch % save_interval == 0:
                print("epoch", epoch, "loss", train_loss)
                self.save_model_weights(sess, self.model_path + '_' + str(epoch) + '.cpkt')
        # Finish training, save final model
        self.save_model_weights(sess, self.model_path + '_final.cpkt')

    
    def predict(self, sess, prev_state):
        
        return sess.run([self.final_prediction], 
                feed_dict={self.prev_state:prev_state, self.network_batch_size:1})
    
    def save_model_weights(self, sess, file_path):
        self.saver.save(sess, file_path)
    
    def load_model_weights(self, sess, file_path):
        self.saver.restore(sess, file_path)

def main():
    # Network parameters
    time_steps = 5
    batch_size = 200
    num_batch = 10
    input_state_size = 5 # [sita1 w1 sita2 w2 torque]_t
    output_state_size = 4 # [sita1 w1 sita2 w2]_t+1
    hidden_state_size = 10
    num_epoch = 100000
    lrn_rate = 1e-4
    
    # Training data file
    train_file = 'train_data.mat'
 
    # Model path
    model_path = './models/'
    if not os.path.exists(model_path):
        os.makedirs(model_path)

    network = RNNNetwork(lrn_rate, input_state_size, hidden_state_size, output_state_size, model_path)
    with tf.Session() as sess:
        network.train(sess, num_epoch, train_file, batch_size, time_steps)
    
    
    """
    def train(self, num_epoch, data_file, batch_size, time_steps):
        input_epoch, output_epoch = gen_epoch('train_data.mat', batch_size, 
                time_steps, input_state_size, output_state_size)
        print(num_batch)
        with tf.Session() as sess:
            # Initialize network
            sess.run(tf.global_variables_initializer())
            for epoch in range(num_epoch):
                for batch in range(num_batch):
                    train_loss, _  = sess.run([self.loss, self.train_step], feed_dict={ 
                        network_batch_size:batch_size,
                        self.prev_state:input_epoch[batch], self.future_state:output_epoch[batch]})
                if epoch % 100 == 0:
                    print("epoch", epoch, "loss", train_loss)
    """
    """
    # Build computational graph
    network_batch_size = tf.placeholder(tf.int32, [])

    rnn_inputs = tf.placeholder(tf.float32, [None, None, input_state_size])
    actual_state = tf.placeholder(tf.float32, [None, None, output_state_size])

    #cell = tf.contrib.rnn.BasicRNNCell(hidden_state_size)
    cell = tf.contrib.rnn.LSTMCell(hidden_state_size, state_is_tuple=True)
    init_states = cell.zero_state(network_batch_size, tf.float32)
    rnn_outputs, final_state = tf.nn.dynamic_rnn(cell, rnn_inputs, initial_state=init_states)
    print('rnn outputs', rnn_outputs)

    outputs = tf.reshape(rnn_outputs, [-1, hidden_state_size])
    print(outputs)


    W = tf.get_variable('W', [hidden_state_size, output_state_size])
    b = tf.get_variable('b', [output_state_size])
    predictions = tf.matmul(outputs, W) + b
    print(predictions)
    print(actual_state)
    loss = tf.losses.mean_squared_error(predictions, tf.reshape(actual_state, [-1, output_state_size]))
    print(loss)

    train_step = tf.train.AdamOptimizer(learning_rate=lrn_rate).minimize(loss)

    print(final_state[1])

    final_prediction = tf.matmul(final_state[1], W) + b

    input_epoch, output_epoch = gen_epoch('train_data.mat', batch_size, time_steps, input_state_size, output_state_size)
    print(num_batch)
    with tf.Session() as sess:
        # Initialize network
        sess.run(tf.global_variables_initializer())
        for epoch in range(num_epoch):
            for batch in range(num_batch):
                train_loss, _  = sess.run([loss, train_step], feed_dict={ \
                        network_batch_size:batch_size, \
                        rnn_inputs:input_epoch[batch], \
                        actual_state:output_epoch[batch]})
            if epoch % 100 == 0:
                print("epoch", epoch, "loss", train_loss)
    """

if __name__ == '__main__':
    main()



