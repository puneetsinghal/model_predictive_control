import tensorflow as tf
import numpy as np
import scipy.io as sio
import sys

def load_matfile(matfile):
    # Load matlab mat file 
    data = sio.loadmat(matfile)
    input_data = data['Input_data']
    output_data = data['Output_data']
    return input_data, output_data

# Store data
class Data:
    def __init__(self,input_data, output_data):
        self.input_data = input_data
        self.output_data = output_data
    def get_batch(self, batch_size):
      
        index = np.random.choice(len(self.input_data), batch_size)
        input_batch = [self.input_data[idx] for idx in index]
        output_batch = [self.output_data[idx] for idx in index]
        return input_batch, output_batch


# Acrobot network
class Network:
    def __init__(self, lrn_rate=0.001, total_epoch=1000, batch_size=256, network_type='Feedforward', \
            model_path=None, log_path=None):
        self.lrn_rate = lrn_rate
        self.total_epoch = total_epoch
        self.batch_size = batch_size
        self.model_path = model_path
        
        # build the network 
        if network_type == 'Feedforward':
            self.build_feedforward_network()
        else:
            print('No such type of network')
            sys.exit(0)
        self.init = tf.global_variables_initializer()
        self.saver = tf.train.Saver()
        self.summary_writer = tf.summary.FileWriter(log_path,tf.get_default_graph())
        
        # count for loss record used in tensorboard
        self.count = 0

    def build_feedforward_network(self):
        # Two hidden layers feedforward network
        # Build the computation graph
        # Input [sita1 sita2 w1 w2 tao] of t
        # Output [sita1 sita2 w1 w2] of t+1
        self.input_dim = 5
        self.output_dim = 4
        
        hidden_units = [32,32]
        
        print("build feedforward network with hidden units", hidden_units[0], hidden_units[1])
        self.input_ph = tf.placeholder(tf.float32, shape=[None, self.input_dim], name="input")
        self.output_ph = tf.placeholder(tf.float32, shape=[None, self.output_dim], name="output")
        
        with tf.name_scope('dense_layers'):
            self.hidden_layer1 = tf.layers.dense(self.input_ph, hidden_units[0], activation = tf.nn.relu)
            self.hidden_layer2 = tf.layers.dense(self.hidden_layer1, hidden_units[1], activation = tf.nn.relu)
        with tf.name_scope('prediction'):
            self.prediction = tf.layers.dense(self.hidden_layer2, self.output_dim, activation = None)
        
        with tf.name_scope('loss'):
            # Define loss
            self.loss = tf.losses.mean_squared_error(self.output_ph, self.prediction)
        with tf.name_scope('optimize'):
            # Step of optimization
            self.train_step = tf.train.AdamOptimizer(learning_rate = self.lrn_rate).minimize(self.loss)

    def load_model_weights(self, sess):
        self.saver.restore(sess, self.model_path)
    
    def save_model_weights(self, sess):
        self.saver.save(sess, self.model_path)
    
    def train(self, sess, train_file, reload_model = False):
        # Train our neural network
        input_data, output_data = load_matfile(train_file)
        data = Data(input_data, output_data)
        if reload_model is True:
            self.load_model_weights(sess)
        else:
            sess.run(self.init)
        
        for epoch in range(self.total_epoch):
            input_batch, output_batch = data.get_batch(self.batch_size)
            feed_dict = {self.input_ph:input_batch, self.output_ph:output_batch}
            cur_loss, _ = sess.run([self.loss, self.train_step], feed_dict=feed_dict)
            if epoch % 1000 == 0:
                print("epoch",epoch,"loss",cur_loss)
                summary = tf.Summary(value=[tf.Summary.Value(tag="training_loss",simple_value=cur_loss)])
                self.summary_writer.add_summary(summary, self.count)
                self.count += 1
            if epoch % 10000 == 0:
                self.save_model_weights(sess)

        print("Finish training")
        self.save_model_weights(sess)
    
    
    def test(self, sess, test_file, reload_model = False):
        # Test the results of the training of our neural network
        test_input_data, test_output_data = load_matfile(test_file)

        # If we want to test the model only and not train
        if reload_model is True:
            self.load_model_weights(sess)
         
        feed_dict = {self.input_ph:test_input_data, self.output_ph:test_output_data}
        test_loss = sess.run(self.loss, feed_dict=feed_dict)
        print("test loss:",test_loss)
    
    def predict_next_state(self, sess, prev_state, control_input):
        # Predict the next state based on our trained neural network
        # Make sure the state and control input both are shape ([m, n_s]) and ([m, n_a])
        input_vector = np.hstack((prev_state, control_input))
        feed_dict = {self.input_ph:input_vector}
        return sess.run(self.prediction, feed_dict=feed_dict)
    

