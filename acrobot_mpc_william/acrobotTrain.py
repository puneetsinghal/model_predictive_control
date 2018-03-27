import tensorflow as tf
import numpy as np
import scipy.io as sio



def load_matfile(matfile):
    data = sio.loadmat(matfile)
    input_data = data['Input_data']
    output_data = data['Output_data']
    return input_data, output_data


class Data:
    def __init__(self,input_data, output_data):
        self.input_data = input_data
        self.output_data = output_data
    def get_batch(self, batch_size):
      
        index = np.random.choice(len(input_data), batch_size)
        input_batch = [self.input_data[idx] for idx in index]
        output_batch = [self.output_data[idx] for idx in index]
        return input_batch, output_batch

if __name__ == '__main__':
    # Build the computation graph
    # Input [sita1 sita2 w1 w2 tao] of t
    # Output [sita1 sita2 w1 w2] of t+1
    
    train_file = 'data.mat'
    input_data, output_data = load_matfile(train_file)
    data = Data(input_data, output_data)
    data.get_batch(10)
    input_dim = 5
    output_dim = 4
    lrn_rate = 0.0001
    total_epoch = 100000
    batch_size = 256
    
    

    input_ph = tf.placeholder(tf.float32, shape=[None, input_dim], name="input")
    output_ph = tf.placeholder(tf.float32, shape=[None, output_dim], name="output")
    hidden_layer1 = tf.layers.dense(input_ph, 32, activation = tf.nn.relu)
    hidden_layer2 = tf.layers.dense(hidden_layer1, 32, activation = tf.nn.relu)
    
    prediction = tf.layers.dense(hidden_layer2, output_dim, activation = None)
    loss = tf.losses.mean_squared_error(output_ph, prediction)
    train_step = tf.train.AdamOptimizer(learning_rate = lrn_rate).minimize(loss)
    init = tf.global_variables_initializer()
    
    saver = tf.train.Saver()
    summary_writer = tf.summary.FileWriter('./logs',tf.get_default_graph())
    count = 0
    with tf.Session() as sess:
        sess.run(init)
        for epoch in range(total_epoch):
            input_batch, output_batch = data.get_batch(batch_size)
            feed_dict = {input_ph:input_batch,output_ph:output_batch}
            cur_loss, _ = sess.run([loss, train_step],feed_dict=feed_dict)
            if epoch % 1000 == 0:
                print "epoch :",epoch,"\tloss:",cur_loss
                summary = tf.Summary(value=[tf.Summary.Value(tag="training_loss",simple_value=cur_loss)])
                summary_writer.add_summary(summary, count)
                count += 1
        
        print("Finish training")
        saver.save(sess,"./model/model.cpkt")





