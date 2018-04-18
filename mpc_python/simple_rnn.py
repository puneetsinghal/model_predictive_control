import numpy as np
import tensorflow as tf
import pickle
from IPython import embed
import matplotlib.pyplot as plt

PIK = "data_for_rnn_small.dat"

data=pickle.load(open(PIK,'rb'))

def generateData():	
	#Uploading the data into xk_data and uk_data
	xk_data = np.empty([len(data),4])
	uk_data = np.empty([len(data),1])

	for row in range(len(data)):
		x = data[row][0]
		u = data[row][1][0]
		xk_data[row,:] = np.array(x)
		uk_data[row] = np.array(u)

	xk_data = xk_data.T
	uk_data = uk_data.T

	input_data = np.vstack((xk_data,uk_data))
	labels_data = input_data[:,1:]

	return input_data,labels_data
# embed()
# RNN variables
batch_size = 4
truncated_backprop_length = 4
state_size = 4 #Need to change the state size based on our application???
num_classes = 2 #idk the use of this
num_epochs = 100
num_batches = len(data)//batch_size//truncated_backprop_length


batchX_placeholder = tf.placeholder(tf.float32, [5, truncated_backprop_length]) #change this 5 into a variable to make it dynamic
batchY_placeholder = tf.placeholder(tf.int32, [5, truncated_backprop_length])

init_state = tf.placeholder(tf.float32, [5, state_size])

W2 = tf.Variable(np.random.rand(state_size, num_classes),dtype=tf.float32)
b2 = tf.Variable(np.zeros((1,num_classes)), dtype=tf.float32)

inputs_series = tf.split(axis=1, num_or_size_splits=truncated_backprop_length, value=batchX_placeholder)
labels_series = tf.unstack(batchY_placeholder, axis=1)
# Forward passes - hope it works
cell = tf.contrib.rnn.BasicRNNCell(state_size)
# embed()
states_series, current_state =  tf.contrib.rnn.static_rnn(cell, inputs_series, init_state)

logits_series = [tf.matmul(state, W2) + b2 for state in states_series] #Broadcasted addition
predictions_series = [tf.nn.softmax(logits) for logits in logits_series]
# embed()
losses = [tf.nn.sigmoid_cross_entropy_with_logits(logits=logits, labels=labels) for logits, labels in zip(logits_series,labels_series)]
total_loss = tf.reduce_mean(losses)

train_step = tf.train.AdamOptimizer(0.3).minimize(total_loss)

def plot(loss_list, predictions_series, batchX, batchY):
    plt.subplot(2, 3, 1)
    plt.cla()
    plt.plot(loss_list)
    plt.draw()
    plt.pause(0.0001)

with tf.Session() as sess:
    sess.run(tf.initialize_all_variables())
    plt.ion()
    plt.figure()
    plt.show()
    loss_list = []
    x,y = generateData()
    for epoch_idx in range(num_epochs):
        
        _current_state = np.zeros((5, state_size))
        # embed()

        print("New data, epoch", epoch_idx)

        for batch_idx in range(num_batches):
            start_idx = batch_idx * truncated_backprop_length
            end_idx = start_idx + truncated_backprop_length

            batchX = x[:,start_idx:end_idx]
            batchY = y[:,start_idx:end_idx]
            # embed()

            _total_loss, _train_step, _current_state, _predictions_series = sess.run(
                [total_loss, train_step, current_state, predictions_series],
                feed_dict={
                    batchX_placeholder:batchX,
                    batchY_placeholder:batchY,
                    init_state:_current_state
                })

            loss_list.append(_total_loss)

            if batch_idx%100 == 0:
                print("Step",batch_idx, "Loss", _total_loss)
                # plot(loss_list, _predictions_series, batchX, batchY)
# embed()
plt.ioff()
plt.show()
