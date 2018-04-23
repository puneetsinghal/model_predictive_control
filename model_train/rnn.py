import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

num_steps = 5 # Truncated steps
batch_size = 200
num_classes = 2
state_size = 4 # Hidden state size
learning_rate = 0.1

def gen_data(size=100000):
    X = np.array(np.random.choice(2, size=(size,)))
    Y = []
    for i in range(size):
        threshold = 0.5
        if X[i-3] == 1:
            threshold += 0.5
        if X[i-8] == 1:
            threshold -= 0.25
        # Binary output with probability threshold
        if np.random.rand() > threshold:
            Y.append(0)
        else:
            Y.append(1)
    return X, np.array(Y)


def gen_batch(raw_data, batch_size, num_steps):
    raw_x, raw_y = raw_data
    data_length = len(raw_x)

    # partition raw data into batch and stack them up
    batch_partition_length = data_length // batch_size
    data_x = np.zeros([batch_size, batch_partition_length], dtype=np.int32)
    data_y = np.zeros([batch_size, batch_partition_length], dtype=np.int32)
    
    for i in range(batch_size):
        data_x[i] = raw_x[batch_partition_length*i : batch_partition_length*(i+1)]
        data_y[i] = raw_y[batch_partition_length*i : batch_partition_length*(i+1)]
    print(np.array(data_x).shape) 
    epoch_size = batch_partition_length // num_steps
    print(epoch_size)
    for i in range(epoch_size):
        x = data_x[:, i*num_steps : (i+1)*num_steps]
        y = data_y[:, i*num_steps : (i+1)*num_steps]
        yield (x,y)

def gen_epochs(n, num_steps):
    data = []
    for i in range(n):
        data.append(gen_batch(gen_data(), batch_size, num_steps))
    return data 

# Model 
x = tf.placeholder(tf.int32, [batch_size, num_steps], name='input_placeholder')
y = tf.placeholder(tf.int32, [batch_size, num_steps], name='output_placeholder')
init_state = tf.zeros([batch_size, state_size])

# rnn_inputs
x_one_hot = tf.one_hot(x, num_classes)

print(x_one_hot)
rnn_inputs = tf.unstack(x_one_hot, axis=1)


# Define rnn_cell
with tf.variable_scope('rnn_cell'):
    W = tf.get_variable('W', [num_classes + state_size, state_size])
    b = tf.get_variable('b', [state_size], initializer=tf.constant_initializer(0.0))


def rnn_cell(rnn_input, state):
    with tf.variable_scope('rnn_cell', reuse=True):
        W = tf.get_variable('W', [num_classes + state_size, state_size])
        b = tf.get_variable('b', [state_size], initializer=tf.constant_initializer(0.0))
    return tf.tanh(tf.matmul(tf.concat([rnn_input, state], 1), W) + b)

state = init_state
rnn_outputs = []
for rnn_input in rnn_inputs:
    state = rnn_cell(rnn_input, state)
    rnn_outputs.append(state)
final_state = rnn_outputs[-1]

with tf.variable_scope('softmax'):
    W = tf.get_variable('W', [state_size, num_classes])
    b = tf.get_variable('b', [num_classes], initializer=tf.constant_initializer(0.0))
logits = [tf.matmul(rnn_output, W) + b for rnn_output in rnn_outputs]
predictions = [tf.nn.softmax(logit) for logit in logits]

# Turn our y placeholder into a list of labels
y_as_list = tf.unstack(y, num=num_steps, axis=1)

# Define loss and optimizer
losses = [tf.nn.sparse_softmax_cross_entropy_with_logits(labels=label, logits=logit) for \
        logit, label in zip(logits, y_as_list)]

total_loss = tf.reduce_mean(losses)
train_step = tf.train.AdagradOptimizer(learning_rate).minimize(total_loss)

def train_network(num_epochs, num_steps, state_size=4, verbose=True):
    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())
        training_losses = []
        for idx, epoch in enumerate(gen_epochs(num_epochs, num_steps)):
            print(epoch)
            training_loss = 0
            training_state = np.zeros((batch_size, state_size))
            if verbose:
                print("\nEPOCH", idx)
            for step, (X,Y) in enumerate(epoch):
                tf_losses, training_loss_, training_state, _ = \
                        sess.run([losses, total_loss, final_state, train_step], \
                        feed_dict={x:X, y:Y, init_state:training_state})
                training_loss += training_loss_
                if step % 100 == 0 and step > 0:
                    if verbose:
                        print("Average loss at step", step, "for last 250 steps:", training_loss/100)
                        training_losses.append(training_loss/100)
                        training_loss = 0

    return training_losses

training_losses = train_network(1, num_steps)
plt.plot(training_losses)
# plt.show()













