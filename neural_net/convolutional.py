#!/usr/bin/env python
from __future__ import division, print_function

from data_handler import *
from net import *

from math import sqrt, exp
import numpy as np
import rospy
import sys
import tensorflow as tf

# Training parameter.
learning_rate = 0.00001
dropout = 0.75 
batch_size = 128
training_iters = 200000
test_iterations = 200
display_step = 10
decrease_rate_step = 500
reload_model = False
equalising = True
# Network parameter.
rospy.init_node('convolutional network')
image_width = rospy.get_param('/cone/width_pixel')
image_height = rospy.get_param('/cone/height_pixel')
# Datasets.
path_to_directory = rospy.get_param('/candidate_path')
path_to_model = rospy.get_param('/model_path')
datasets = rospy.get_param('/neural_net/datasets')

#Data Handler.
data = DataHandler(image_height,image_width,batch_size,test_iterations, 
                   path_to_directory, path_to_model, datasets, equalising)

# tf Graph input.
X = tf.placeholder(tf.float32, [None,image_height,image_width,3])
Y = tf.placeholder(tf.float32, [None,2])
rate = tf.placeholder(tf.float32, shape=[])
keep_prob = tf.placeholder(tf.float32)

# Construct model.
pred = conv_without_contrib(X, image_height, image_width, 2, keep_prob)
# Define loss and optimizer
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=pred, labels=Y))
optimizer = tf.train.AdamOptimizer(learning_rate=rate).minimize(cost)
# Evaluate model
correct_pred = tf.equal(tf.argmax(pred, 1), tf.argmax(Y, 1))
accuracy = tf.reduce_mean(tf.cast(correct_pred, tf.float32))
# Initializing the variables
init = tf.global_variables_initializer()
saver = tf.train.Saver()

# Launch the graph
with tf.Session() as sess:
    sess.run(init)
    step = 1
    # Training network.
    if(reload_model): 
        saver.restore(sess, path_to_model + getModelName(datasets) +" .cpkt")
        print("\nRestored model ! \n")
    else:
        print("\nOptimizing with %f steps and batch size %f" % (training_iters, batch_size))
        while (step*batch_size < training_iters):
            batch_x, batch_y = data.getBatch()
            sess.run(optimizer, feed_dict={X: batch_x, Y: batch_y, 
                                          rate:learning_rate, keep_prob: dropout})
            step += 1
            # Display progress.
            if(step%display_step == 0): 
                # Calculate batch loss and accuracy
                loss, acc = sess.run([cost, accuracy], feed_dict={X: batch_x, Y: batch_y, 
                                                                  keep_prob: 1.0})
                print("Iter " + str(step*batch_size) + ", Minibatch Loss= " + \
                      "{:.6f}".format(loss) + ", Training Accuracy= " + \
                      "{:.5f}".format(acc))
            #Decrease learning rate.
            if(step%decrease_rate_step == 0): 
                learning_rate /= 2.5
                print("Decreasing learning rate to %f" % learning_rate)
        save_path = saver.save(sess, path_to_model + getModelName(datasets) +" .cpkt")
        print("Optimization Finished! \n")
    #Testing network.
    test_x, test_y = data.getRandomTestData()
    print("Testing Accuracy:", \
        sess.run(accuracy, feed_dict={X: test_x, Y: test_y, keep_prob: 1.0}))

