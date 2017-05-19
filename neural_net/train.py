#!/usr/bin/env python
from __future__ import division, print_function

from data_handler import *
from net import *

from math import sqrt, exp
import numpy as np
import rospy
import sys
import tensorflow as tf
import time

# Training parameter.
learning_rate = 0.0001
eps_regularization = 0.01
batch_size = 64
training_iters = 2000
test_iterations = 64
display_step = 10
# Network parameter.
rospy.init_node('convolutional network')
image_width = rospy.get_param('/cone/width_pixel')
image_height = rospy.get_param('/cone/height_pixel')
# Datasets.
path_to_directory = rospy.get_param('/candidate_path')
path_to_model = rospy.get_param('/model_path')
datasets = rospy.get_param('/neural_net/datasets')
datasets_test = rospy.get_param('/neural_net/datasets_test')

#Data Handler.
data = DataHandler(image_height,image_width,batch_size,test_iterations, 
                   path_to_directory, path_to_model, datasets, datasets_test)
# tf Graph placeholder.
input_placeholder = tf.placeholder(tf.float32, [None, image_height, image_width, 3])
output_placeholder = tf.placeholder(tf.float32, [None, 2])
input_placeholder_flat = tf.contrib.layers.flatten(input_placeholder)
y_true = tf.argmax(output_placeholder, dimension=1)

# Construct model.
output_layer = fully_connected(input_placeholder_flat, eps_regularization)
y_pred = tf.argmax(tf.nn.softmax(output_layer), dimension=1)
#Cost and optimizer.
cross_entropy = tf.nn.softmax_cross_entropy_with_logits(logits=output_layer,labels=output_placeholder)
cost = tf.reduce_mean(cross_entropy)
optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)
#Prediction and accuracy.
correct_prediction = tf.equal(y_pred, y_true)
accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

# Initializing the variables
session = tf.Session()
session.run(tf.global_variables_initializer())
saver = tf.train.Saver()

#Start-time used for printing time-usage below.
start_time = time.time()
for i in range(training_iters):
    #Get a batch of training examples.
    x_batch, y_batch = data.getBatch()
    #Optimizing net according to batch.
    feed_dict_train = {input_placeholder: x_batch,
                       output_placeholder: y_batch}
    session.run(optimizer, feed_dict=feed_dict_train)
    #Calculate the accuracy status.
    if i % display_step == 0:
        x_training_test, y_training_test = data.getTrainingTestBatch()
        feed_dict_training_test = {input_placeholder: x_training_test,
                                   output_placeholder: y_training_test}
        x_test, y_test = data.getTestBatch()
        feed_dict_test = {input_placeholder: x_test,
                          output_placeholder: y_test}
        loss_batch, acc_batch = session.run([cost, accuracy], feed_dict=feed_dict_train)
        acc_training = session.run(accuracy, feed_dict=feed_dict_training_test)
        acc_test = session.run(accuracy, feed_dict=feed_dict_test)
        msg = "Optimization Iteration: %f, Accuracy on batch: %f, Accuracy on training data: %f, Accuracy on test data : %f"
        print(msg % (i+1, acc_batch, acc_training, acc_test))
        data.saveAccuracyAndLoss(i+1, acc_batch, acc_training, acc_test, loss_batch)
#Print training time.
end_time = time.time()
time_dif = end_time - start_time
print("Time usage: " + str(time_dif))
#Testing accuracy on training data set.
x_training_test, y_training_test = data.getTrainingTestBatch()
feed_dict_training_test = {input_placeholder: x_training_test,
                           output_placeholder: y_training_test}
x_test, y_test = data.getTestBatch()
feed_dict_test = {input_placeholder: x_test,
                  output_placeholder: y_test}
acc_training = session.run(accuracy, feed_dict=feed_dict_training_test)
acc_test = session.run(accuracy, feed_dict=feed_dict_test)
print("Accuracy at all on training: %f and test: %f" % (acc_training, acc_test))
#Plot accuracy plot.
data.plotAccuracyPlot()
#Save model.
save = saver.save(session, path_to_model + getModelName(datasets) +" .cpkt")
print("Model saved to " + path_to_model + getModelName(datasets))