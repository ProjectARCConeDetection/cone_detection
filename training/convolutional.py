#!/usr/bin/env python
from __future__ import print_function

import csv
from PIL import Image
from math import sqrt, exp
import numpy as np
import os
from random import shuffle
import rospy
import sys
import tensorflow as tf

# Training parameter.
learning_rate = 0.01
training_iters = 200
display_step = 50

# Datasets.
path_to_directory = "/home/sele/candidates"
datasets = ["biberist_evening"]
# Network parameter.
rospy.init_node('convolutional network')
image_width = rospy.get_param('/object/width_pixel')
image_height = rospy.get_param('/object/height_pixel')
n_classes = 2 # Cone or None.

def conv_net(x, weights, biases):
    # Reshape input picture.
    x = tf.reshape(x, shape=[-1,image_height,image_width,1])
    print("x: ", x.get_shape())
    # Convolutional Layer.
    conv1  = tf.nn.conv2d(x, weights['wc1'], strides=[1,1,1,1], padding='SAME')
    conv1 = tf.nn.bias_add(conv1, biases['bc1'])
    conv1 = tf.nn.relu(conv1)
    conv1 = tf.nn.max_pool(conv1, ksize=[1,2,2,1], strides=[1,2,2,1], padding='SAME')
    print("conv1: ", conv1.get_shape())
    conv2  = tf.nn.conv2d(conv1, weights['wc2'], strides=[1,1,1,1], padding='SAME')
    conv2 = tf.nn.bias_add(conv2, biases['bc2'])
    conv2 = tf.nn.relu(conv2)
    conv2 = tf.nn.max_pool(conv2, ksize=[1,2,2,1], strides=[1,2,2,1], padding='SAME')
    print("conv2: ", conv2.get_shape())
    # Fully connected layer.
    fc1 = tf.reshape(conv2, [-1, weights['wd1'].get_shape().as_list()[0]])
    fc1 = tf.add(tf.matmul(fc1, weights['wd1']), biases['bd1'])
    fc1 = tf.nn.relu(fc1)
    print("fc1: ", fc1.get_shape())
    # Output, class prediction
    out = tf.add(tf.matmul(fc1, weights['out']), biases['out'])
    return out

def getImage(path):
    img = Image.open(path)
    arr = np.array(img.getdata(),
                   np.uint8).reshape(img.size[1], img.size[0], 3)
    return arr

def getLabels(path, dataset):
    labeled_list = []
    reader = csv.reader(open(path))
    for row in reader:
        image = int(row[0])
        label = int(row[1])
        location = dataset
        labeled_list.append([image, label, dataset])
    return labeled_list

def getTrainingData():
    train_X = []
    train_Y = []
    labeled_list = []
    # Getting labeled list.
    for dataset in datasets:
        path = os.path.join(path_to_directory, dataset)
        labeled_list.append(getLabels(path + "/labeling.csv", dataset))
    # Shuffle list.
    labeled_list = labeled_list[0]
    shuffle(labeled_list)
    # Getting training data.
    for data in labeled_list:
        path = path_to_directory + "/" + data[2] + "/" + str(data[0]) + ".jpg"
        img = getImage(path)
        label = data[1]
        train_X.append(img)
        if label: 
            train_Y.append(np.array([1,0]))
        else: 
            train_Y.append(np.array([0,1]))
    return train_X, train_Y

# Training data.
train_X, train_Y = getTrainingData()

# tf Graph input.
X = tf.placeholder(tf.float32, [image_height,image_width,3])
# Y = tf.placeholder(tf.float32, [n_classes,])
Y = tf.placeholder(tf.float32, [1,n_classes])

# Store layers weight & bias.
weights = {
    'wc1': tf.Variable(tf.random_normal([5, 5, 1, 32])),
    'wc2': tf.Variable(tf.random_normal([5, 5, 32, 64])),
    'wd1': tf.Variable(tf.random_normal([64, 1024])),
    'out': tf.Variable(tf.random_normal([1024, n_classes]))
}

biases = {
    'bc1': tf.Variable(tf.random_normal([32])),
    'bc2': tf.Variable(tf.random_normal([64])),
    'bd1': tf.Variable(tf.random_normal([1024])),
    'out': tf.Variable(tf.random_normal([n_classes]))
}

# Construct model.
pred = conv_net(X, weights, biases)
print("pred: ", pred.get_shape())
print("Y: ", Y.get_shape())
# Define loss and optimizer
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=pred, labels=Y))
optimizer = tf.train.AdamOptimizer(learning_rate).minimize(cost)
# Define Accuracy.
correct_pred = tf.equal(tf.argmax(pred, 1), tf.argmax(Y, 1))
accuracy = tf.reduce_mean(tf.cast(correct_pred, tf.float32))
# Initializing the variables
init = tf.global_variables_initializer()

# Launch the graph
with tf.Session() as sess:
    sess.run(init)
    step = 1
    # Training network.
    print("Optimizing")
    while (step < training_iters):
        batch_x, batch_y = [train_X[step], train_Y[step]]
        sess.run(optimizer, feed_dict={X: batch_x, Y: batch_y})
        # if step % display_step == 0:
        #     # Calculate batch loss and accuracy
        #     loss, acc = sess.run([cost, accuracy], feed_dict={X: batch_x,Y: batch_y})
        #     print("Iter " + str(step*batch_size) + ", Minibatch Loss= " + \
        #           "{:.6f}".format(loss) + ", Training Accuracy= " + \
        #           "{:.5f}".format(acc))
        step += 1
    # print("\n Optimization Finished!")