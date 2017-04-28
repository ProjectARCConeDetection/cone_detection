#!/usr/bin/env python
from __future__ import division, print_function

from net import *

import csv
from PIL import Image
from math import sqrt, exp
import numpy as np
import os
from random import randint, shuffle
import rospy
import sys
import tensorflow as tf

# Training parameter.
learning_rate = 0.01
training_iters = 10000
display_step = training_iters/5
reload_model = False

# Network parameter.
rospy.init_node('convolutional network')
image_width = rospy.get_param('/cone/width_pixel')
image_height = rospy.get_param('/cone/height_pixel')
# Datasets.
path_to_directory = rospy.get_param('/candidate_path')
path_to_model = rospy.get_param('/model_path')
datasets = rospy.get_param('/neural_net/datasets')

def getBatch(train_X, train_Y):
    length = len(train_X)-1
    i = randint(0,length)
    return train_X[i], train_Y[i]

def getImageFromPath(path):
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
    positive = 0; negative = 0;
    for data in labeled_list:
        path = path_to_directory + data[2] + "/" + str(data[0]) + ".jpg"
        img = getImageFromPath(path)
        label = data[1]
        train_X.append(img)
        if label: 
            train_Y.append(np.array([1,0]))
            positive += 1
        else: 
            train_Y.append(np.array([0,1]))
            negative += 1
    print("Length of data sets: %f with %f positive and %f negatives !" % (len(labeled_list),positive,negative))
    return train_X, train_Y

# Training data.
train_X, train_Y = getTrainingData()

# tf Graph input.
X = tf.placeholder(tf.float32, [image_height,image_width,3])
Y = tf.placeholder(tf.float32, [2,])

# Construct model.
pred = conv_net(X, image_height, image_width, 2)
# Define loss and optimizer
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=pred, labels=Y))
optimizer = tf.train.AdamOptimizer(learning_rate).minimize(cost)
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
        print("\nOptimizing with %f steps" % training_iters)
        while (step < training_iters and not reload_model):
            batch_x, batch_y = getBatch(train_X, train_Y)
            sess.run(optimizer, feed_dict={X: batch_x, Y: batch_y})
            step += 1
            if(step%display_step == 0): print("Progress: %f " % (step/training_iters))
        save_path = saver.save(sess, path_to_model + getModelName(datasets) +" .cpkt")
        print("Optimization Finished! \n")
    #Testing network.
    positive = 0; valid = 0
    test_iterations = 200
    for iteration in range(0,test_iterations):
        #Get random data.
        random_int = randint(1,len(train_X)-1)
        random_image = train_X[random_int]
        random_label = train_Y[random_int]
        img = Image.fromarray(random_image,'RGB')
        #Predict random.
        evaluation = pred.eval(feed_dict={X: random_image})[0]
        prediction = (evaluation[0] > evaluation[1])
        groundtruth = bool(random_label[0])
        if(prediction == groundtruth): valid += 1
        if(prediction): positive += 1
    print("Testing %f data " % test_iterations)
    print("Positives: %f " % positive)
    print("Accuracy: %f !" % (valid/test_iterations))

