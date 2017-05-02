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
training_iters = 1000
batch_size = 50
test_iterations = 20
display_step = training_iters/20
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

def equaliseSet(set_X, set_Y):
    #Count positives and negatives.
    positives = 0; negatives = 0;
    for element in set_Y:
        if(element[0]): positives += 1
        else: negatives += 1 
    #Find bias.
    negative_bias = (negatives > (len(set_X)/2))
    if(negative_bias): counter = positives
    if(not negative_bias): counter = negatives
    #Equalise.
    equalised_x = []; equalised_y = [];
    for index in range(0, len(set_X)-1):
        if(counter <= 0 and negative_bias and set_Y[index][1]): continue
        if(counter <= 0 and not negative_bias and set_Y[index][0]): continue
        equalised_x.append(set_X[index])
        equalised_y.append(set_Y[index])
        if(negative_bias and set_Y[index][1]): counter -= 1
        if(not negative_bias and set_Y[index][0]): counter -= 1
    return equalised_x, equalised_y

def getAccuracy(train_X, train_Y):
    positive = 0; valid = 0; negatives = 0;
    iteration = 0
    test_x = []; test_y = [];
    #Get random data.
    while iteration < test_iterations:
        random_int = randint(1,len(train_X)-1)
        test_x.append(train_X[random_int])
        test_y.append(train_Y[random_int])
        iteration += 1
    #Equalise dataset.
    test_x, test_y = equaliseSet(test_x, test_y)
    #Predict random.
    valid = 0
    length = len(test_x)-1
    for index in range(0, length):
        random_image = np.zeros((1,image_height,image_width,3))
        random_image[0][:][:][:] =  test_x[index]
        random_label = test_y[index]
        evaluation = pred.eval(feed_dict={X: random_image})[0]
        prediction = (evaluation[0] > evaluation[1])
        groundtruth = bool(random_label[0])
        if(prediction == groundtruth): valid += 1
    accuracy = valid/length
    return accuracy

def getBatch(train_X, train_Y):
    #Get random list element.
    length = len(train_X)-1
    batch_x = np.zeros((batch_size,image_height, image_width,3))
    batch_y = np.zeros((batch_size,2))
    for index in range(0,batch_size-1):
        i = randint(0,length)
        batch_x[index][:][:][:] = train_X[i]
        batch_y[index][:][:][:] = train_Y[i]
    return batch_x,batch_y

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
    # Not equalising.
    if(not equalising):
        print("Length of data sets: %f with %f positive and %f negatives !" % (len(labeled_list),positive,negative))
        return train_X, train_Y
    # Equalise dataset.
    equalised_x, equalised_y = equaliseSet(train_X, train_Y)
    print("Lenght of data sets: %f, equalised !" % len(equalised_x))
    return equalised_x, equalised_y

# Training data.
train_X, train_Y = getTrainingData()

# tf Graph input.
X = tf.placeholder(tf.float32, [None,image_height,image_width,3])
Y = tf.placeholder(tf.float32, [None,2,])

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
            if(step%display_step == 0): 
                accuracy = getAccuracy(train_X, train_Y)
                print("Progress: %f with Accuracy %f" % (step/training_iters, accuracy))
        save_path = saver.save(sess, path_to_model + getModelName(datasets) +" .cpkt")
        print("Optimization Finished! \n")
    #Testing network.
    accuracy = getAccuracy(train_X, train_Y)
    print("Testing %f data " % test_iterations)
    print("Accuracy: %f !" % accuracy)

