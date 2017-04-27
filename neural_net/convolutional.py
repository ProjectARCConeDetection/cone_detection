#!/usr/bin/env python
from __future__ import print_function

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
display_step = 100
toolbar_length = training_iters/100
reload_model = True

# Network parameter.
rospy.init_node('convolutional network')
image_width = rospy.get_param('/cone/width_pixel')
image_height = rospy.get_param('/cone/height_pixel')
# Datasets.
path_to_directory = rospy.get_param('/candidate_path')
path_to_model = rospy.get_param('/model_path')
datasets = rospy.get_param('/neural_net/datasets')

def evaluation(result):
    print("Result: ", result)
    if result[0] > result[1]: max_name = "Cone"
    else: max_name = "None"
    print("Prediction: ", max_name)

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
    for data in labeled_list:
        path = path_to_directory + data[2] + "/" + str(data[0]) + ".jpg"
        img = getImageFromPath(path)
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
        print("Restored model !")
    else:
        print("Optimizing")
        sys.stdout.write("[%s]" % (" " * (training_iters/toolbar_length)))
        sys.stdout.flush()
        sys.stdout.write("\b" * (training_iters/toolbar_length+1))
        while (step < training_iters and not reload_model):
            batch_x, batch_y = getBatch(train_X, train_Y)
            sess.run(optimizer, feed_dict={X: batch_x, Y: batch_y})
            if(step%toolbar_length == 0):
                sys.stdout.write("-")
                sys.stdout.flush()
            step += 1
        save_path = saver.save(sess, path_to_model + getModelName(datasets) +" .cpkt")
        print("\n Optimization Finished!")
    #Testing network.
    random_image = train_X[randint(1,len(train_X)-1)]
    img = Image.fromarray(random_image,'RGB')
    img.show()
    evaluation(pred.eval(feed_dict={X: random_image})[0])