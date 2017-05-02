#!/usr/bin/env python
from __future__ import print_function, division

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

#Trainings Parameter
n_nodes_hl1 = 3000
n_nodes_hl2 = 3000
n_nodes_hl3 = 3000
batch_size = 50
n_classes = 2 # Cone or None.
hm_epochs = 10


# Network parameter.
rospy.init_node('fully connected network')
image_width = rospy.get_param('/cone/width_pixel')
image_height = rospy.get_param('/cone/height_pixel')
# Datasets.
path_to_directory = rospy.get_param('/candidate_path')
path_to_model = rospy.get_param('/model_path')
datasets = rospy.get_param('/neural_net/datasets')

modelName = ""
modelName = datasets[0]

x = tf.placeholder(tf.float32, [None, image_height*image_width])
y = tf.placeholder(tf.float32, [None, n_classes,])



def getLabels():
    labeled_list = []
    reader = csv.reader(open(path_to_directory + modelName + "/labeling.csv"))
    for row in reader:
        image = int(row[0])
        label = int(row[1])
        labeled_list.append([image, label])
    return labeled_list

def getImageFromPath(path):
    img = Image.open(path)
    img = img.convert('1')      # Convert to black&white
    arr = np.array(img)
    arr = arr.reshape((1, image_height*image_width))
    '''arr = np.array(img.getdata(),
                   np.uint8).reshape(img.size[1], img.size[0], 3)'''
    return arr



def getBatch(train_X, train_Y, numbers):
    train_Batch_X = np.zeros((10, image_width*image_height))
    train_Batch_Y = np.zeros((10, 2))

    for ite in range(10):
        train_Batch_X[ite][:] = train_X[numbers*batch_size+ite]
        train_Batch_Y[ite][:] = train_Y[numbers*batch_size+ite]
    return train_Batch_X, train_Batch_Y

def getBatchAccuracy(train_X, train_Y, numbers):
    train_Batch_X = train_X[numbers]
    train_Batch_Y = train_Y[numbers]
    return train_Batch_X, train_Batch_Y    


def getTrainingData():
    train_X = []
    train_Y = []
    labeled_list = []
    # Getting labeled list.
    labeled_list.append(getLabels())
    # Shuffle list.
    labeled_list = labeled_list[0]
    shuffle(labeled_list)
    # Getting training data.
    for data in labeled_list:
        path = path_to_directory + modelName + "/" + str(data[0]) + ".jpg"
        img = getImageFromPath(path)
        label = data[1]
        train_X.append(img)
        if label: 
            train_Y.append(np.array([1,0]))
        else: 
            train_Y.append(np.array([0,1]))
    return train_X, train_Y

def evaluation(result):
    print("Result: ", result)
    if result[0] > result[1]: max_name = "Cone"
    else: max_name = "None"
    print("Prediction: ", max_name)


def neural_network_model(data):
    l1 = tf.contrib.layers.fully_connected(data, n_nodes_hl1)
    l2 = tf.contrib.layers.fully_connected(l1, n_nodes_hl2)
    l3 = tf.contrib.layers.fully_connected(l2, n_nodes_hl3)
    output_layer = tf.contrib.layers.fully_connected(l3, n_classes)
    return output_layer
    '''
    hidden_1_layer = {'weights':tf.Variable(tf.random_normal([image_height*image_width, n_nodes_hl1])),
                      'biases':tf.Variable(tf.random_normal([n_nodes_hl1]))}

    hidden_2_layer = {'weights':tf.Variable(tf.random_normal([n_nodes_hl1, n_nodes_hl2])),
                      'biases':tf.Variable(tf.random_normal([n_nodes_hl2]))}

    hidden_3_layer = {'weights':tf.Variable(tf.random_normal([n_nodes_hl2, n_nodes_hl3])),
                      'biases':tf.Variable(tf.random_normal([n_nodes_hl3]))}

    output_layer = {'weights':tf.Variable(tf.random_normal([n_nodes_hl3, n_classes])),
                    'biases':tf.Variable(tf.random_normal([n_classes]))}

    l1 = tf.add(tf.matmul(data,hidden_1_layer['weights']), hidden_1_layer['biases'])
    l1 = tf.nn.relu(l1)

    l2 = tf.add(tf.matmul(l1,hidden_2_layer['weights']), hidden_2_layer['biases'])
    l2 = tf.nn.relu(l2)

    l3 = tf.add(tf.matmul(l2,hidden_3_layer['weights']), hidden_3_layer['biases'])
    l3 = tf.nn.relu(l3)

    output = tf.matmul(l3,output_layer['weights']) + output_layer['biases']

    return output'''


def train_neural_network(train_X, train_Y):
    prediction = neural_network_model(x)
    cost = tf.reduce_mean( tf.nn.softmax_cross_entropy_with_logits(logits=prediction, labels=y) )
    optimizer = tf.train.AdamOptimizer().minimize(cost)
    tf.summary.scalar('cost', cost)
    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())
        writer = tf.summary.FileWriter('/home/nico/test/test')
        for epoch in range(hm_epochs):
            epoch_loss = 0
            number_correct_guesses = 0
            for step in range(60):
                batch_x, batch_y = getBatch(train_X, train_Y, step)
                a, c = sess.run([optimizer, cost], feed_dict={x: batch_x, y: batch_y})
                epoch_loss += c
            
            print('Epoch', epoch, 'completed out of',hm_epochs,'loss:',epoch_loss)


        #Testing network.
        number_correct_guesses = 0
        for ite in range(500):
            batch_x1, batch_y1 = getBatchAccuracy(train_X, train_Y, ite + 2500)
            batch_x = np.zeros((1, image_width*image_height))
            batch_x[0][:] = batch_x1[0]
            possibility = prediction.eval(feed_dict={x: batch_x})[0]
            cone = 0
            if possibility[0]>possibility[1]:
                cone=1
            else:
                cone=0
            if cone==batch_y1[0]:
                number_correct_guesses+=1
        percentage = (number_correct_guesses/500)*100
        print("Correct guesses:  ", percentage, "%")

        # Calculate accuracy

        '''writer = tf.summary.FileWriter("/home/nico/1")
        writer.add_graph(sess.graph)'''

train_X, train_Y = getTrainingData()

train_neural_network(train_X, train_Y)

