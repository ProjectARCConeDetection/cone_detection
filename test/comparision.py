#!/usr/bin/env python
import sys
sys.path.append("/home/sele/catkin_ws/src/cone_detection/neural_net")
from net import *

import csv
import cv2
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import numpy as np
import os
from PIL import Image
from random import randint, shuffle
from skimage import color
import tensorflow as tf
import time

# Validation dataset.
dataset = "/home/sele/cones/candidates/validation_test1"
# Neural net model.
machine_model_name = "/home/sele/cones/models/ConvOptimised/__biberist_20_4__hoengg_18_05__Hoengg_26_05_01__Hoengg_26_05_02__Hoengg_26_05_03__hoengg_09_06_01__hoengg_09_06_02 .cpkt"
# File name.
filename = "/home/sele/Desktop/comparison.png"
# Colour detection.
lower = np.array([115,50,50])
upper = np.array([130,255,255])
colour_minnorm = 2000.0
# Triangle detection.
triangle_lower_angle =  50.0
triangle_upper_angle = 80.0
triangle_line_filter = 8
# Angle detection.
angle_lower_angle =  -1.0
angle_upper_angle = 30.0
angle_line_filter = 12
# Machine detection.
machine_threshold = 0.1
# Overfilter detection.
filter_line_filter = 8
filter_colour_minnorm = 1500.0
filter_machine_threshold = 0.1

# Define network placeholder.
keep_prob = tf.placeholder(tf.float32)
input_placeholder = tf.placeholder(tf.float32, [None,60,50,3])
output_placeholder = tf.placeholder(tf.float32, [None, 2])
input_placeholder_flat = tf.contrib.layers.flatten(input_placeholder)
output_layer = conv_net(input_placeholder, 0.01, keep_prob)
y_pred = tf.nn.softmax(output_layer)
# Load network.
session = tf.Session()
session.run(tf.global_variables_initializer())
saver = tf.train.Saver()
saver.restore(session, machine_model_name)

class DataHandler:
    def __init__(self, path_to_dataset):
        # Read dataset.
        self.data, self.positive, self.negative = self.readDataset(path_to_dataset)

    def calculateParameter(self, programm_list, time_effort):
        # Parameter table (x-axis = Label, y-axis = Classification).
        ##### t #### f ####
        # t # a      b ####
        # f # c      d ####
        ###################
        a = 0.0; b = 0.0; c = 0.0; d = 0.0;
        # Check every list element.
        for index in range(0, len(data)):
            label_data = data[index][1]
            label_program = programm_list[index][1]
            if(label_data and label_program): a += 1.0
            elif(label_data and not label_program): c += 1.0
            elif(not label_data and label_program): b += 1.0
            elif(not label_data and not label_program): d += 1.0
            else: print("Error at index" + str(index))
        # Calculate paramter.
        recall = a / (a+c)
        accuracy = (a+d) / (a+b+c+d)
        false_positives = b / (b+d)
        true_negatives = c / (a+c)
        return [accuracy, recall, false_positives, true_negatives, time_effort]

    def imageFromPath(self, path):
        img = Image.open(path)
        arr = np.array(img.getdata(),np.uint8)
        arr = arr.reshape(60, 50, 3)
        return arr

    def readDataset(self, path_to_dataset):
        # Read and save datasets.
        labeled_list = []
        reader = csv.reader(open(path_to_dataset + "/labeling.csv"))
        for row in reader:
            image = int(row[0])
            label = int(row[1])
            labeled_list.append([image, label])
        # Shuffle dataset.
        shuffle(labeled_list)
        # Getting data.
        data = []; 
        positive = 0; negative = 0;
        for element in labeled_list:
            path = path_to_dataset + "/" + str(element[0]) + ".jpg"
            img = self.imageFromPath(path)
            label = element[1]
            data.append([img, label])
            if label: positive += 1
            else: negative += 1
        return data, positive, negative

    def getData(self):
        return self.data

def colourClassification(data):
    colour_list = []
    # Start time.
    start_time = time.time()
    for element in data:
        # Convert to hsv.
        hsv = cv2.cvtColor(element[0], cv2.COLOR_BGR2HSV)
        # Find colors in bounds and apply mask.
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(element[0], element[0], mask = mask)
        # Labeling.
        if(np.linalg.norm(mask) > colour_minnorm): colour_list.append([None, 1])
        else: colour_list.append([None, 0])
    # Stop time.
    time_difference = time.time() - start_time
    return colour_list, time_difference

def triangleClassification(data):
    triangle_list = []
    # Start time.
    start_time = time.time()
    for element in data:
        # Transform image.
        cone_detected = False
        gray = cv2.cvtColor(element[0], cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,80,150)
        lines = cv2.HoughLines(edges,0.1,np.pi/180,triangle_line_filter)
        angle = -1.0
        if lines is None: 
            triangle_list.append([None, 0])
            continue
        for rho, theta in lines[0]:
            a = np.cos(theta); b = np.sin(theta)
            x0 = a*rho; y0 = b*rho
            x1 = int(x0 + 1000*(-b)); y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b)); y2 = int(y0 - 1000*(a))
            angle = abs(np.arctan2(y2-y1, x2-x1))*180/np.pi
            for rho2, theta2 in lines[0]:
                a = np.cos(theta2); b = np.sin(theta2)
                x0 = a*rho2; y0 = b*rho2
                x3 = int(x0 + 1000*(-b)); y3 = int(y0 + 1000*(a))
                x4 = int(x0 - 1000*(-b)); y4 = int(y0 - 1000*(a))
                angle2 = abs(np.arctan2(y4-y3, x4-x3))*180/np.pi
                diff_angle = abs(angle - angle2)
                if(diff_angle > triangle_lower_angle and diff_angle < triangle_upper_angle): 
                    cone_detected = True
            # Evaluation
            if cone_detected: triangle_list.append([None, 1])
            else: triangle_list.append([None, 0])
    # Stop time.
    time_difference = time.time() - start_time
    return triangle_list, time_difference

def angleClassification(data):
    angle_list = []
    # Start time.
    start_time = time.time()
    for element in data:
        # Transform image.
        cone_detected = False
        gray = cv2.cvtColor(element[0], cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,80,150)
        lines = cv2.HoughLines(edges,0.1,np.pi/180,angle_line_filter)
        angle = -1.0
        if lines is None: 
            angle_list.append([None, 0])
            continue
        for rho, theta in lines[0]:
            a = np.cos(theta); b = np.sin(theta)
            x0 = a*rho; y0 = b*rho
            x1 = int(x0 + 1000*(-b)); y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b)); y2 = int(y0 - 1000*(a))
            angle = abs(np.arctan2(y2-y1, x2-x1))*180/np.pi
            if(angle > angle_lower_angle and angle < angle_upper_angle): 
                cone_detected = True
        # Evaluation
        if cone_detected: angle_list.append([None, 1])
        else: angle_list.append([None, 0])
    # Stop time.
    time_difference = time.time() - start_time
    return angle_list, time_difference

def machineClassification(data):
    machine_list = []
    # Start time.
    start_time = time.time()
    # Evaluation.
    for element in data:
        image = np.zeros((1,60,50,3))
        image[0][:][:][:] =  color.rgb2lab(element[0]) / 255.0
        # Labeling.
        label = y_pred.eval(session=session,feed_dict={input_placeholder: image, keep_prob: 1.0})[0]
        if(label[0] > machine_threshold): machine_list.append([None, 1])
        else: machine_list.append([None, 0])
    # Stop time.
    time_difference = time.time() - start_time
    return machine_list, time_difference

def overallFilteredClassification(data):
    over_filter_list = []
    # Start time.
    start_time = time.time()
    # Evaluation.
    for element in data:
        # Colour filtering.
        colour_img = np.copy(element[0])
        hsv = cv2.cvtColor(colour_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(colour_img, colour_img, mask = mask)
        if(np.linalg.norm(mask) < filter_colour_minnorm): 
            over_filter_list.append([None, 0])
            continue
        # Angle filtering.
        angle_img = np.copy(element[0])
        cone_detected = False
        gray = cv2.cvtColor(element[0], cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,80,150)
        lines = cv2.HoughLines(edges,0.1,np.pi/180,filter_line_filter)
        angle = -1.0
        if lines is not None: 
            for rho, theta in lines[0]:
                a = np.cos(theta); b = np.sin(theta)
                x0 = a*rho; y0 = b*rho
                x1 = int(x0 + 1000*(-b)); y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b)); y2 = int(y0 - 1000*(a))
                angle = abs(np.arctan2(y2-y1, x2-x1))*180/np.pi
                if(y2==y1): angle = 90
                if(angle>90): angle = angle - 90
                if(angle < angle_lower_angle or angle > angle_upper_angle): 
                    cone_detected = True
            if not cone_detected: 
                over_filter_list.append([None, 0])
                continue
        # Machine filter.
        image = np.zeros((1,60,50,3))
        image[0][:][:][:] =  color.rgb2lab(element[0]) / 255.0
        label = y_pred.eval(session=session,feed_dict={input_placeholder: image, keep_prob: 1.0})[0]
        if(label[0] < filter_machine_threshold): 
            over_filter_list.append([None, 0])
            continue
        # Cone evaluated.
        over_filter_list.append([None, 1])
    # Stop time.
    time_difference = time.time() - start_time
    return over_filter_list, time_difference

def createTableEntry(params, name):
    params = [round(element, 3) for element in params]
    return [name, params[0], params[1], params[2], params[3], params[4]]


if __name__ == '__main__':
    # Init datahandler.
    datahandler = DataHandler(dataset)
    data = datahandler.getData()
    # Merely colour.
    colour_list, colour_time = colourClassification(data)
    colour_parameter = datahandler.calculateParameter(colour_list, colour_time)
    colour_parameter = createTableEntry(colour_parameter, 'colour')
    # Merely triangle.
    # triangle_list, triangle_time = triangleClassification(data)
    # triangle_parameter = datahandler.calculateParameter(triangle_list, triangle_time)
    # triangle_parameter = createTableEntry(triangle_parameter, 'triangle')
    # Merely triangle.
    angle_list, angle_time = angleClassification(data)
    angle_parameter = datahandler.calculateParameter(angle_list, angle_time)
    angle_parameter = createTableEntry(angle_parameter, 'angle')
    # Merely network.
    machine_list, machine_time = machineClassification(data)
    machine_parameter = datahandler.calculateParameter(machine_list, machine_time)
    machine_parameter = createTableEntry(machine_parameter, 'CNN')
    # Overall filtering.
    over_filter_list, over_filter_time = overallFilteredClassification(data)
    over_filter_parameter = datahandler.calculateParameter(over_filter_list, over_filter_time)
    over_filter_parameter = createTableEntry(over_filter_parameter, 'filtered CNN')
    # Plot table.
    frame = plt.gca()
    frame.axes.get_xaxis().set_ticks([])
    frame.axes.get_yaxis().set_ticks([])
    mean_labels=['Approach','Accuracy','Recall','FPR','TFR','Time (108 img)']
    mean_vals=[colour_parameter, angle_parameter, 
               machine_parameter, over_filter_parameter]
    mean_table = plt.table(cellText=mean_vals,
                           colWidths = [0.15]*6,
                           colLabels=mean_labels,
                           loc='center left')
    mean_table.set_fontsize(14)
    plt.savefig(filename)
    plt.close()
