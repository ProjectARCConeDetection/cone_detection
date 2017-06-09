#!/usr/bin/env python
import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys

import rospy
from cone_detection.msg import Label
from sensor_msgs.msg import Image

sys.path.append('../neural_net')
from data_handler import *

test_size = 64

# Init ROS.
rospy.init_node('comparing')
# Image ṕarameter.
image_width = rospy.get_param('/cone/width_pixel')
image_height = rospy.get_param('/cone/height_pixel')
# Datasets.
path_to_directory = rospy.get_param('/candidate_path')
path_to_model = rospy.get_param('/model_path')
datasets = rospy.get_param('/neural_net/datasets')
datasets_test = rospy.get_param('/neural_net/datasets_test')

class Evaluator:
	def __init__(self):
		# Subscriber and Publisher.
		rospy.Subscriber('/cones', Label, self.netEvaluation, queue_size=10)
		rospy.Subscriber('/cones_angle', Label, self.angleEvaluation, queue_size=10)
		rospy.Subscriber('/cones_color', Label, self.colorEvaluation, queue_size=10)
		self.test_pub = rospy.Publisher('/candidates', Label, queue_size=10)
		# Get test data.
		data = DataHandler(image_height,image_width,  
                   		   path_to_directory, path_to_model, datasets, datasets_test)
		self.test_x, self.test_y = data.getTestBatch(test_size)
		# Counter.
		self.net_counter = 0
		self.color_counter = 0
		self.angle_counter = 0

	def angleEvaluation(self, msg):
		index = msg.x 
		groundtruth = (self.test_y[index] == [0])
		if(msg.label == groundtruth) self.net_counter += 1

	def colorEvaluation(self, msg):
		index = msg.x 
		groundtruth = (self.test_y[index] == [0])
		if(msg.label == groundtruth) self.net_counter += 1

	def netEvaluation(self, msg):
		index = msg.x 
		groundtruth = (self.test_y[index] == [0])
		if(msg.label == groundtruth) self.net_counter += 1


# Get test data and publish.

for index in range(0, test_size-1):
	pass






