#!/usr/bin/env python
import numpy
import tensorflow as tf

import rospy
from cone_detection import Label
from sensor_msgs import Image

def convertMsgToArray(image):
	pass

def label(msg):
	#Get position and image.
	pose_x = msg.x
	pose_y = msg.y
	image = convertMsgToArray(msg.image)
	#Load model and label.

if __name__ == '__main__':
	#Init ros, publisher and subscriber.
	rospy.init_node('cone_eval')
	rospy.Subscriber('/candidates', Label, labeling, queue_size=10)
	rospy.Publisher('/cones', Label, queue_size=10)
	#Spinning.
	rospy.spin()

#ToDo: Create Model (s.convolutional), aus argv modell name, tf.eval (s. conv)
#	   convert sensor_msgs to numpy array, load model (s. conv).
