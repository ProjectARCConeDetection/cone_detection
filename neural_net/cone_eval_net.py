#!/usr/bin/env python
from net import *

import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
#import tensorflow as tf
from skimage import color

import rospy
from cone_detection.msg import Label
from sensor_msgs.msg import Image

#Init ros.
rospy.init_node('cone_eval')
#Net parameters.
path_to_candidate = rospy.get_param('/candidate_path')
path_to_model = rospy.get_param('/model_path')
image_width = rospy.get_param('/cone/width_pixel')
image_height = rospy.get_param('/cone/height_pixel')
datasets = rospy.get_param('/neural_net/datasets')
#Init and saver variable.
input_placeholder = tf.placeholder(tf.float32, [None, image_height, image_width, 3])
output_placeholder = tf.placeholder(tf.float32, [None, 2])
input_placeholder_flat = tf.contrib.layers.flatten(input_placeholder)
y_true = tf.argmax(output_placeholder, dimension=1)
output_layer = fully_connected(input_placeholder_flat, 0.01)
y_pred = tf.argmax(tf.nn.softmax(output_layer), dimension=1)


def convertMsgToArray(image):
	bridge = CvBridge()
	try:
		image_array = bridge.imgmsg_to_cv2(image, "bgr8")
        img_arr = numpy.asarray(image_array)
	except CvBridgeError as error:
		print(error)
	return img_arr

def deleteFolderContent(path):
	for element in os.listdir(path):
		os.remove(os.path.join(path, element))

class NeuralNet:
	def __init__(self):
		#Init tf session.
		self.session = tf.Session()
		self.session.run(tf.global_variables_initializer())
		saver = tf.train.Saver()
		saver.restore(self.session, path_to_model + getModelName(datasets) +" .cpkt")
		#Init publisher and subscriber.
		rospy.Subscriber('/candidates', Label, self.labeling, queue_size=10)
		self.cones_pub = rospy.Publisher('/cones', Label, queue_size=10)
		#Init cone counter.
		self.cone_counter = 0

	def labeling(self,msg):
		#Get image.
		image = np.zeros((1,image_height, image_width,3))
		image[0][:][:][:] =  color.rgb2lab(convertMsgToArray(msg.image)) / 255.0
		# Labeling.
		label = y_pred.eval(session=self.session,feed_dict={input_placeholder: image})
		if(label == [0]):
		#if(True):
			msg.label = True
			self.cone_counter += 1
			cv2.imwrite(path_to_candidate + "cones/" + str(self.cone_counter) + ".jpg", convertMsgToArray(msg.image))
			self.cones_pub.publish(msg)
#------------------------------------------------------------------------

if __name__ == '__main__':
	#Delete files in candidates and cones order.
	deleteFolderContent(path_to_candidate + "cones/")
	deleteFolderContent(path_to_candidate + "candidates/")
	#Init neural net.
	neural_net = NeuralNet()
	#Spinning.
	rospy.spin()
