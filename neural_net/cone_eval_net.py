#!/usr/bin/env python
from net import *

import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
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

def convertMsgToArray(image):
	bridge = CvBridge()
	try:
		image_array = bridge.imgmsg_to_cv2(image, "bgr8")
	except CvBridgeError as error:
		print(error)
	return image_array

def deleteFolderContent(path):
	for element in os.listdir(path):
		os.remove(os.path.join(path, element))

class NeuralNet:
	def __init__(self):
		#Init and saver variable.
		input_placeholder = tf.placeholder(tf.float32, [None, image_height, image_width, 3])
		output_layer = fully_connected(input_placeholder_flat, 0.01)
		#Init tf session.
		self.sess = tf.Session()
		self.init = tf.global_variables_initializer().run(session=self.sess)
		self.saver = tf.train.Saver()
		self.saver.restore(self.sess, path_to_model + getModelName(datasets) +" .cpkt")
		#Init publisher and subscriber.
		rospy.Subscriber('/candidates', Label, self.labeling, queue_size=10)
		self.cones_pub = rospy.Publisher('/cones', Label, queue_size=10)
		#Init cone counter.
		self.cone_counter = 0

	def labeling(self,msg):
		#Get image.
		image = np.zeros((1,image_height, image_width,3))
		image[0][:][:][:] =  color.rgb2lab(convertMsgToArray(msg.image)) / 255.0
		#Labeling.
		label = self.pred.eval(session=self.sess,feed_dict={self.X: image})[0]
		if(label[0] > label[1]):
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
