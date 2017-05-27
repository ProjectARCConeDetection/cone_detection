#!/usr/bin/env python
from net import *

import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from cone_detection.msg import Label
from sensor_msgs.msg import Image

#Init ros.
rospy.init_node('cone_eval_color')
#Net parameters.
path_to_candidate = rospy.get_param('/candidate_path')
image_width = rospy.get_param('/cone/width_pixel')
image_height = rospy.get_param('/cone/height_pixel')

def convertMsgToArray(image):
	bridge = CvBridge()
	try:
		image_array = bridge.imgmsg_to_cv2(image, "rgb8")
	except CvBridgeError as error:
		print(error)
	return image_array

def deleteFolderContent(path):
	for element in os.listdir(path):
		os.remove(os.path.join(path, element))

class NeuralNet:
	def __init__(self):
		#Init publisher and subscriber.
		rospy.Subscriber('/candidates', Label, self.labeling, queue_size=10)
		self.cones_pub = rospy.Publisher('/cones', Label, queue_size=10)
		#Init cone counter.
		self.cone_counter = 0
		print("Eval initialised !")

	def labeling(self,msg):
		#Get image.
		image = convertMsgToArray(msg.image)
		#Convert to hsv.
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		#Define color boundaries (hsv - Farbwert, Saettigung, Hellwert).
		lower = np.array([115,50,50])
		upper = np.array([130,255,255])
		#Find colors in bounds and apply mask.
		mask = cv2.inRange(hsv, lower, upper)
 		output = cv2.bitwise_and(image, image, mask = mask)
		if(np.linalg.norm(mask) > 2000.0):
			msg.label = True
			self.cone_counter += 1
			cv2.imwrite(path_to_candidate + "cones/" + str(self.cone_counter) + ".jpg", image)
			cv2.imwrite(path_to_candidate + "cones/" + str(self.cone_counter) + "_mask.jpg", mask)
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