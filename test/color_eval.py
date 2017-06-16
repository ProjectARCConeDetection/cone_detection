#!/usr/bin/env python
import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from cone_detection.msg import Label
from sensor_msgs.msg import Image

#Init ros.
rospy.init_node('cone_eval_color')
#Grid parameters.
cone_area_x = rospy.get_param('/detection/cone_area_x')
cone_area_y = rospy.get_param('/detection/cone_area_y')
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

class ColorEvaluator:
	def __init__(self):
		#Init publisher and subscriber.
		rospy.Subscriber('/candidates', Label, self.labeling, queue_size=10)
		self.cones_pub = rospy.Publisher('/cones', Label, queue_size=10)
		#Init cone counter.
		self.cone_counter = 0
		#Cone positions.
		self.cone_positions = []

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
		if(np.linalg.norm(mask) < 200.0): return
		#Update cone label.
		msg.label = True
		#Write cone image and mask.
		cv2.imwrite(path_to_candidate + "cones/" + str(self.cone_counter) + ".jpg", image)
		cv2.imwrite(path_to_candidate + "cones/" + str(self.cone_counter) + "_mask.jpg", mask)
		#Check already existing cones in area.
		for element in self.cone_positions:
			if (abs(msg.x - element[0]) < cone_area_x) and (abs(msg.y - element[1]) < cone_area_y):
				return
		#Update counter, cone positions and publish.
		self.cone_counter += 1
		self.cone_positions.append([msg.x, msg.y])
		self.cones_pub.publish(msg)
		print("Cone %f detected at x = %f and y = %f" % (self.cone_counter, msg.x, msg.y))
#------------------------------------------------------------------------

if __name__ == '__main__':
	#Delete files in candidates and cones order.
	deleteFolderContent(path_to_candidate + "cones/")
	deleteFolderContent(path_to_candidate + "candidates/")
	#Init neural net.
	evaluator = ColorEvaluator()
	#Spinning.
	rospy.spin()
