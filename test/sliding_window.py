#!/usr/bin/env python
import sys
sys.path.append("/home/sele/catkin_ws/src/cone_detection/neural_net")
from net import *

import time
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
rospy.init_node('sliding_window_detection')
#Net parameters.
path_to_candidate = rospy.get_param('/candidate_path')
path_to_model = rospy.get_param('/model_path')
image_width = rospy.get_param('/cone/width_pixel')
image_height = rospy.get_param('/cone/height_pixel')
datasets = rospy.get_param('/neural_net/datasets')
# Color detection parameter.
lower = np.array([115,50,50])
upper = np.array([130,255,255])
# Neural net variables.
keep_prob = tf.placeholder(tf.float32)
input_placeholder = tf.placeholder(tf.float32, [None, image_height, image_width, 3])
output_placeholder = tf.placeholder(tf.float32, [None, 2])
input_placeholder_flat = tf.contrib.layers.flatten(input_placeholder)
y_true = tf.argmax(output_placeholder, dimension=1)
output_layer = fully_connected(input_placeholder_flat, 0.01, keep_prob)
y_pred = tf.nn.softmax(output_layer)
# Image drop rate.
drop_rate = 15

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

class SlidingWindowEvaluator:
    def __init__(self):
        #Init tf session.
        self.session = tf.Session()
        self.session.run(tf.global_variables_initializer())
        saver = tf.train.Saver()
        saver.restore(self.session, path_to_model + getModelName(datasets) +" .cpkt")
        #Init publisher and subscriber.
        rospy.Subscriber('/usb_cam/image_raw', Image, self.throttleImageRate, queue_size=1)
        self.cones_pub = rospy.Publisher('/cones', Label, queue_size=10)
        # Pass counter.
        self.pass_counter = 0
        # Subscribing Msg counter.
        self.msg_counter = 0

    def throttleImageRate(self, msg):
        # Update counter.
        self.msg_counter += 1
        # Labeling every drop_rate msg.
        if(self.msg_counter % drop_rate == 0):
            self.msg_counter = 0
            self.labeling(msg)

    def labeling(self,msg):
        # Get image from sensor msg.
        image = convertMsgToArray(msg)
        M = cv2.getRotationMatrix2D((image.shape[1]/2,image.shape[0]/2),180,1)
        image = cv2.warpAffine(image,M,(image.shape[1],image.shape[0]))
        # Sliding window.
        for y in range(0, image.shape[0]/image_height-1):
            for x in range(0, image.shape[1]/image_width):
                # Get partial image
                x_image = x*image_width; y_image = y*image_height;
                clone = image.copy()
                cropp_image = image[y_image:y_image + image_height, x_image:x_image + image_width].copy()
                # Color evaluation orange.
                color_clone = cropp_image.copy()
                hsv = cv2.cvtColor(color_clone, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower, upper)
                output = cv2.bitwise_and(color_clone, color_clone, mask = mask)
                masknorm = np.linalg.norm(mask)
                # Line evaluation.
                line_clone = cropp_image.copy()
                gray = cv2.cvtColor(line_clone, cv2.COLOR_BGR2GRAY)
                edges = cv2.Canny(gray,90,150)
                lines = cv2.HoughLines(edges,0.1,np.pi/180,8)
                angle = -1.0
                if lines != None: 
                    for rho, theta in lines[0]:
                        a = np.cos(theta); b = np.sin(theta);
                        x0 = a*rho; y0 = b*rho;
                        x1 = int(x0 + 1000*(-b)); y1 = int(y0 + 1000*(a));
                        x2 = int(x0 - 1000*(-b)); y2 = int(y0 - 1000*(a));
                        angle = abs(np.arctan2(y2-y1, x2-x1))*180/np.pi
                        if(angle > 70.0 and angle < 80):
                            cv2.line(line_clone, (x1,y1), (x2,y2), (0,0,255), 2)
                            break
                # Evaluation.
                # if(masknorm > 2000.0 and ((angle > 70.0 and angle < 80) or angle == -1.0)): 
                if(masknorm > 2000.0 and angle == -1.0):
                    # Neural net evaluation.
                    image_array = np.zeros((1,image_height, image_width,3))
                    image_array[0][:][:][:] =  color.rgb2lab(cropp_image) / 255.0
                    label = y_pred.eval(session=self.session,feed_dict={input_placeholder: image_array, keep_prob: 1.0})[0]
                    # Cone iff neural net agrees.
                    if(label[0] >= 0.4):
                        self.pass_counter += 1
                        cv2.rectangle(clone, (x_image, y_image), (x_image + image_width, y_image + image_height), (0, 255, 0), 2)
                        cv2.imwrite(path_to_candidate + "cones/" + str(self.pass_counter) + "_color_" + str(masknorm) + ".jpg",mask)
                        cv2.imwrite(path_to_candidate + "cones/" + str(self.pass_counter) + "_angle_" + str(angle) + ".jpg",line_clone)
                        cv2.imwrite(path_to_candidate + "cones/" + str(self.pass_counter) + "_net_" + str(label[0]) + ".jpg",cropp_image)
                        print("Cone found !")
                # Show image.
                cv2.imshow("Window", clone)
                cv2.waitKey(1)

# #------------------------------------------------------------------------
if __name__ == '__main__':
    #Delete files in candidates and cones order.
    deleteFolderContent(path_to_candidate + "cones/")
    deleteFolderContent(path_to_candidate + "candidates/")
    #Init neural net.
    evaluator = SlidingWindowEvaluator()
    #Spinning.
    rospy.spin()




