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
rospy.init_node('cone_eval_colornet')
#Grid parameters.
cone_area_x = rospy.get_param('/detection/cone_area_x')
cone_area_y = rospy.get_param('/detection/cone_area_y')
#Net parameters.
path_to_candidate = rospy.get_param('/candidate_path')
path_to_model = rospy.get_param('/model_path')
image_width = rospy.get_param('/cone/width_pixel')
image_height = rospy.get_param('/cone/height_pixel')
datasets = rospy.get_param('/neural_net/datasets')
# Neural net variables.
keep_prob = tf.placeholder(tf.float32)
input_placeholder = tf.placeholder(tf.float32, [None, image_height, image_width, 3])
output_placeholder = tf.placeholder(tf.float32, [None, 2])
input_placeholder_flat = tf.contrib.layers.flatten(input_placeholder)
y_true = tf.argmax(output_placeholder, dimension=1)
output_layer = fully_connected(input_placeholder_flat, 0.01, keep_prob)
y_pred = tf.nn.softmax(output_layer)
# Color detection parameter.
lower = np.array([115,50,50])
upper = np.array([130,255,255])
maskmin = 1000.0
# Line detection parameter.
lower_angle = -1.0
upper_angle = 30.0
line_filter = 10

def convertMsgToArray(image):
    bridge = CvBridge()
    try:
        image_array = bridge.imgmsg_to_cv2(image, "rgb8")
    except CvBridgeError as error:
        print(error)
    return np.asarray(image_array)

def deleteFolderContent(path):
    for element in os.listdir(path):
        os.remove(os.path.join(path, element))

class CombinedDetector:
    def __init__(self):
        #Init tf session.
        self.session = tf.Session()
        self.session.run(tf.global_variables_initializer())
        saver = tf.train.Saver()
        saver.restore(self.session, path_to_model + getModelName(datasets) +" .cpkt")
        #Init publisher and subscriber.
        rospy.Subscriber('/candidates', Label, self.labeling, queue_size=100)
        self.cones_pub = rospy.Publisher('/cones', Label, queue_size=10)
        #Init cone and pass counter.
        self.cone_counter = 0
        self.pass_counter = 0
        #Cone positions.
        self.cone_positions = []

    def labeling(self,msg):
        # Color evaluation orange.
        image = convertMsgToArray(msg.image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)
        masknorm = np.linalg.norm(mask)
        if(masknorm < maskmin): return
        # Line evaluation.
        image_line = convertMsgToArray(msg.image)
        gray = cv2.cvtColor(image_line, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,120,150)
        lines = cv2.HoughLines(edges,0.1,np.pi/180,line_filter)
        angle = -1.0
        if lines != None: 
            for rho, theta in lines[0]:
                a = np.cos(theta); b = np.sin(theta);
                x0 = a*rho; y0 = b*rho;
                x1 = int(x0 + 1000*(-b)); y1 = int(y0 + 1000*(a));
                x2 = int(x0 - 1000*(-b)); y2 = int(y0 - 1000*(a));
                angle = abs(np.arctan2(y2-y1, x2-x1))*180/np.pi
                if(y2==y1): angle = 90
                if(angle>90): angle = angle - 90
                angle = max(0, angle)
            if(angle < lower_angle or angle > upper_angle): return
            #     if(angle > lower_angle and angle < upper_angle):
            #         cv2.line(image_line, (x1,y1), (x2,y2), (0,0,255), 2)
            #         break
            # if(angle < lower_angle and angle < upper_angle): return
        # Machine learning evaluation.
        image_array = np.zeros((1,image_height, image_width,3))
        image_array[0][:][:][:] =  color.rgb2lab(convertMsgToArray(msg.image)) / 255.0
        label = y_pred.eval(session=self.session,feed_dict={input_placeholder: image_array, keep_prob: 1.0})[0]
        if(label[0] <= 0.01): return
        #Update cone label.
        msg.label = True
        self.pass_counter += 1
        #Write cone image.
        cv2.imwrite(path_to_candidate + "cones/" + str(self.pass_counter) + "_orange_" + str(masknorm) + ".jpg",mask)
        cv2.imwrite(path_to_candidate + "cones/" + str(self.pass_counter) + "_angle_" + str(angle) + ".jpg",image_line)
        cv2.imwrite(path_to_candidate + "cones/" + str(self.pass_counter) + "_net_" + str(label[0]) + ".jpg",image)
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
    detector = CombinedDetector()
    #Spinning.
    rospy.spin()
