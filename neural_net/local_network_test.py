#!/usr/bin/env python
from net import *

import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from skimage import color
from PIL import Image

import rospy
from cone_detection.msg import Label

#Init ros.
rospy.init_node('local_network_test')
#Net parameters.
image_width = 50
image_height = 60
path_to_candidate = '/home/sele/cones/candidates/'
path_to_model = '/home/sele/cones/models/'
datasets = ['biberist_20_4']
#Init and saver variable.
input_placeholder = tf.placeholder(tf.float32, [None, image_height, image_width, 3])
output_placeholder = tf.placeholder(tf.float32, [None, 2])
input_placeholder_flat = tf.contrib.layers.flatten(input_placeholder)
y_true = tf.argmax(output_placeholder, dimension=1)
output_layer = fully_connected(input_placeholder_flat, 0.01)
y_pred = tf.argmax(tf.nn.softmax(output_layer), dimension=1)


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
        #Init cone counter.
        self.cone_counter = 0
        for i in range(0,1500):
            path = path_to_candidate + "candidates/" + str(i) + ".jpg"
            try:
                img = Image.open(path)
            except:
                continue
            arr = np.array(img.getdata(),np.uint8)        
            arr = arr.reshape(image_height, image_width, 3)
            self.labeling(arr, i)


    def labeling(self,msg, index):
        #Get image.
        image = np.zeros((1,image_height, image_width,3))
        image[0][:][:][:] =  color.rgb2lab(msg) / 255.0
        # Labeling.
        label = y_pred.eval(session=self.session,feed_dict={input_placeholder: image})
        if(label == [0]):
            self.cone_counter += 1
            cv2.imwrite(path_to_candidate + "cones/" + str(index) + ".jpg", msg)
#------------------------------------------------------------------------

if __name__ == '__main__':
    #Delete files in candidates and cones order.
    deleteFolderContent(path_to_candidate + "cones/")
    #Init neural net.
    neural_net = NeuralNet()