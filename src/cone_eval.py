#!/usr/bin/env python
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf

import rospy
from cone_detection.msg import Label
from sensor_msgs.msg import Image

#Init ros.
rospy.init_node('cone_eval')
#Net parameters.
path_to_model = rospy.get_param('/model_path')
image_width = rospy.get_param('/object/width_pixel')
image_height = rospy.get_param('/object/height_pixel')
n_classes = 2
datasets = rospy.get_param('/neural_net/datasets')


def convertMsgToArray(image):
	bridge = CvBridge()
	try:
		image_array = bridge.imgmsg_to_cv2(image, "rgb8")
	except CvBridgeError as error:
		print(error)
	return image_array

def getModelName():
    model_name = ""
    for element in datasets:
        model_name += "__" + element
    return model_name

class NeuralNet:
	def __init__(self):
		#Init and saver variable.
		self.X = tf.placeholder(tf.float32, [image_height,image_width,3])
		self.Y = tf.placeholder(tf.float32, [n_classes,])
		self.pred = self.conv_net(self.X)
		self.sess = tf.Session()
		self.init = tf.global_variables_initializer().run(session=self.sess)
		self.saver = tf.train.Saver()
		self.saver.restore(self.sess, path_to_model + getModelName() +" .cpkt")
		#Init publisher and subscriber.
		rospy.Subscriber('/candidates', Label, self.labeling, queue_size=10)
		self.cones_pub = rospy.Publisher('/cones', Label, queue_size=10)

	def conv_net(self,x):
	    # Store layers weight & bias.
	    weights = {
	        'wc1': tf.Variable(tf.random_normal([5, 5, 1, 32])),
	        'wc2': tf.Variable(tf.random_normal([5, 5, 32, 64])),
	        'fc1': tf.Variable(tf.random_normal([15*39*64, 1024])),
	        'out': tf.Variable(tf.random_normal([1024, n_classes]))
	    }

	    biases = {
	        'bc1': tf.Variable(tf.random_normal([32])),
	        'bc2': tf.Variable(tf.random_normal([64])),
	        'fc1': tf.Variable(tf.random_normal([1024])),
	        'out': tf.Variable(tf.random_normal([n_classes]))
	    }   
	    # Reshape input picture.
	    x = tf.reshape(x, shape=[-1,image_height,image_width,1])
	    # Convolutional Layer.
	    conv1  = tf.nn.conv2d(x, weights['wc1'], strides=[1,1,1,1], padding='SAME')
	    conv1 = tf.nn.bias_add(conv1, biases['bc1'])
	    conv1 = tf.nn.relu(conv1)
	    conv1 = tf.nn.max_pool(conv1, ksize=[1,2,2,1], strides=[1,2,2,1], padding='SAME')
	    conv2  = tf.nn.conv2d(conv1, weights['wc2'], strides=[1,1,1,1], padding='SAME')
	    conv2 = tf.nn.bias_add(conv2, biases['bc2'])
	    conv2 = tf.nn.relu(conv2)
	    conv2 = tf.nn.max_pool(conv2, ksize=[1,2,2,1], strides=[1,2,2,1], padding='SAME')
	    # Fully connected layer.
	    fc1 = tf.reshape(conv2, [-1, weights['fc1'].get_shape().as_list()[0]])
	    fc1 = tf.add(tf.matmul(fc1, weights['fc1']), biases['fc1'])
	    fc1 = tf.nn.relu(fc1)
	    # Output, class prediction
	    out = tf.add(tf.matmul(fc1, weights['out']), biases['out'])
	    return tf.nn.softmax(out)

	def labeling(self,msg):
		#Get image.
		image = convertMsgToArray(msg.image)
		#Labeling.
		label = self.pred.eval(session=self.sess,feed_dict={self.X: image})[0]
		if(label[0] > label[1]):
			msg.label = True
			self.cones_pub.publish(msg)
			print("Found cone at x: %f and y: %f" % (msg.x, msg.y))
#------------------------------------------------------------------------

if __name__ == '__main__':
	#Init neural net.
	neural_net = NeuralNet()
	#Spinning.
	rospy.spin()
