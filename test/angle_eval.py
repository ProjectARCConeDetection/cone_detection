#!/usr/bin/env python
import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from cone_detection.msg import Label
from sensor_msgs.msg import Image

#Init ros.
rospy.init_node('cone_eval_angle')
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

class AngleEvaluator:
    def __init__(self):
        #Init publisher and subscriber.
        rospy.Subscriber('/candidates', Label, self.labeling, queue_size=10)
        self.cones_pub = rospy.Publisher('/cones_angle', Label, queue_size=10)
        #Init cone counter.
        self.cone_counter = 0
        print("Eval initialised !")

    def labeling(self,msg):
        #Get image.
        image = convertMsgToArray(msg.image)
        #Convert to hsv.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        #Canny and Hough transform.
        matrix = np.ones((5,5),np.float32)/5
        img = cv2.filter2D(img,-1,matrix)
        edges = cv2.Canny(gray,90,100)
        lines = cv2.HoughLines(edges,0.1,np.pi/360, 10)
        #Find valid edges.
        for rho_1, theta_1 in lines[0]:
            for rho_2, theta_2 in lines[0]:
                delta = abs(theta_2 - theta_1)/np.pi*180
                if(delta < 89 and delta > 40): 
                    msg.label = True
                    self.cone_counter += 1
                    cv2.imwrite(path_to_candidate + "cones/" + str(self.cone_counter) + ".jpg", image)
                    cv2.imwrite(path_to_candidate + "cones/" + str(self.cone_counter) + "_lines.jpg", edges)
                    self.cones_pub.publish(msg) 

#------------------------------------------------------------------------
if __name__ == '__main__':
    #Delete files in candidates and cones order.
    deleteFolderContent(path_to_candidate + "cones/")
    deleteFolderContent(path_to_candidate + "candidates/")
    #Init neural net.
    evaluator = AngleEvaluator()
    #Spinning.
    rospy.spin()




