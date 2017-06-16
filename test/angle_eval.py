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

class AngleEvaluator:
    def __init__(self):
        #Init publisher and subscriber.
        rospy.Subscriber('/candidates', Label, self.labeling, queue_size=10)
        self.cones_pub = rospy.Publisher('/cones_angle', Label, queue_size=10)
        #Init cone counter.
        self.cone_counter = 0
        #Cone positions.
        self.cone_positions = []

    def labeling(self,msg):
        #Get image.
        image = convertMsgToArray(msg.image)
        #Canny and Hough transform.
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150)
        lines = cv2.HoughLines(edges,0.1,np.pi/360,8)
        if lines == None: return
        #Find valid edges.
        for rho, theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            angle = abs(np.arctan2(y2-y1, x2-x1))*180/np.pi
            for rho2, theta2 in lines[0]:
                a = np.cos(theta2)
                b = np.sin(theta2)
                x0 = a*rho2
                y0 = b*rho2
                x3 = int(x0 + 1000*(-b))
                y3 = int(y0 + 1000*(a))
                x4 = int(x0 - 1000*(-b))
                y4 = int(y0 - 1000*(a))
                angle2 = abs(np.arctan2(y4-y3, x4-x3))*180/np.pi
                if(abs(angle - angle2) > 8 and abs(angle - angle2) < 10):
                    #Update cone label.
                    msg.label = True
                    #Write cone image and mask.
                    cv2.line(image, (x1,y1), (x2,y2), (0,0,255), 2)
                    cv2.line(image, (x3,y3), (x4,y4), (0,0,255), 2) 
                    cv2.imwrite(path_to_candidate + "cones/" + str(self.cone_counter) + ".jpg", image)
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
    evaluator = AngleEvaluator()
    #Spinning.
    rospy.spin()




