#!/usr/bin/env python
import cv2
import numpy as np

img = cv2.imread('/home/sele/Desktop/5_angle_-1.0.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray,90,150)
lines = cv2.HoughLines(edges,0.1,np.pi/180,4)
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
        if(angle > 50 and angle < 80):
            cv2.line(img, (x1,y1), (x2,y2), (0,0,255), 2)

cv2.imwrite('/home/sele/Desktop/gray.jpg', edges)
cv2.imwrite("/home/sele/Desktop/lines_" + str(angle) + ".jpg", img)

# More exact but computational demanding method.

# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# edges = cv2.Canny(gray,50,150)
# lines = cv2.HoughLines(edges,0.1,np.pi/360,8)
# for rho, theta in lines[0]:
#   a = np.cos(theta)
#   b = np.sin(theta)
#   x0 = a*rho
#   y0 = b*rho
#   x1 = int(x0 + 1000*(-b))
#   y1 = int(y0 + 1000*(a))
#   x2 = int(x0 - 1000*(-b))
#   y2 = int(y0 - 1000*(a))
#   angle = abs(np.arctan2(y2-y1, x2-x1))*180/np.pi
#   for rho2, theta2 in lines[0]:
#       a = np.cos(theta2)
#       b = np.sin(theta2)
#       x0 = a*rho2
#       y0 = b*rho2
#       x3 = int(x0 + 1000*(-b))
#       y3 = int(y0 + 1000*(a))
#       x4 = int(x0 - 1000*(-b))
#       y4 = int(y0 - 1000*(a))
#       angle2 = abs(np.arctan2(y4-y3, x4-x3))*180/np.pi
#       if(abs(angle - angle2) > 8 and abs(angle - angle2) < 10):
#           print("cone detected !")
#           cv2.line(img, (x1,y1), (x2,y2), (0,0,255), 2)
#           cv2.line(img, (x3,y3), (x4,y4), (0,0,255), 2)