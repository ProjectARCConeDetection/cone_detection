#!/usr/bin/env python
import cv2
import numpy as np

img = cv2.imread('/home/sele/candidates/915.jpg',3)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

matrix = np.ones((5,5),np.float32)/5
img = cv2.filter2D(img,-1,matrix)

edges = cv2.Canny(gray,90,100)

lines = cv2.HoughLines(edges,0.1,np.pi/360, 10)
for rho_1, theta_1 in lines[0]:
	for rho_2, theta_2 in lines[0]:
		delta = abs(theta_2 - theta_1)/np.pi*180
		if(delta < 89 and delta > 40): 
			print(delta)
			print("Cone! ")

for rho,theta in lines[0]:
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))

    cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imshow('houghlines', img)
cv2.waitKey(0)