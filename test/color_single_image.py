#!/usr/bin/env python
import cv2
import numpy as np

image = cv2.imread('/home/sele/Desktop/3.jpg')
# Convert to hsv.
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# Define color boundaries (hsv - Farbwert, Saettigung, Hellwert).
lower = np.array([115,50,50])
upper = np.array([130,255,255])
# Find colors in bounds and apply mask.
mask = cv2.inRange(hsv, lower, upper)
output = cv2.bitwise_and(image, image, mask = mask)
# Labeling.
if(np.linalg.norm(mask) > 2000.0):
	print("Cone detected !")

cv2.imwrite('/home/sele/Desktop/mask.jpg', mask)