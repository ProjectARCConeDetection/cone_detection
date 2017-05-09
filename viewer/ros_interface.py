from PyQt4 import QtCore
import math
import numpy as np

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

class ROSInterface(QtCore.QObject):

	grid_signal = QtCore.pyqtSignal(list)
	position_signal = QtCore.pyqtSignal(list)
	trajectory_signal = QtCore.pyqtSignal(list, list)

	def __init__(self):
		QtCore.QObject.__init__(self)
		#Init ROS.
		rospy.init_node('cone detection gui')
		#Define publisher and subscriber.
		self.mode_pub = rospy.Publisher('mode',Bool, queue_size=10)
		self.grid_sub = rospy.Subscriber('cones_grid',OccupancyGrid,self.gridCallback, queue_size=10)
		self.position_sub = rospy.Subscriber('car_position',Point,self.positionCallback, queue_size=10)
		self.trajectory_sub = rospy.Subscriber('path', Float32MultiArray,self.trajectoryCallback, queue_size=10)

	def gridCallback(self, msg):
		#Init cone list.
		cones = []
		#Get grid parameter.
		grid_height = msg.info.height
		grid_resolution = msg.info.resolution
		grid_width = msg.info.width
		#Convert to point iff cone.
		for x in range(0,grid_height):
			for y in range(0,grid_width):
				if msg.data[x*grid_width + y] == 1: 
					x_coord = x*grid_resolution
					y_coord = (y-grid_width/2)*grid_resolution
					cones.append([x_coord, y_coord])
		#Send signal to gui.
		if(len(cones) > 0): self.grid_signal.emit(cones)

	def trajectoryCallback(self, msg):
		#Get trajectory list.
		listx = []
		listy = []
		for i in range(0,(len(msg.data)/2-1)):
			listx.append(msg.data[2*i])
			listy.append(msg.data[2*i+1])

		#Send signal to gui.
		if(len(msg.data) > 0): self.trajectory_signal.emit(listx, listy)

	def positionCallback(self, msg):
		#Get position.
		position = [msg.x, msg.y]
		#Send signal to gui.
		self.position_signal.emit(position)

	def getSearchingArea(self):
		grid_length = rospy.get_param('/detection/searching_length')
		grid_width = rospy.get_param('/detection/searching_width')
		return grid_length, grid_width

	def toAutonomousMode(self):
		mode_msg = Bool()
		mode_msg.data = True
		self.mode_pub.publish(mode_msg)