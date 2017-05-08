#!/usr/bin/env python
from PyQt4 import QtGui, QtCore
import pyqtgraph as pg
import numpy as np
import signal
import sys

from ros_interface import ROSInterface

def getPointsFromList(x_points,y_points):
	#Compare list length.
	if(len(x_points) != len(y_points)): print("Error in getPointsFromList !")
	#Write to single arrays.
	points = []
	for index in range(0, len(x_points)-1):
		points.append([x_points[index],y_points[index]])
	return points

class GUI(QtGui.QWidget):
	def __init__(self):
		super(GUI, self).__init__()
		#UI parameter.
		self.height = 600
		self.width = 1000
		#Graph list.
		self.graph_points = []
		#Init ROSInterface.
		self.ros_interface = ROSInterface()
		#Build up UI.
		self.initUI()
		self.qtConnections()

	def initUI(self):
		self.setWindowTitle('ARC Cone Detection')
		#Define layouts.
		left_layout = QtGui.QHBoxLayout() 
		right_up_layout = QtGui.QHBoxLayout()
		right_down_layout = QtGui.QHBoxLayout()
		#Get grid area.
		grid_length, grid_width = self.ros_interface.getSearchingArea()
		#Path plot.
		self.plotwidget = pg.PlotWidget()
		self.plotcurve = pg.ScatterPlotItem()
		self.plotwidget.addItem(self.plotcurve)
		self.plotwidget.setYRange(-10, 10)
		self.plotwidget.setXRange(0,grid_length)
		left_layout.addWidget(self.plotwidget)
		#Start button.
		self.start_button = QtGui.QPushButton("System Booting")
		self.start_button.setFont(QtGui.QFont('SansSerif',15,weight=QtGui.QFont.Bold))
		self.start_button.setStyleSheet("background-color: yellow")
		self.start_button.setFixedSize(self.width/4,self.height/8)
		right_down_layout.addWidget(self.start_button)
		#Set layouts.
		layout = QtGui.QGridLayout()
		layout.addLayout(left_layout, 0, 0, 3, 3)
		layout.addLayout(right_up_layout, 0, 3, 2, 1)
		layout.addLayout(right_down_layout, 2, 2)
		self.setLayout(layout)
		#Set Window geometry and background color.
		palette = QtGui.QPalette()
		palette.setColor(QtGui.QPalette.Background,QtCore.Qt.black)
		self.setPalette(palette)
		self.setGeometry(100, 100, self.width, self.height)
		self.show()

	def qtConnections(self):
		#Start button.
		self.start_button.clicked.connect(self.changeMode)
		#Cone grid.
		self.ros_interface.grid_signal.connect(self.updateGrid)
		#Position.
		self.ros_interface.position_signal.connect(self.updatePosition)
		#Trajectory.
		self.ros_interface.trajectory_signal.connect(self.updateTrajectory)

	def changeMode(self):
		self.ros_interface.toAutonomousMode()
		self.start_button.setStyleSheet("background-color: blue")
		self.start_button.setText("System started") 

	def updateGrid(self, cones):
		#Get cone position.
		cones_x = []; cones_y = []
		for element in cones:
			cones_x.append(element[0])
			cones_y.append(element[1])
		#Add cone points to graph.
		self.plotcurve.addPoints(cones_x, cones_y, symbol='x', pen=QtGui.QPen(QtGui.QColor(255, 255, 255)))
		#Update point list.
		self.graph_points.append(getPointsFromList(cones_x, cones_y))

	def updatePosition(self, position):
		#Get car position and add points to graph.
		x = [position[0]]; y = [position[1]]
		self.plotcurve.addPoints(x, y, symbol='o', pen=QtGui.QPen(QtGui.QColor(255, 0, 0)))
		#Update point list.
		self.graph_points.append(getPointsFromList(x, y))

	def updateTrajectory(self, trajectory):
		#Get path points.
		points = []
		points.append(getPointsFromList(trajectory[0],trajectory[1]))
		points.append(self.graph_points)
		x = [points[0]]; y = [points[1]]
		#Replot graph.
		self.plotcurve.clear()
		self.plotcurve.addPoints(x, y, symbol='o', pen=QtGui.QPen(QtGui.QColor(0, 255, 0)))

def main():
	#Starting application.
	app = QtGui.QApplication(sys.argv)
	app.setApplicationName('ARC Cone Detection')
	#Creating GUI.
	ex = GUI()
	# Exit.
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()
