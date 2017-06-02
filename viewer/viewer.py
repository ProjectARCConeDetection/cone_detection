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
		self.graph_points_x = []; self.graph_points_y = []
		self.cones_x = []; self.cones_y = []
		#Init ROSInterface.
		self.ros_interface = ROSInterface()
		#Build up UI.
		self.initUI()
		self.qtConnections()

	def initUI(self):
		self.setWindowTitle('ARC Cone Detection')
		#Define layouts.
		upper_layout = QtGui.QHBoxLayout() 
		lower_left_layout = QtGui.QVBoxLayout()
		lower_middle_left_layout = QtGui.QVBoxLayout()
		lower_middle_right_layout = QtGui.QVBoxLayout()
		lower_right_layout = QtGui.QHBoxLayout()
		#Get grid area.
		grid_length, grid_width = self.ros_interface.getSearchingArea()
		#Path plot.
		self.plotwidget = pg.PlotWidget()
		self.plotcurve = pg.ScatterPlotItem()
		self.plotwidget.addItem(self.plotcurve)
		self.plotwidget.setXRange(0, 100)
		self.plotwidget.setYRange(-3, 3)
		upper_layout.addWidget(self.plotwidget)
		#Start button.
		self.start_button = QtGui.QPushButton("System Booting")
		self.start_button.setFont(QtGui.QFont('SansSerif',15,weight=QtGui.QFont.Bold))
		self.start_button.setStyleSheet("background-color: yellow")
		self.start_button.setFixedSize(self.width/4,self.height/8)
		lower_right_layout.addWidget(self.start_button)
		#Cone Counter label.
		label = QtGui.QLabel("Cone counter")
		label.setStyleSheet('color: white')
		self.cone_counter_label = QtGui.QTextEdit("0")
		self.cone_counter_label.setFixedHeight(20)
		self.cone_counter_label.setFixedWidth(200)
		lower_left_layout.addWidget(label)
		lower_left_layout.addWidget(self.cone_counter_label)
		#Vehicle control label.
		label = QtGui.QLabel("Steering angle")
		label.setStyleSheet('color: white')
		self.steering_label = QtGui.QTextEdit("0.0")
		self.steering_label.setFixedHeight(20)
		self.steering_label.setFixedWidth(200)
		lower_middle_left_layout.addWidget(label)
		lower_middle_left_layout.addWidget(self.steering_label)
		label = QtGui.QLabel("Velocity")
		label.setStyleSheet('color: white')
		self.velocity_label = QtGui.QTextEdit("0.0")
		self.velocity_label.setFixedHeight(20)
		self.velocity_label.setFixedWidth(200)
		lower_middle_right_layout.addWidget(label)
		lower_middle_right_layout.addWidget(self.velocity_label)
		#Set layouts.
		layout = QtGui.QGridLayout()
		layout.addLayout(upper_layout, 0, 0, 2, 4)
		layout.addLayout(lower_left_layout, 2, 0)
		layout.addLayout(lower_middle_left_layout, 2, 1)
		layout.addLayout(lower_middle_right_layout, 2, 2)
		layout.addLayout(lower_right_layout, 2, 3)
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
		#Controls.
		self.ros_interface.controls_signal.connect(self.updateControls)
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

	def updateControls(self, controls):
		self.steering_label.setText(str(controls[0]))
		self.velocity_label.setText(str(controls[1]))

	def updateGrid(self, cones):
		#Get cone position.
		for element in cones:
			self.cones_x.append(element[0])
			self.cones_y.append(element[1])
		#Add cone points to graph.
		self.plotcurve.addPoints(self.cones_x, self.cones_y, symbol='x', pen=QtGui.QPen(QtGui.QColor(255, 255, 255)))

	def updatePosition(self, position):
		#Get car position and add points to graph.
		self.graph_points_x.append(position[0])
		self.graph_points_y.append(position[1])
		self.plotcurve.addPoints(self.graph_points_x, self.graph_points_y, symbol='o', pen=QtGui.QPen(QtGui.QColor(255, 0, 0)))

	def updateTrajectory(self, listx, listy):
		#Replot graph.
		self.plotcurve.clear()
		self.plotcurve.addPoints(listx, listy, symbol='o', pen=QtGui.QPen(QtGui.QColor(0, 255, 0)))
		self.plotcurve.addPoints(self.graph_points_x, self.graph_points_y, symbol='o', pen=QtGui.QPen(QtGui.QColor(255, 0, 0)))
		self.plotcurve.addPoints(self.cones_x, self.cones_y, symbol='x', pen=QtGui.QPen(QtGui.QColor(255, 255, 255)))

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
