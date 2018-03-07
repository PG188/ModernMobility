#!/usr/bin/env python

import rospy
import array
import time
from std_msgs.msg import String
import sys

# gets the Qt stuff
import PyQt5
from PyQt5.QtWidgets import *

#window from QtCreator
import mainwindow_auto

# create class for our Raspberry Pi GUI
class MainWindow(QMainWindow, mainwindow_auto.Ui_MainWindow):

	def getBrakeMode(self):
		return self.brkmode

# access variables inside of the UI's file
	def BrakeMode(self,brkmode):
		#print ("Brake Mode On!")
		self.brkmode = "assisted"
		self.pub.publish(self.brkmode)
		print (self.brkmode)

	def FreeRolling(self,brkmode):
		#print ("Free Rolling Mode On!")
		self.brkmode = "freeroll"
		self.pub.publish(self.brkmode)
		print (self.brkmode)

	def __init__(self):
		super(self.__class__, self).__init__()
		self.setupUi(self) # gets defined in the UI file
		### Hooks for buttons
		self.BrakeModeBtn.clicked.connect(lambda: self.BrakeMode(self.brkmode))
		self.FreeRollingBtn.clicked.connect(lambda: self.FreeRolling(self.brkmode))
		### Brake mode variable
		self.brkmode = ""
		self.pub = rospy.Publisher('screen_cmd', String, queue_size = 1000)

def TouchScreen():
	#global pub
	#pub = rospy.Publisher('screen_cmd', String, queue_size = 1000)
	#sub = rospy.Subscriber('leftMotorVel_SerialOut', String, SerialOutCallback)
	rospy.init_node('TouchScreen', anonymous=True)
	# a new app instance
	app = QApplication(sys.argv)
	form = MainWindow()
	form.show()
	# without this, the script exits immediately.
	sys.exit(app.exec_())

if __name__ == 	'__main__':
	try:
		TouchScreen()
	except rospy.ROSInterruptException:
		pass


