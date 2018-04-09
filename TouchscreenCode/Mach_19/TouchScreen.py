#!/usr/bin/env python

import array
import time
import sys
import serial
import time
from struct import pack, unpack
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
		print ("Sending assisted flag")
		self.ser.write(self.assisted_flag)

	def FreeRolling(self,brkmode):
		print ("Sending freeroll flag")
		self.ser.write(self.freeroll_flag)

	def __init__(self):
		super(self.__class__, self).__init__()
		self.setupUi(self) # gets defined in the UI file
		self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0)
		time.sleep(1)

		### Hooks for buttons
		self.BrakeModeBtn.clicked.connect(lambda: self.BrakeMode(self.brkmode))
		self.FreeRollingBtn.clicked.connect(lambda: self.FreeRolling(self.brkmode))
		### Brake mode variable
		self.brkmode = ""
		self.assisted_flag = pack('B', 0)
		self.freeroll_flag = pack('B', 1)
		self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0)

def TouchScreen():
	#global pub
	# a new app instance
	app = QApplication(sys.argv)
	form = MainWindow()
	form.show()
	# without this, the script exits immediately.
	sys.exit(app.exec_())

if __name__ == 	'__main__':
	TouchScreen()


