#!/usr/bin/env python

import sys

# gets the Qt stuff
from PyQt5.QtWidgets import (QWidget, QApplication, QPushButton)
from PyQt5.QtGui import QFont

class Example(QWidget):
	def __init__(self):
		super(self.__class__, self).__init__()
		self.initGUI()
	
	def initGUI(self):
		button = QPushButton('Button',self)
		button.move(50,50)

		self.resize(250,150)
		self.move(100,100)
		self.show()

app = QApplication(sys.argv)
ex = Example()
sys.exit(app.exec_())

if Example.isEnabled:
    print("Enabled")
else:
    print("Disabled")
