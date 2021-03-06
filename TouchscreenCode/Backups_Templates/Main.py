import sys
 
# gets the Qt stuff
import PyQt5
from PyQt5.QtWidgets import *
 
#window from QtCreator
import mainwindow_auto
 
# create class for our Raspberry Pi GUI
class MainWindow(QMainWindow, mainwindow_auto.Ui_MainWindow):
 # access variables inside of the UI's file

 # getter method for brkmode
 def getBrakeMode(self):
     return self.brkmode
 
 ### functions for the buttons to call
 def BrakeMode(self):
     #print ("Brake Mode On!")
     self.brkmode = 0;
     #print (self.brkmode)
 
 def FreeRolling(self):
     #print ("Free Rolling Mode On!")
     self.brkmode = 1;
     #print (self.brkmode)
 
 def __init__(self):
     super(self.__class__, self).__init__()
     self.setupUi(self) # gets defined in the UI file
     self.brkmode = 1
 
 ### Hooks for buttons
     self.BrakeModeBtn.clicked.connect(lambda: self.BrakeMode(self.brkmode))
     self.FreeRollingBtn.clicked.connect(lambda: self.FreeRolling(self.brkmode))
 
def main():
 # a new app instance
 app = QApplication(sys.argv)
 form = MainWindow()
 form.show()

 print(form.getBrakeMode())
 
 # without this, the script exits immediately.
 sys.exit(app.exec_())
 
# python bit to figure how who started This
if __name__ == "__main__":
 main()
