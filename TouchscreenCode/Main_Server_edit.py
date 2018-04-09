import sys
 
# gets the Qt stuff
import PyQt5
import socket

import threading
##import freeport
##from psutil import process_iter
##from signal import SIGTERM

from PyQt5.QtWidgets import *

#bluetooth communication for phone commands
#import BTReceive
 
#window from QtCreator
import mainwindow_auto


##for proc in process_iter():
##    for conns in proc.connections(kind='inet'):
##        if conns.laddr.port == 8080:
##            proc.send_signal(SIGTERM)
##            continue

def newConnection(conn, addr):
    print("Main_Server.py: newConnection")
    print ('Main_Server.py: Connection address: ', addr)

#set up connecion
TCP_IP = '172.17.50.37' 
TCP_PORT = 8080
BUFFER_SIZE = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt (socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((TCP_IP, TCP_PORT))
s.listen(2)

while 1:
    print('Main_Server.py: Listening for connection...')   
    threading.Thread(target=newConnection, args=(s.accept())).start()
    print("Main_Server.py: You in Trouble")
    
##print('Main_Server.py: Listening for Touchscreen...')
##conn1, addr1 = s.accept()
##print('Main_Server.py: Connected to Touchscreen!')
##print ('Main_Server.py: Connection address for Touchscreen: ', addr)
##print('Main_Server.py: Listening for Android...')
##conn2, addr2 = s.accept()
##print('Main_Server.py: Connected to Android!')
##print ('Main_Server.py: Connection address for Android:', addr)

while 1:
    print('Main_Server.py: still alive')
    data = conn.recv(BUFFER_SIZE).decode()
    print('Main_Server.py: data received')
    if not data: break
    print ("Main_Server.py: received data: ", data)
    
s.close()

def sendbyte(msg):
    conn.send((msg).encode())  # echo
    
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
     sendbyte('0')
     print (self.brkmode)
 
 def FreeRolling(self):
     #print ("Free Rolling Mode On!")
     self.brkmode = 1;
     sendbyte('1')
     print (self.brkmode)
 
 def __init__(self):
     super(self.__class__, self).__init__()
     self.setupUi(self) # gets defined in the UI file
     self.brkmode = 1
 
 ### Hooks for buttons
     self.BrakeModeBtn.clicked.connect(lambda: self.BrakeMode())
     self.FreeRollingBtn.clicked.connect(lambda: self.FreeRolling())
    #self.BrakeModeBtn.clicked.connect(lambda: self.brkmode)
    #self.FreeRollingBtn.clicked.connect(lambda: self.brkmode)
 
def main():
 # a new app instance
  
    app = QApplication(sys.argv)
    form = MainWindow()
    form.show()

    #print(form.getBrakeMode())
    #conn.close()

 # without this, the script exits immediately.
    sys.exit(app.exec_())
 
# python bit to figure how who started This
if __name__ == "__main__":
 main()
