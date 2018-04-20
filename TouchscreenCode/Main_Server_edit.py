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

#set up connecion
#TCP_IP = '10.0.0.3'  
TCP_IP = '127.0.0.1' 
TCP_PORT = 8080
BUFFER_SIZE = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt (socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

class Tmanager:
    def __init__(self):
        self.connP = None
        self.connT = None

def sendbyte(msg):
    m.connT.send((msg).encode())  # echo
    
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


def newConnection(s, m, t):

    if t == 'touchscreen':
        print('\n\nMain_Server.py: Listening for touchscreen connection...')
        m.connT, addrT = s.accept()
        print('\nMain_Server.py: Touchscreen connected!')
        print ('\nMain_Server.py: Connection address: ', addrT)

        while 1:
            print('\n\nMain_Server.py: still alive')
            data = m.connT.recv(BUFFER_SIZE).decode()
            print('\nMain_Server.py: data received')
            if not data:
                print('Touchscreen disconnected!')
                break
            print ("\nMain_Server.py: received data: ", data)
            sendbyte(data)
            print('Sent data: ' + str(data))

    elif t == 'phone':
        print('\n\nMain_Server.py: Listening for phone connection...')
        m.connP, addrP = s.accept()
        print('\nMain_Server.py: Phone connected!')
        print ('\nMain_Server.py: Connection address: ', addrP)

        while 1:
            print('\n\nMain_Server.py: still alive')
            data = m.connP.recv(BUFFER_SIZE).decode()
            print('\nMain_Server.py: data received')
            if not data:
                print('Phone disconnected!')
                break
            print ("\nMain_Server.py: received data: ", data)
            sendbyte(data)
            print('Sent data: ' + str(data))

    else:
        print('Main_Server.py: Unknown connection attempted')

m = Tmanager()

try: 
    tt = threading.Thread(target=newConnection, args=(s, m, 'touchscreen'))
    tt.start()
    tp = threading.Thread(target=newConnection, args=(s, m, 'phone'))
    tp.start()

except e as Exception:
    print('Main_Server.py: ' + str(e))

#s.close()

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
