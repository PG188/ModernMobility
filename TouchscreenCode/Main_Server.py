import sys
 
# gets the Qt stuff
import PyQt5
import socket

from PyQt5.QtWidgets import *
 
#window from QtCreator
import mainwindow_auto

import threading

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
        self.connW = None

m = Tmanager()

def sendbyte(msg):
        m.connW.send((msg).encode())  # echo
    
 # create class for our Raspberry Pi GUI
class MainWindow(QMainWindow, mainwindow_auto.Ui_MainWindow):
 # access variables inside of the UI's file

 # getter method for brkmode
    def getMode(self):
        return self.mode
 
 ### functions for the buttons to call
    def AssistMode(self):
        self.mode = 0;
        if m.connW is None:
           print('MainServer.py: ERROR:  Client not connected, please ensure Client.py is running and has made a connection')
        else:
           sendbyte('0')
           #print('Main_Server.py: Sent Manual Assisted Mode Command (0)')
     
    def FreeRolling(self):
        self.mode = 1;
        if m.connW is None:
            print('MainServer.py: ERROR:  Client not connected, please ensure Client.py is running and has made a connection')
        else:
            sendbyte('1')
            #print('Main_Server.py: Sent Manual Free Rolling Mode Command (1)')

    def Disconnect(self):
        if m.connW is None:
            print('MainServer.py: ERROR:  Client not connected, please ensure Client.py is running and has made a connection')
        else:
            sendbyte('-2')
            try:
                if not (m.connP is None):
                    m.connP.close()
                m.connW.close()
                s.close()
                self.close()
                
            except:
                print('Main_Server.py: ERROR with disconnect')
     
    def __init__(self):
         super(self.__class__, self).__init__()
         self.setupUi(self) # gets defined in the UI file
         self.mode = 1
     
     ### Hooks for buttons
         self.AssistModeBtn.clicked.connect(lambda: self.AssistMode())
         self.FreeRollingBtn.clicked.connect(lambda: self.FreeRolling())
         self.DisconnectBtn.clicked.connect(lambda: self.Disconnect())

# a new app instance
app = QApplication(sys.argv)
form = MainWindow()
form.show()

def newConnection(s, m, t):
    if t == 'walker':
        print('Main_Server.py: Listening for walker connection...\n')
        m.connW, addrW = s.accept()
        print('Main_Server.py: Walker connected!')
        print('Main_Server.py: Walker TCP client connection address: ' + str(addrW))
        sendbyte('-3')

    elif t == 'phone':
        print('Main_Server.py: Listening for phone connection...\n')
        m.connP, addrP = s.accept()
        print('Main_Server.py: TCP Client for Phone connected!')
        print ('Main_Server.py: Phone TCP client connection address: ' + str(addrP))
        while 1:
            data = m.connP.recv(BUFFER_SIZE).decode()
            print('Main_Server.py: data received')
            if not data or (data == -1):
                print('Main_server.py: Phone disconnected!')
                sendbyte('-1')
                m.connP.close()
                break
            print ('\nMain_Server.py: Received data: '  + str(data))
            sendbyte(data)
            print('Main_server.py: Sent data: ' + str(data))

    else:
        print('Main_Server.py: Unknown connection attempted')
        
try: 
    tw = threading.Thread(target=newConnection, args=(s, m, 'walker'))
    tw.start()
    tp = threading.Thread(target=newConnection, args=(s, m, 'phone'))
    tp.start()

except e as Exception:
    print('Main_Server.py: ' + str(e))

    m.connP.close()
    m.connW.close()
    s.close()
    form.close()

def main():
 
 # without this, the script exits immediately.
    sys.exit(app.exec_())
    m.connP.close()
    m.connW.close()
    s.close()
 
# python bit to figure how who started This
if __name__ == "__main__":
    main()
