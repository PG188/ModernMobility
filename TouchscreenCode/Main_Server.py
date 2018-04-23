import sys
 
# gets the Qt stuff
import PyQt5
import socket
import time

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
        self.connFirst = None
        self.connSecond = None
        self.connP = None
        self.connW = None
        self.First = None
        self.firstIsConnected = False

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

def walkerConn(conn, addr):
    if not m.firstIsConnected:
        m.firstIsConnected = True
    m.connW = conn
    print('Main_Server.py: Walker TCP client connection address: ' + str(addr))
    #sendbyte('-3')

def phoneConn(conn, addr):
    if not m.firstIsConnected:
        m.firstIsConnected = True
    m.connP = conn
    print ('Main_Server.py: Phone TCP client connection address: ' + str(addr))
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

def newConnection(s, m, t):
    if t == 'first':
        print('Main_Server.py: Identifying first connection...')
        m.connFirst, addrFirst = s.accept()
        m.connFirst.send(('-3').encode())
        data = m.connFirst.recv(BUFFER_SIZE).decode()
        if data == '-9':
            m.First= 'Walker'
            print('Main_Server.py: Walker connected first!')
            walkerConn(m.connFirst, addrFirst)
        elif data == '-8':
            m.First= 'Phone'
            print('Main_Server.py: Phone connected first!')
            phoneConn(m.connFirst, addrFirst)
        else:
            print('Main_Server.py: Unexpected data in first thread')
    elif t == 'second':
        print('Main_Server.py: Identifying second connection...')
        m.connSecond, addrSecond = s.accept()
        if m.First == 'Phone':
            print('Main_Server.py: Walker connected second!')
            walkerConn(m.connSecond, addrSecond)
        elif m.First == 'Walker':
            print('Main_Server.py: Phone connected second!')
            phoneConn(m.connSecond, addrSecond)
        else:
            print('Main_Server.py: Unexpected data in second thread')
    else:
        print('This should be impossible to see')

def main():
    # a new app instance
    app = QApplication(sys.argv)
    form = MainWindow()
    form.show()
    sys.exit(app.exec_())
 
     # without this, the script exits immediately.
    sys.exit(app.exec_())
    m.connP.close()
    m.connW.close()
    s.close()
 
### python bit to figure how who started This
##if __name__ == "__main__":
##    main()

try:
    mainT = threading.Thread(target=main, args=())
    mainT.start()
    t1 = threading.Thread(target=newConnection, args=(s, m, 'first'))
    t1.start()
    while not m.firstIsConnected:
        time.sleep(1)
    t2 = threading.Thread(target=newConnection, args=(s, m, 'second'))
    t2.start()

except e as Exception:
    print('Main_Server.py: ' + str(e))

    m.connP.close()
    m.connW.close()
    s.close()
    form.close()
