import socket
import sys

#TCP_IP = '169.254.10.85'
TCP_IP = '10.0.0.3' 
TCP_PORT = 8080
BUFFER_SIZE = 1024
MESSAGE = '33'


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
print('Client.py successfully connected')

py_ver = sys.version_info[0]
if (py_ver == 2):
	s.send(bytes(MESSAGE))
elif (py_ver == 3):
	s.send(bytes(MESSAGE, 'utf-8'))

while 1:
    data = s.recv(BUFFER_SIZE)
    if not data:
        print('Disconnected!')
        break
    print ("Client.py: Received ", data)
    #determine what cmd is being requested
    
s.close()
