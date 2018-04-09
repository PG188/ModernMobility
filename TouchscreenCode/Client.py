import socket

#TCP_IP = '169.254.10.85'
TCP_IP = '172.17.50.37' 
TCP_PORT = 8080
BUFFER_SIZE = 1024


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

#s.send(bytes(MESSAGE, 'utf-8')))
#s.send(MESSAGE)
while 1:
    data = s.recv(BUFFER_SIZE)
    if not data: break
    print ("Client.py: Received ", data)
    
s.close()



