import socket
import sys
import time

import rospy
from std_msgs.msg import Int8

#py_ver = sys.version_info[0]


pubScreen = rospy.Publisher('screen_cmd', Int8, queue_size=10)
pubPhone = rospy.Publisher('phone_cmd', Int8, queue_size=10)
rospy.init_node('hmi_cmds', anonymous=True)

def interpretCmd(data):
        if data == '0' or data == b'0':
                print('Client.py: Manual: Free Rolling Mode (0)\n')
                #Publish 1 to screen_cmd topic
                pubScreen.publish(Int8(1))
        elif data == '1' or data == b'1':
                print('Client.py: Manual: Brake Mode (1)\n')
                #Publish 0 to screen_cmd topic
                pubScreen.publish(Int8(0))
        elif data == '2' or data == b'2':
                print('Client.py: Autonomous: Cancel Command (2)\n')
                #Publish 0 to phone_cmd topic
                pubPhone.publish(Int8(0))
        elif data == '3' or data == b'3':
                print('Client.py: Autonomous: Go To Dock Command (3)\n')
                #Publish 1 to phone_cmd topic
                pubPhone.publish(Int8(1))
        elif data == '4' or data == b'4':
                print('Client.py: Autonomous: Park Command (4)\n')
                #Publish 2 to phone_cmd topic
                pubPhone.publish(Int8(2))
        elif data == '5' or data == b'5':
                print('Client.py: Autonomous: Extra Command (5)\n')
                #Publish 3 to phone_cmd topic
                pubPhone.publish(Int8(3))
        elif data == '6' or data == b'6':
                print('Client.py: Autonomous: Cancel Command (6)\n')
                #Publish4 to phone_cmd topic
                pubPhone.publish(Int8(4))
        elif data == '-1':
                print('Client.py: Phone disconnected!\n')
        elif data == '-2':
                print('Client.py: Touch Screen and Phone disconnected!\n')
        elif int(data) <= -3:
                pass
        else:
                print('Client.py: Invalid data Received\n' + str(data))

TCP_IP = '10.0.0.3'
#TCP_IP = '127.0.0.1'
TCP_PORT = 8080
BUFFER_SIZE = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

while 1:
        try:
                s.connect((TCP_IP, TCP_PORT))
                if s.recv(BUFFER_SIZE) == '-3':
                        print('Client.py: Successfully connected to TCP server\n')
                        break
        except ConnectionRefusedError:
                print('Client.py: Cannot find connection , retrying in 5 seconds...')
                time.sleep(5)
                continue
        
        except OSError:
                print('Client.py: Successfully connected to TCP server\n')
                break

s.send(('-9').encode()) #So Main_Server.py knows this connection is the walker
                
while 1:
        data = s.recv(BUFFER_SIZE).decode()
        print('data: ' + str(data))
        try:  
                if not data:
                        print('Client.py: ERROR: Disconnected from TCP server!\n')
                        break
                if not rospy.is_shutdown():
                        interpretCmd(data)

        except Exception as e:
                print(e)
                print('Client.py: ERROR with disconnect')
                break
    
s.close()
