#!/usr/bin/env python

import socket
import sys

import rospy
from std_msgs.msg import int8

#py_ver = sys.version_info[0]


pubScreen = rospy.Publisher('screen_cmd', String, queue_size=10)
pubPhone = rospy.Publisher('phone_cmd', String, queue_size=10)
rospy.init_node('hmi_cmds', anonymous=True)

def interpretCmd(data):
        if data == '0':
                print('Client.py: Manual: Free Rolling Mode (0)\n')
                #Publish 1 to screen_cmd topic
                pubScreen.publish(1)
        elif data == '1':
                print('Client.py: Manual: Brake Mode (1)\n')
                #Publish 0 to screen_cmd topic
                pubScreen.publish(0)
        elif data == '2':
                print('Client.py: Autonomous: Cancel Command (2)\n')
                #Publish 0 to phone_cmd topic
                pubPhone.publish(0)
        elif data == '3':
                print('Client.py: Autonomous: Go To Dock Command (3)\n')
                #Publish 1 to phone_cmd topic
                pubPhone.publish(1)
        elif data == '4':
                print('Client.py: Autonomous: Park Command (4)\n')
                #Publish 2 to phone_cmd topic
                pubPhone.publish(2)
        elif data == '5':
                print('Client.py: Autonomous: Extra Command (5)\n')
                #Publish 3 to phone_cmd topic
                pubPhone.publish(3)
        elif data == '6':
                print('Client.py: Autonomous: Cancel Command (6)\n')
                #Publish4 to phone_cmd topic
                pubPhone.publish(4)
        elif data == '-1':
                print('Client.py: Phone disconnected!\n')
        elif data == '-2':
                print('Client.py: Touch Screen and Phone disconnected!\n')
        else:
                print('Client.py: Invalid data Received\n')

#TCP_IP = '10.0.0.3'
TCP_IP = '127.0.0.1'
TCP_PORT = 8080
BUFFER_SIZE = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
print('Client.py: Successfully connected to TCP server\n')

while 1:
        data = s.recv(BUFFER_SIZE)
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
