#!/usr/bin/env python

import rospy
import array
import time
from std_msgs.msg import String
import serial
import struct

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0)

def SerialOutCallback(msg):
	global ser
	ser.write(bytes(msg.data))
	#rospy.loginfo("velCmd")

def serialNode():
	global ser
	time.sleep(2) #Delay to allow serial comms to open up
	pub = rospy.Publisher('leftEncoder_SerialIn', String, queue_size = 1000)
	sub = rospy.Subscriber('leftMotorVel_SerialOut', String, SerialOutCallback)
	rospy.init_node('serialNode_motor_left', anonymous=True)
	StartVal = struct.pack("b",127)
	StopVal =  struct.pack("b",126)
	StopValReturn = bytes(struct.pack("h", 32767))
	readByte = b''
	ser.write(StopVal) #Send stop command
	time.sleep(0.3)
	ser.flushInput()
	time.sleep(0.1)
	ser.write(StartVal) #Start command
	
	rate = rospy.Rate(100)
	#Process incoming encoder values
	while not rospy.is_shutdown():
		readByte += ser.read(1)
		if len(readByte) == 2:
			pub.publish(str(readByte))
			readByte = b''
		rate.sleep()
if __name__ == 	'__main__':
	try:
		serialNode()
	except rospy.ROSInterruptException:
		pass


