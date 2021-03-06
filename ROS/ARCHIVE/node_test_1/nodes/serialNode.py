#!/usr/bin/env python

import rospy
import serial
import array
import time
from std_msgs.msg import String


ready_to_read = True

def SerialOutCallback(msg):
	ser.write(msg.data)

def serialNode():
	ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0)
	rospy.init_node('serialNode', anonymous=True)
	#Define Publisher
	pub = rospy.Publisher('SerialIn', String, queue_size = 1000)
	#Define Subscriber
	sub = rospy.Subscriber('SerialOut', String, SerialOutCallback)
	rate = rospy.Rate(1000)
	bytecount = 0
	readserial = String()
	writeserial = String()
	writeserial.data = ''
	NotStartFlag = True;
	readByte = 0;
	ser.write(b'Z') #Stop command
	while NotStartFlag: #Wait for arduino to acknowledge the stop
		if ser.in_waiting > 0:
			readByte = ser.read(1)
			if readByte == b'Z':
                                NotStartFlag = False
	ser.flushInput()
	ser.write(b'A')#Start command
	while not rospy.is_shutdown():
		while ser.in_waiting:
			writeserial.data += str(ser.read(1))
			bytecount += 1
			if bytecount >= 12:
				pub.publish(writeserial)
				bytecount = 0
				writeserial.data = ''
			msg_str = "ser.in_waiting = %d" %ser.in_waiting
			rospy.loginfo(msg_str)
		rate.sleep()
if __name__ == 	'__main__':
	try:
		serialNode()
	except rospy.ROSInterruptException:
		pass


