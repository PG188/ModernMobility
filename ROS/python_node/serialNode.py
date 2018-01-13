#!/usr/bin/env python

import rospy
import serial

from std_msgs.msg import ByteMultiArray

def callback(msg):
	ser.write(msg)	#writing subscribed serial bytemultiarray

def serialNode():
	ser = serial.Serial('/dev/ttyNAME', 19200)
	readSerial = bytearray()
	
	#Define Publisher
	pub = rospy.Publisher('USB_toROS', ByteMultiArray, queue_size = 1000)
	
	rospy.init_node('serialNode', anonymous=True)

	#Define Subscriber
	rospy.Subscriber("USB_toArd", ByteMultiArray, callback)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if ser.in_waiting > 12
			readSerial = ser.read(12)
			pub.publish(readSerial)		#publishing serial bytemultiarray

if __name__ == 	'__main__'	
	try:
		serialNode()
	except rospy.ROSInterruptException:
		pass


