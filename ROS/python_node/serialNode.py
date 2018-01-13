#!/usr/bin/env python

import rospy
import serial

from std_msgs.msg import ByteMultiArray

def callback(data):
	data	

def serialNode():
	ser = serial.Serial('/dev/ttyNAME', 19200)
	pub = rospy.Publisher('USB_out', ByteMultiArray, queue_size = 1000)
	rospy.init_node('serialNode', anonymous=True)
	rospy.Subscriber("USB_in", ByteMultiArray, callback)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		//publishing bytemultiarray
		pub.publish()
		if ser.in_waiting > 12
			ser.read(12)

if __name__ == 	'__main__'	
	try:
		serialNode()
	except rospy.ROSInterruptException:
		pass


