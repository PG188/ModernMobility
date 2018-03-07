#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String

def SerialOutCallback(msg):
	ser.write(msg.data)

def serialNode():
	ser = serial.Serial('/dev/ttyACM0', 19200)
	rospy.init_node('serialNode', anonymous=True)
	#Define Publisher
	pub = rospy.Publisher('SerialIn', String, queue_size = 12)
	sub = rospy.Subscriber('SerialOut', String, SerialOutCallback)
	rate = rospy.Rate(10)
	a=String()
	#ser.flushInput()
	while not rospy.is_shutdown():
		if ser.in_waiting >= 12:
			a.data = str(ser.read(12)) 
			pub.publish(a)		#publishing serial bytemultiarray
			rate.sleep()
if __name__ == 	'__main__':
	try:
		serialNode()
	except rospy.ROSInterruptException:
		pass


