#!/usr/bin/env python

import rospy
import roslib
import array
import time
from std_msgs.msg import UInt8
import serial
from struct import unpack

class serial_GUI_in:
	def __init__(self):
		try:
			#Serial init
			self.ser = serial.Serial('/dev/ttyACM2', baudrate=115200, timeout=0)
			time.sleep(1)

			#Ros init
			self.screen_cmd_pub = rospy.Publisher('screen_cmd', UInt8, queue_size = 1000)
			self.rate = rospy.Rate(100)

			#Misc init
			read_byte = b''

			self.ser.flushInput()
			while not rospy.is_shutdown():
				read_byte += self.ser.read(1)
				if len(read_byte) > 0:
					self.screen_cmd_pub.publish(serial_GUI_in.bytes2int(read_byte))
					read_byte = b''
				self.rate.sleep()
				
		except serial.SerialException:
			rospy.loginfo("[serial_GUI_in] Could not find the port")

	@staticmethod
	def bytes_to_char(val):
		return unpack("B", val)[0]

if __name__ == 	'__main__':
	rospy.init_node('serial_GUI_in')
	ne = serial_GUI_in()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass