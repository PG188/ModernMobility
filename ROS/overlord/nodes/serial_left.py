#!/usr/bin/env python

import sys
import rospy
import roslib
import array
import time
from std_msgs.msg import Int16, Int8
import serial
from struct import pack, unpack

class serial_left:
	def __init__(self):
		try:
			#Serial init
			self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0)
			time.sleep(1)

			#Ros init
			self.encoder_pub = rospy.Publisher('l_encoder_serial', Int16, queue_size = 1000)
			self.vel_sub = rospy.Subscriber('l_vel_serial', Int8, self.vel_out_cb)
			self.rate = rospy.Rate(100)

			#Misc init
			start_flag = serial_left.int2byte(127)
			stop_flag = serial_left.int2byte(126)
			read_byte = b''

			self.ser.write(stop_flag)
			time.sleep(0.3)
			self.ser.flushInput()
			time.sleep(0.1)
			self.ser.write(start_flag)

			while not rospy.is_shutdown():
				read_byte += self.ser.read(1)
				if len(read_byte) == 2:
					self.encoder_pub.publish(serial_left.bytes2int(read_byte))
					read_byte = b''
				self.rate.sleep()
		except serial.SerialException:
			rospy.loginfo("[Serial Left] Could not find the port")

	def vel_out_cb(self, msg):
		# rospy.loginfo("[Serial] Serial vel out: {} Writing: {}".format(msg.data, serial_left.int2byte(msg.data)))
		self.ser.write(serial_left.int2byte(msg.data))

	@staticmethod
	def bytes2int(val):
		return unpack("h", val)[0]

	@staticmethod
	def int2byte(val):
		return bytes(pack("b", val))


if __name__ == 	'__main__':
	rospy.init_node('serial_left')
	ne = serial_left()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

