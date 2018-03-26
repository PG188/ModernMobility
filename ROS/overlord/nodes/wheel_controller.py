#!/usr/bin/env python

import roslib
import rospy
import sys
from commands import *
from std_msgs.msg import Bool, Float32, UInt16, Int16, Int8
import struct

class wheel_controller:
    def __init__(self):
        #State init
        self.use_slider = False
        self.use_freeroll = False
        #Initial state of arduino should be freeroll
        self.l_vel = 0
        self.r_vel = 0

        #Pub/Sub init
        self.use_freeroll_sub = rospy.Subscriber("use_freeroll", Bool, self.use_freeroll_cb)
        self.use_slider_sub = rospy.Subscriber("use_slider", Bool, self.use_slider_cb)
        self.r_vtarget_sub = rospy.Subscriber("rwheel_vtarget", Float32, self.right_vel_cb)
        self.l_vtarget_sub = rospy.Subscriber("lwheel_vtarget", Float32, self.left_vel_cb)
        self.l_slider_sub = rospy.Subscriber("lslider_pos", UInt16, self.left_slider_cb)
        self.r_slider_sub = rospy.Subscriber("rslider_pos", UInt16, self.right_slider_cb)
        self.r_encoder_serial_sub = rospy.Subscriber("r_encoder_serial", Int16, self.right_encoder_cb)
        self.l_encoder_serial_sub = rospy.Subscriber("l_encoder_serial", Int16, self.left_encoder_cb)

        self.r_vel_pub = rospy.Publisher("r_vel_serial", Int8, queue_size = 1000)
        self.l_vel_pub = rospy.Publisher("l_vel_serial", Int8, queue_size = 1000)
        self.r_encoder_pub = rospy.Publisher("rightEncoder", Int16, queue_size = 1000)
        self.l_encoder_pub = rospy.Publisher("leftEncoder", Int16, queue_size = 1000)

    def use_freeroll_cb(self, msg):
        self.use_freeroll =  msg.data
        if self.use_freeroll:
            self.change_vel(FREEROLL_FLAG, FREEROLL_FLAG)

    def use_slider_cb(self, msg):
        self.use_slider = msg.data

    def right_vel_cb(self, msg):
        if not self.use_freeroll and not self.use_slider:
            self.change_r_vel(msg.data)

    def left_vel_cb(self, msg):
        if not self.use_freeroll and not self.use_slider:
            self.change_l_vel(msg.data)

    def left_slider_cb(self, msg):
        if self.use_slider:
            self.change_l_vel(wheel_controller.get_slider_vel(msg.data))

    def right_slider_cb(self, msg):
        if self.use_slider:
            self.change_r_vel(wheel_controller.get_slider_vel(msg.data))

    def right_encoder_cb(self, msg):
    	#rospy.loginfo("[wheel controlller] right encoder received:{}".format(msg.data))
        self.r_encoder_pub.publish(msg)

    def left_encoder_cb(self, msg):
    	#rospy.loginfo("[wheel controlller] left encoder received:{}".format(msg.data))
        self.l_encoder_pub.publish(msg)

    def change_vel(self, rightVel, leftVel):
        self.change_r_vel(rightVel)
        self.change_l_vel(leftVel)

    def change_r_vel(self, rightVel):
        self.r_vel =  rightVel
    	rospy.loginfo("[wheel controlller] Right vel out:{}".format(rightVel))
        self.r_vel_pub.publish(Int8(wheel_controller.vel_meter_to_cm(rightVel)))

    def change_l_vel(self, leftVel):
        self.l_vel =  leftVel
        rospy.loginfo("[wheel controlller] Left vel out:{}".format(leftVel))
        # rospy.loginfo("[Wheel Controller] Vel: {}  Int out:{}".format(leftVel, wheel_controller.vel_meter_to_cm(leftVel)))
        # rospy.loginfo("Int8: {}".format(Int8(wheel_controller.vel_meter_to_cm(leftVel))))
        self.l_vel_pub.publish(Int8(wheel_controller.vel_meter_to_cm(leftVel)))

    @staticmethod
    def vel_meter_to_cm(flt):
        if flt >= 2.5:
            return 125
        elif flt <= -2.5:
            return -125
        else:
            return int(round(100*flt))/2

    @staticmethod
    def get_slider_vel(slider_val):
        if slider_val <= 407:
            return MAX_SLIDER_VEL*(slider_val/408 - 1)
        elif 408 <= slider_val <= 615:
            return 0 #Free roll?
        else:
            return MAX_SLIDER_VEL*(slider_val - 615)/408

    @staticmethod
    def ROS_INFO(str):
        rospy.loginfo(str) 

if __name__ == '__main__':
    rospy.init_node('wheel_controller')
    ne = wheel_controller()
    try:
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass 
