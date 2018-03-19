#!/usr/bin/env python
import roslib
import rospy
import sys
from commands import *
from std_msgs.msg import Bool, Int8

class overlord:
    def __init__(self):
        #State init
        self.state = MANUAL_FREEROLL
        self.use_slider = False
        self.use_freeroll = True

        #Pub/Sub init
        self.phone_cmd_sub = rospy.Subscriber("phone_cmd", Int8, self.phone_cmd_cb)
        self.screen_cmd_sub = rospy.Subscriber("screen_cmd", Int8, self.screen_cmd_cb)
        self.slider_moved_sub = rospy.Subscriber("slider_moved", Bool, self.slider_moved_cb)
        self.nav_status_sub = rospy.Subscriber("nav_status", Bool, self.nav_status_cb)

        self.use_freeroll_pub = rospy.Publisher("use_freeroll", Bool, queue_size = 1000)
        self.use_slider_pub = rospy.Publisher("use_slider", Bool, queue_size = 1000)
        self.nav_cmd_pub = rospy.Publisher("navigation_command", Int8, queue_size = 1000)

    def phone_cmd_cb(self, cmd):
        if (self.state != AUTO_NAV) and (cmd.data != CANCEL_NAV):
            self.nav_cmd_pub.publish(cmd)
            self.state_to_auto_nav()
        elif (self.state == AUTO_NAV) and (cmd.data == CANCEL_NAV):
            self.nav_cmd_pub.publish(cmd)

    def screen_cmd_cb(self, cmd):
        if self.state == AUTO_NAV:
            sendCancelNav()
            self.state_to_freeroll()
        elif (self.state == MANUAL_FREEROLL) and (cmd.data == ASSISTED):
            self.state_to_assisted()
        elif (self.state == MANUAL_ASSISTED) and (cmd.data == FREE_ROLL):
            self.state_to_freeroll()

    def slider_moved_cb(self, moved):
        if self.state == AUTO_NAV:
            sendCancelNav()
            self.state_to_freeroll()

    def nav_status_cb(self, nav_done):
        overlord.ROS_INFO('[Overlord] Nav status arrived')
        if nav_done.data and self.state == AUTO_NAV:
            self.state_to_freeroll()

    def state_to_freeroll(self):
        overlord.ROS_INFO('Overlord state to FREEROLL')
        self.state = MANUAL_FREEROLL
        self.modifyState(False, True)

    def state_to_assisted(self):
        overlord.ROS_INFO('Overlord state to ASSISTED')
        self.state = MANUAL_ASSISTED
        self.modifyState(True, False)

    def state_to_auto_nav(self):
        overlord.ROS_INFO('Overlord state to AUTO_NAV')
        self.state = AUTO_NAV
        self.modifyState(False, False)

    def modifyState(self, slider, freeroll):
        overlord.ROS_INFO('Slider:{}    Freeroll:{}\n'.format(slider, freeroll))
        self.use_slider = slider
        self.use_slider_pub.publish(self.use_slider)
        self.use_freeroll = freeroll
        self.use_freeroll_pub.publish(self.use_freeroll)

    def sendCancelNav(self):
        cmd = Int8()
        cmd.data = CANCEL_NAV
        self.nav_cmd_pub.publish(cmd)

    @staticmethod
    def ROS_INFO(str):
        rospy.loginfo(str) 

if __name__ == '__main__':
    rospy.init_node('overlord')
    ne = overlord()
    try:
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass 