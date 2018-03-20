#!/usr/bin/env python
#import sys
#sys.path.append("atDock_Dependencies")

import roslib
import rospy
import sys
import actionlib
from commands import *
from std_msgs.msg import Bool, Int8
from sensor_msgs.msg import Image, PointCloud2
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32
from tf.transformations import quaternion_from_euler
from atDock import *
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

class nav_controller():
    def __init__(self):
        self.goalArrived = False
        self.navigationDone = True

        self.nav_status_pub = rospy.Publisher("nav_status", Bool, queue_size = 1000)
        self.nav_cmd_sub = rospy.Subscriber("navigation_command", Int8, self.nav_cmd_cb)
        self.goal_sub = rospy.Subscriber("nav_goal", Point32,  self.receive_goal_cb)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.bridge = CvBridge()

    def nav_cmd_cb(self, nav_cmd):
        if nav_cmd.data == CANCEL_NAV and not self.navigationDone:
            self.cancelNavigation()
        elif nav_cmd.data == GO_TO_DOCK:
            nav_.ROS_INFO("[NAV CONTROLLER]: GO TO DOCK not implemented")
        elif nav_cmd.data == PARK_IN_DOCK:
            self.navigationDone = False
            result = self.parkInDock()
        elif (nav_cmd.data == GO_TO_TESTPOSE) or (nav_cmd.data == GO_TO_POSE):
            self.navigationDone = False
            while not self.goalArrived:
                pass
            self.goalArrived = False
            result = self.goToTestPose(self.goalReceived)
        self.navigationDone = True
        self.nav_status_pub.publish(self.navigationDone)

    def receive_goal_cb(self, goal):
        receivedPose = pose(goal.x, goal.y, goal.z)
        self.goalReceived = self.buildGoal(receivedPose)
        self.goalArrived = True

    def parkInDock(self):
        # data_out4symbol = list(pc2.read_points(nav_controller.getDepthFrame(), field_names=("x", "y", "z"), skip_nans=True, uvs=[[1, 1], [1, 2]]))
        # print(data_out4symbol)

        dockPose = atDock(self.bridge.imgmsg_to_cv2(nav_controller.getColorFrame(), desired_encoding="passthrough"), nav_controller.getDepthFrame())
        if dockPose is not None:
            result = self.client.send_goal_and_wait(self.buildAlignGoal(dockPose), rospy.Duration(10), rospy.Duration(10))
            if result == GoalStatus.SUCCEEDED:
                result = self.client.send_goal_and_wait(self.buildForwardGoal(dockPose), rospy.Duration(10), rospy.Duration(10))
                if result == GoalStatus.SUCCEEDED:
                    self.ROS_INFO("[NAV_CONTROLLER]: Driving into docking station succeeded")
                    return True
                else:
                    self.ROS_INFO("[NAV_CONTROLLER]: Driving into docking station failed!")
            else:
                self.ROS_INFO("[NAV_CONTROLLER]: Alignment with docking station failed")             
        else:
            self.ROS_INFO("[NAV_CONTROLLER]: Could not find the dock")
        return False

    def goToTestPose(self, pose):
        self.ROS_INFO("[NAV CONTROLLER]: Navigating to point")
        result = self.client.send_goal_and_wait(pose, rospy.Duration(10), rospy.Duration(10))
        if result == GoalStatus.SUCCEEDED:
            self.ROS_INFO("[NAV_CONTROLLER]: Driving to test point succeeded!")
            return True         
        else:
            self.ROS_INFO("[NAV_CONTROLLER]: Driving to test point failed!")
            return False

    def cancelNavigation(self):
        self.ROS_INFO("[NAV CONTROLLER]: Cancelling navigation")
        self.client.cancelGoal()

    def buildAlignGoal(self, overallGoal):
        poseAlign = pose(overallGoal.x, 0, overallGoal.theta)
        return self.buildGoal(poseAlign)

    def buildForwardGoal(self, overallGoal):
        poseForward = pose(overallGoal.y, 0, 0)
        return self.buildGoal(poseForward)

    def buildGoal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose.x
        goal.target_pose.pose.position.y = pose.y
        goal.target_pose.pose.position.z = 0
        poseQuat = quaternion_from_euler(0, 0, pose.theta)
        goal.target_pose.pose.orientation.x = poseQuat[0]
        goal.target_pose.pose.orientation.y = poseQuat[1]
        goal.target_pose.pose.orientation.z = poseQuat[2]
        goal.target_pose.pose.orientation.w = poseQuat[3]
        return goal

    @staticmethod
    def ROS_INFO(str):
        rospy.loginfo(str) 

    @staticmethod
    def getDepthFrame():
        return rospy.wait_for_message("/camera/depth/points", PointCloud2)

    @staticmethod
    def getColorFrame():
        return rospy.wait_for_message("/camera/rgb/image_color", Image)

class pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


if __name__ == '__main__':
    rospy.init_node('nav_controller')
    ne = nav_controller()
    try:
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass 
