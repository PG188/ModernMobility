#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_sub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/depthimage_to_laserscan/scan", 1, chatterCallback);
  ros::spin();
  return 0;
}