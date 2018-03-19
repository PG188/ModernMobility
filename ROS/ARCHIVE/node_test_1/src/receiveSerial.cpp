#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"

const char *serialIn;
int mag[4] = {0};
int lslider = 0;
int rslider = 0;

void serial_callback(const std_msgs::String::ConstPtr& receiveSerial) {
	serialIn = (receiveSerial->data).data();
	lslider = (serialIn[1] << 8) | serialIn[0];
	rslider = (serialIn[3] << 8) | serialIn[2];
	mag[0] = (serialIn[5] << 8) | serialIn[4];
	mag[1] = (serialIn[7] << 8) | serialIn[6];
	mag[2] = (serialIn[9] << 8) | serialIn[8];
	mag[3] = (serialIn[11] << 8) | serialIn[10];
	ROS_INFO("lslider = %d, ", lslider);
	ROS_INFO("rslider = %d, ", rslider);
	ROS_INFO("mag0 = %d, ", mag[0]);
	ROS_INFO("mag1 = %d, ", mag[1]);
	ROS_INFO("mag2 = %d, ", mag[2]);
	ROS_INFO("mag3 = %d\n", mag[3]);
}

int main(int argc, char** argv){
	//ROS node initialization
	ros::init(argc, argv, "receiveSerial");
	ros::NodeHandle nh;

	//Adds subscriber that subscribesleft slider value on appropriate topic, reads with callback
	ros::Subscriber lslider_sub = nh.subscribe("SerialIn", 1, serial_callback);
	ros::Rate loop_rate(10);  //Specifices how often the while loop should execute 
	ros::spin();
	return 0;
}
