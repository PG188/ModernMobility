//ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32.h>
#include <sstream>



		
int main(int argc, char **argv){

	//create master of type publisher
	ros::Publisher GUI_pub;
	//message of type Int32
	std_msgs::Int32 msg;
	//GUI node called GUI
	ros::NodeHandle GUI;

	//call ros::init() for remapping purposes
	ros::init(argc, argv, "GUINode");
	
	//tell master we're going to publish on topic GUItalk
	GUI_pub = GUI.advertise<std_msgs::Int32>("GUItalk", 1000);
	
	//frequency of messages being sent until told to sleep
	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()){
		//view message 
		ROS_INFO("msg: (%d)", msg);
		//publish message
		GUI_pub.publish(msg);
		
		loop_rate.sleep();
	}
	
	
	return 0;
}
