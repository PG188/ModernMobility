#include <ros/ros.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

void freeroll_callback(const std_msgs::Bool::ConstPtr& allow_freeroll) {
	ROS_INFO("allow_freeroll arrived: %d \n", allow_freeroll->data);
}

void use_slider_callback(const std_msgs::Bool::ConstPtr& use_slider_vel) {
	ROS_INFO("use_slider_vel arrived: %d \n",use_slider_vel->data);
}

int main(int argc, char** argv){
	//ROS init
	ros::init(argc, argv, "overlord_test");
	ros::NodeHandle overlord_test_node;
	ros::Rate loop_rate(10);
	
	std_msgs::String phone_cmd, screen_cmd;
	phone_cmd.data = "Cancel nav";
	screen_cmd.data = "Assist";
	std_msgs::Bool slider_moved;
	slider_moved.data = false;
	geometry_msgs::Pose goal;
	goal.position.x = 1;
	goal.position.y = 0;
	goal.position.z = 0;
	goal.orientation.x = 0;
	goal.orientation.y = 0;
	goal.orientation.z = 0;
	goal.orientation.w = 1;
	std_msgs::UInt16 val;
	val.data = 616;

	//Publishers
	ros::Publisher phone_cmd_pub = overlord_test_node.advertise<std_msgs::String>("phone_cmd", 1000);
	ros::Publisher screen_cmd_pub = overlord_test_node.advertise<std_msgs::String>("screen_cmd", 1000);
	ros::Publisher slider_moved_pub = overlord_test_node.advertise<std_msgs::Bool>("slider_moved", 1000);
	ros::Publisher goal_pub = overlord_test_node.advertise<geometry_msgs::Pose>("nav_goal", 1000);
	ros::Publisher slider_pub = overlord_test_node.advertise<std_msgs::UInt16>("lslider_pos", 1000);

	//Subscribers
	ros::Subscriber free_roll_sub = overlord_test_node.subscribe("allow_free_roll", 1, freeroll_callback);
	ros::Subscriber useSliderVel_sub = overlord_test_node.subscribe("use_slider_vel", 1, use_slider_callback);

	while (ros::ok()) {
		ros::spinOnce();
		//phone_cmd_pub.publish(phone_cmd);
		//screen_cmd_pub.publish(screen_cmd);
		//slider_moved_pub.publish(slider_moved);
		//goal_pub.publish(goal);
		slider_pub.publish(val);
		loop_rate.sleep();
	}
	ros::spin(); //Should never actually be reached
	return 0;
}
