#include <ros/ros.h>
#include "std_msgs/UInt16.h"

#define manual_free_roll 0
#define manual_assisted 1
#define auto_go_user 2
#define auto_go_park 3
#define auto_stop 4


std_msgs::UInt16 state;

void phone_cmd_callback(const std_msgs::UInt16::ConstPtr& PhoneCommand) {
	if (PhoneCommand->data == 0) {
		state.data = auto_go_user;
	}
	else if (PhoneCommand->data == 1) {
		state.data = auto_go_park;	
	}
	else if (PhoneCommand->data == 2) {
		state.data = auto_stop;	
	}
}

int main(int argc, char** argv){
	//ROS node initialization
	ros::init(argc, argv, "HMINode");
	ros::NodeHandle nh;

	state.data = manual_assisted;	//initial state

	//Suscriber initialization
	//Adds subscriber that subscribes to bluetooth phone commands on the appropriate topic, reads with callback
	ros::Subscriber phoneCommandSub = nh.subscribe("blue_cmd", 1, phone_cmd_callback);
	//ros::Subscriber phonePoseSub = nh.subscribe("blue_pose", 1, phone_cmd_callback);

	//Publisher initialization
	//Adds publisher that publishes walker mode on the appropriate topic, with a buffer size of 1000	
	ros::Publisher walkerModePub = nh.advertise<std_msgs::UInt16>("walker_mode",1000);
	
	ros::Rate loop_rate(10);  //Specifices how often the while loop should execute 

	while(ros::ok()){
		ros::spinOnce(); //Allows the subscriber callbacks to execute if new data available
		switch (state.data) {
			case manual_free_roll:
				walkerModePub.publish(state);
				//ROS_INFO("State = manual_free_roll");
				break;
			case manual_assisted:
				walkerModePub.publish(state);
				//ROS_INFO("State = manual_assisted");
				break;
			case auto_go_user:
				walkerModePub.publish(state);
				//ROS_INFO("State = auto_go_user");
				break;	
			case auto_go_park:
				walkerModePub.publish(state);
				//ROS_INFO("State = auto_go_park");
				break;
			case auto_stop:
				walkerModePub.publish(state);
				//ROS_INFO("State = auto_go_park");
				break;
		}
		loop_rate.sleep();
	}
	ros::spin(); //Should never actually be reached
	return 0;
}
