#include <ros/ros.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"

//State definitions
#define MANUAL_FREEROLL 0
#define MANUAL_ASSISTED 1
#define AUTO_GETGOAL 2
#define AUTO_NAV 3
#define AUTO_ATGOAL 4

//PHONE COMMANDS
#define CANCEL_NAV 0
#define START_NAV 1

unsigned int systemState, nextState, phone_cmd, screen_cmd;
bool slider_moved, slider_moved_changed, phone_cmd_changed, screen_cmd_changed; 
std_msgs::Bool allowFreeRoll;
std_msgs::Bool useSliderVel;

void phone_cmd_callback(const std_msgs::UInt16::ConstPtr& phone_cmd_in) {
	phone_cmd = phone_cmd_in->data;
	phone_cmd_changed = true;
}

void  screen_cmd_callback(const std_msgs::UInt16::ConstPtr& screen_cmd_in) {
	screen_cmd = screen_cmd_in->data;
	screen_cmd_changed = true;
}

void slider_moved_callback(const std_msgs::UInt16::ConstPtr& slider_moved_in) {
	slider_moved = slider_moved_in->data;
	slider_moved_changed = true;
}

int main(int argc, char** argv){
	//ROS init
	ros::init(argc, argv, "overlord");
	ros::NodeHandle overlord_node;
	ros::Rate loop_rate(20);

	//Subscribers
	ros::Subscriber phone_cmd_sub = nh.subscribe("rwheel_vtarget", 1, phone_cmd_callback);
	ros::Subscriber screen_cmd_sub = nh.subscribe("rwheel_vtarget", 1, screen_cmd_callback);
	ros::Subscriber slider_moved_sub = nh.subscribe("rwheel_vtarget", 1, slider_moved_callback);
	
	//Publishers
	ros::Publisher free_roll_pub = nh.advertise<std_msgs::Bool>("allow_free_roll",1000);
	rosros::Publisher useSliderVel_pub = nh.advertise<std_msgs::Bool>("use_slider_vel",1000);

	//Initialize data 
	systemState = 0;
	nextState = 0;
	phone_cmd = 0; 
	screen_cmd = 0;
	allowFreeRoll.data = true;
	useSliderVel.data = true;
	slider_moved_changed = false;
	phone_cmd_changed = false; 
	screen_cmd_changed = false;

	while (ros::ok()) {
		systemState = nextState;
		ros::spinOnce();
		switch (systemState) {
			case MANUAL_FREEROLL:
				allowFreeRoll.data = true;
				if (screen_cmd.data == ASSISTED) {
					allowFreeRoll.data = false;
					nextState = MANUAL_ASSISTED;
				}
				break;
			case MANUAL_ASSISTED:
				if (screen_cmd.data == FREE_ROLL) {
					useSliderVel.data = false;
					nextState = MANUAL_FREEROLL;
				} else if (phone_cmd == START_NAV) {
					useSliderVel.data = false;
					nextState = AUTO_GETGOAL;
				} else {
					useSliderVel.data = true;
				}
				break;
			case AUTO_GETGOAL:
				if (slider_moved_changed || screen_cmd_changed || phone_cmd == CANCEL_NAV) {
					nextState = MANUAL_FREEROLL;
				} else {
					getGoal();
					//Begin navigation
					nextState = AUTO_NAV;
				}
				break;
			case AUTO_NAV:
				if (slider_moved_changed || screen_cmd_changed || phone_cmd == CANCEL_NAV) {
					nextState = MANUAL_FREEROLL;
					//Cancel move_base goal
				} else if (reachedGoal) {
					nextState = AUTO_ATGOAL;
				}
				break;
			case AUTO_ATGOAL:
				nextState = MANUAL_FREEROLL;
				break;
		}
		free_roll_pub.publish(allowFreeRoll);
		useSliderVel_pub.publish(useSliderVel);
		loop_rate.sleep();
	}
	ros::spin(); //Should never actually be reached
	return 0;
}
