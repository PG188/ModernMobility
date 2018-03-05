#include <ros/ros.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//System state definitions
enum systemStates {MANUAL_FREEROLL, MANUAL_ASSISTED, AUTO_GETGOAL, AUTO_NAV, AUTO_ATGOAL};

//Phone commands definitions
enum phoneCommands {CANCEL_NAV, START_NAV};

//Screen commands definitions
enum screenCommands {ASSISTED, FREE_ROLL};

//
static struct {
	phoneCommands cmd;
	bool cmd_flag; 
} phone;

static struct {
	screenCommands cmd;
	bool cmd_flag; 
} screen;

static struct {
	bool moved;
	bool moved_changed;
	bool useSliderVel;
} slider;

static struct {
	bool arrived;
	move_base_msgs::MoveBaseGoal goal;
} goal;

void phone_cmd_callback(const std_msgs::String::ConstPtr& phone_cmd_in) {
	if (phone_cmd_in->data == "Cancel nav")
		phone.cmd = CANCEL_NAV;
	else if (phone_cmd_in->data == "Start nav")
		phone.cmd = START_NAV;
	phone.cmd_flag = true;
}

void  screen_cmd_callback(const std_msgs::String::ConstPtr& screen_cmd_in) {
	if (screen_cmd_in->data == "Assist")
		screen.cmd = ASSISTED;
	else if (screen_cmd_in->data == "Free roll")
		screen.cmd = FREE_ROLL;
	screen.cmd_flag = true;
}

void slider_moved_callback(const std_msgs::Bool::ConstPtr& slider_moved_in) {
	slider.moved = slider_moved_in->data;
	slider.moved_changed = slider_moved_in->data;
}

void nav_goal_callback(const geometry_msgs::Pose::ConstPtr& goalPose) {
	goal.goal.target_pose.header.frame_id = "base_link";
	goal.goal.target_pose.header.stamp = ros::Time::now();
	goal.goal.target_pose.pose = *goalPose;
	goal.arrived = true;
}

int main(int argc, char** argv){
	//ROS init
	ros::init(argc, argv, "overlord");
	ros::NodeHandle overlord_node;
	ros::Rate loop_rate(5);
	MoveBaseClient ac("move_base", true);
	while (!ac.waitForServer(ros::Duration(5.0))); //Waits for action lib to come up

	//Subscribers
	ros::Subscriber phone_cmd_sub = overlord_node.subscribe("phone_cmd", 1, phone_cmd_callback);
	ros::Subscriber screen_cmd_sub = overlord_node.subscribe("screen_cmd", 1, screen_cmd_callback);
	ros::Subscriber slider_moved_sub = overlord_node.subscribe("slider_moved", 1, slider_moved_callback);
	ros::Subscriber goal_sub = overlord_node.subscribe("nav_goal", 1, nav_goal_callback);

	//ros::Subscriber init_pos_sub = overlord_node.subscribe("init_pos", 1, init_pos_callback);
	
	//Publishers
	ros::Publisher free_roll_pub = overlord_node.advertise<std_msgs::Bool>("allow_free_roll",1000);
	ros::Publisher useSliderVel_pub = overlord_node.advertise<std_msgs::Bool>("use_slider_vel",1000);

	//Data init 
	bool allowFreeRoll = true;
	systemStates systemState = MANUAL_ASSISTED;
	systemStates nextState = MANUAL_ASSISTED;
	phone.cmd = CANCEL_NAV;
	phone.cmd_flag = false;
	screen.cmd = FREE_ROLL;
	screen.cmd_flag = false;
	slider.moved = false;
	slider.moved_changed = false;
	slider.useSliderVel = true;
	goal.arrived = false;
	std_msgs::Bool allowFreeRoll_out, useSliderVel_out;

	while (ros::ok()) {
		systemState = nextState;
		ros::spinOnce();
		switch (systemState) {
			case MANUAL_FREEROLL:
				allowFreeRoll = true;
				if (screen.cmd_flag) {
					if (screen.cmd == ASSISTED)
						nextState = MANUAL_ASSISTED;
					screen.cmd_flag = false;
				} else if (phone.cmd_flag) {
					if (phone.cmd == START_NAV)
						nextState = AUTO_GETGOAL;
					phone.cmd_flag = false;
				}
				slider.useSliderVel= false;
				allowFreeRoll = true;
				break;

			case MANUAL_ASSISTED:
				ROS_INFO("In manual assisted\n");
				if (screen.cmd_flag) {
					ROS_INFO("In screen cmd flag\n");
					if (screen.cmd == FREE_ROLL)
						nextState = MANUAL_FREEROLL;
					screen.cmd_flag = false;
				} else if (phone.cmd_flag) {
					ROS_INFO("In phone cmd flag\n");
					if (phone.cmd == START_NAV)
						nextState = AUTO_GETGOAL;
					phone.cmd_flag = false;
				}
				slider.useSliderVel= true;
				allowFreeRoll = false;
				break;

			case AUTO_GETGOAL:
				if (slider.moved_changed) {
					nextState = MANUAL_FREEROLL;
					slider.moved_changed = false;
				} else if (screen.cmd_flag) {
					nextState = MANUAL_FREEROLL;
					screen.cmd_flag = false;
				} else if (phone.cmd_flag) {
					if (phone.cmd == CANCEL_NAV) {
						nextState = MANUAL_FREEROLL;
					}
					phone.cmd_flag = false;
				} else {
					if (goal.arrived) {
						ac.sendGoal(goal.goal); //Begin navigation
						nextState = AUTO_NAV;
						goal.arrived = false;
					}
				}
				slider.useSliderVel= false;
				allowFreeRoll = false;
				break;

			case AUTO_NAV:
				if (slider.moved_changed) {
					nextState = MANUAL_FREEROLL;
					ac.cancelGoal(); //Cancels goal
					slider.moved_changed = false;
				} else if (screen.cmd_flag) {
					nextState = MANUAL_FREEROLL;
					ac.cancelGoal(); //Cancels goal
					screen.cmd_flag = false;
				} else if (phone.cmd_flag) {
					if (phone.cmd == CANCEL_NAV) {
						nextState = MANUAL_FREEROLL;
					}
					ac.cancelGoal(); //Cancels goal
					phone.cmd_flag = false;
				} else {
					if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
						ROS_INFO("Navigating to goal\n");
					} else if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
						ROS_INFO("Navigating succeeded\n");
						nextState = AUTO_ATGOAL;
					} else {
						ROS_INFO("Navigation failed; Returning to manual mode\n");
						ac.cancelGoal();
						nextState = MANUAL_FREEROLL;
					}
				}
				slider.useSliderVel= false;
				allowFreeRoll = false;
				break;

			case AUTO_ATGOAL:
				ROS_INFO("Goal acheived. Returning to free roll\n");
				nextState = MANUAL_FREEROLL;
				break;
		}
		ROS_INFO("Overlord state: %d \n", systemState);
		allowFreeRoll_out.data = allowFreeRoll;
		useSliderVel_out.data = slider.useSliderVel;
		free_roll_pub.publish(allowFreeRoll_out);
		useSliderVel_pub.publish(useSliderVel_out);
		loop_rate.sleep();
	}
	ros::spin(); //Should never actually be reached
	return 0;
}
