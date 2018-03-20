#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"

#define manual_free_roll 0
#define manual_assisted 1
#define auto_go_user 2
#define auto_go_park 3

#define free_roll_flag 999.0
#define max_slider_vel 1

std_msgs::Float32 vel_cmd_in;
std_msgs::UInt16 sliderVal_in;
std_msgs::Float32 vel_cmd_out;
std_msgs::UInt16 walker_mode;

void velocity_cmd_callback(const std_msgs::Float32::ConstPtr& VelCommand) {
	vel_cmd_in.data = VelCommand->data;
}

void slider_callback(const std_msgs::UInt16::ConstPtr& sliderVal) {
	sliderVal_in.data = sliderVal->data;
}

void mode_callback(const std_msgs::UInt16::ConstPtr& modeVal) {
	walker_mode.data = modeVal->data;
}

int main(int argc, char** argv){
	//ROS node initialization
	ros::init(argc, argv, "rightVelocitySelection");
	ros::NodeHandle nh;

	vel_cmd_in.data = 0;
	sliderVal_in.data = 512; //In middle of slider
	vel_cmd_out.data = 0;

	//Suscriber initialization
	//Adds subscriber that subscribes left motor velocity command on appropriate topic, reads with callback
	ros::Subscriber rmtr_velocity_sub = nh.subscribe("rwheel_vtarget", 1, velocity_cmd_callback);
	//Adds subscriber that subscribesleft slider value on appropriate topic, reads with callback
	ros::Subscriber rslider_sub = nh.subscribe("rslider_pos", 1, slider_callback);
	//Adds subscriber that subscribes to walker mode on appropriate topic, reads with callback
	ros::Subscriber walker_mode_sub = nh.subscribe("walker_mode", 1, mode_callback);

	//Publisher initialization
	//Adds publisher that publishes left motor velocity on appropriate topic, with a buffer size of 1000	
	ros::Publisher rmtr_velocity_pub = nh.advertise<std_msgs::Float32>("rwheel_vel_master",1000);
	
	ros::Rate loop_rate(10);  //Specifices how often the while loop should execute 

	while(ros::ok()){
		ros::spinOnce(); //Allows the subscriber callbacks to execute if new data available
		/*if (walker_mode == manual) {
			if (manual_mode == free_roll) {
				vel_cmd_out.data = free_roll_flag; //Identifies free roll to motor controller
			} else if (manual_mode == assisted) {
				if (sliderVal_in.data <= 407) {
					vel_cmd_out.data = max_slider_vel*(-1 + float(sliderVal_in.data)/407);
				} else if (sliderVal_in.data >= 408 && sliderVal_in.data <= 615) {
					vel_cmd_out.data = 0; //Identifies free roll to motor controller
				} else {
					vel_cmd_out.data = max_slider_vel*(float(sliderVal_in.data) - 616)/407;
				}
			}
		} else if (walker_mode == autonomous) {
			vel_cmd_out = vel_cmd_in;
		}*/
		if (walker_mode.data == manual_free_roll) {
			vel_cmd_out.data = free_roll_flag; //Identifies free roll to motor controller
		} else if (walker_mode.data == manual_assisted) {
			if (sliderVal_in.data <= 407) {
				vel_cmd_out.data = max_slider_vel*(-1 + float(sliderVal_in.data)/407);
			} else if (sliderVal_in.data >= 408 && sliderVal_in.data <= 615) {
				vel_cmd_out.data = 0; //Identifies free roll to motor controller
			} else {
				vel_cmd_out.data = max_slider_vel*(float(sliderVal_in.data) - 616)/407;
			}
		} else if (walker_mode.data == auto_go_user || walker_mode.data == auto_go_park) {
			vel_cmd_out = vel_cmd_in;
		}
		rmtr_velocity_pub.publish(vel_cmd_out);
		loop_rate.sleep();
	}
	ros::spin(); //Should never actually be reached
	return 0;
}
