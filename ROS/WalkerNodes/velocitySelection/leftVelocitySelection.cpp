#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"

#define manual 0
#define autonomous 1
#define free_roll 0
#define assisted 1
#define free_roll_flag 999
#define max_slider_vel 1

std::Float32 vel_cmd_in;
std::UInt16 sliderVal_in;
std::Float32 vel_cmd_out;

int manual_mode = assisted; //Change to topic later
int walker_mode = manual; //Change to topic later

void velocity_cmd_callback(const std_msgs::Float32::ConstPtr& VelCommand) {
	vel_cmd_in.data = VelCommand->data;
}

void slider_callback(const std_msgs::UInt16::ConstPtr& sliderVal) {
	sliderVal_in.data = sliderVal->data;
}

int main(int argc, char** argv){
	//ROS node initialization
	ros::init(argc, argv, "leftVelocitySelection");
	ros::NodeHandle nh;

	vel_cmd_in.data = 0;
	sliderVal_in.data = 512; //In middle of slider
	vel_cmd_out.data = 0;

	//Suscriber initialization
	//Adds subscriber that subscribes left motor velocity command on appropriate topic, reads with callback
	ros::Subscriber lmtr_velocity_sub = nh.subscribe("lwheel_vtarget", 1, velocity_cmd_callback);
	//Adds subscriber that subscribesleft slider value on appropriate topic, reads with callback
	ros::Subscriber lslider_sub = nh.subscribe("lslider_pos", 1, slider_callback);

	//Publisher initialization
	//Adds publisher that publishes left motor velocity on appropriate topic, with a buffer size of 1000	
	ros::Publisher lmtr_velocity_pub = nh.advertise<std_msgs::Float32>("lwheel_vel_master",1000)
	
	ros::Rate loop_rate(10);  //Specifices how often the while loop should execute 

	while(ros::ok()){
		ros::spinOnce(); //Allows the subscriber callbacks to execute if new data available
		if (walker_mode == manual) {
			if (manual_mode == free_roll) {
				vel_cmd_out.data = free_roll_flag; //Identifies free roll to motor controller
			} else if (manual_mode == assisted) {
				if (sliderVal_in.data <= 407) {
					vel_cmd_out.data = max_slider_vel*(-1 + sliderVal_in.data/407);
				} else if (sliderVal_in.data >= 408 && sliderVal_in.data <= 615) {
					vel_cmd_out.data = 0; //Identifies free roll to motor controller
				} else {
					vel_cmd_out.data = max_slider_vel*(sliderVal_in.data - 616)/407;
				}
			}
		} else if (walker_mode == autonomous) {
			vel_cmd_out = vel_cmd_in;
		}
		lmtr_velocity_pub.publish(vel_cmd_out)
		loop_rate.sleep();
	}
	ros::spin(); //Should never actually be reached
	return 0;
}
