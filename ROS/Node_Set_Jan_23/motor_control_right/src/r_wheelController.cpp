#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "stdint.h"
#include <string>

/* Description: ROS node for the arduinos that are working with the motors
 * - Publishes information about the encoders for usage by the differential
 * drive package received over I2C from the arduino. 
 * - Subscribes to the motor control commands (in effort) from the differential
 * drive library and sends it to the arduino for actuation
 */ 

using namespace std;

std_msgs::Int16 encoderCount;
std_msgs::String velCmd_String;
ros::Subscriber velCommandSub;
ros::Subscriber encoderSub;
ros::Publisher encoderPub;
ros::Publisher velCommandPub;
int8_t velCommand;
char serialOutString[2];

void velocity_cmd_callback(const std_msgs::Float32::ConstPtr& VelCommand) {
	if (VelCommand->data >= 2.5) {
		velCommand = 125;
	} else if (VelCommand->data <= -2.5){
		velCommand = -125;
	} else {
		velCommand = static_cast<char>(static_cast<int>(100*VelCommand->data)/2);
	}
	velCmd_String.data = string(1,velCommand); //String representation of the velocity command
	velCommandPub.publish(velCmd_String);
}

void encoder_callback(const std_msgs::String::ConstPtr& encoderCount_str) {
	encoderCount.data = (short)encoderCount_str->data[0] | ((short)encoderCount_str->data[1] << 8);
	encoderPub.publish(encoderCount);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "r_wheelController");   //Starts ROS for the node wheelController
	ros::NodeHandle n;   //Creates a node handle and actually initializes the node
	encoderSub = n.subscribe("rightEncoder_SerialIn", 1000, encoder_callback);
	velCommandSub = n.subscribe("rwheel_vel_master", 1000, velocity_cmd_callback);
	//Adds a publisher that publishes a String message on the different_chatter topic, with a buffer size of 1000
	encoderPub = n.advertise<std_msgs::Int16>("rightEncoder", 1000);
	velCommandPub = n.advertise<std_msgs::String>("rightMotorVel_SerialOut", 1000);
	encoderCount.data = 0;
	ros::spin();
	return 0;
}
