#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
//#include <stdint.h>

/* Description: ROS node for the arduinos that are working with the motors
 * - Publishes information about the encoders for usage by the differential
 * drive package received over I2C from the arduino. 
 * - Subscribes to the motor control commands (in effort) from the differential
 * drive library and sends it to the arduino for actuation
 */ 

void lmotor_cmd_callback(const std_msgs::Float32::ConstPtr& motorCommand) {
	//Need to change parameter to float 32
	/*Value needs to be sent to the arduino, may have to make objects for doing so global so
	callback has access to them
	 */ 
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "r_wheelController");   //Starts ROS for the node wheelController
	ros::NodeHandle n;   //Creates a node handle and actually initializes the node
	/*Starts the subscribtion to the 'lmotor_cmd' topic published by the differential package
	 *The subscriber callback function is lmotor_cmd_callback
	 */
	ros::Subscriber commandSub = n.subscribe("rmotor_cmd", 1000, lmotor_cmd_callback);
	//Adds a publisher that publishes a String message on the different_chatter topic, with a buffer size of 1000
	ros::Publisher encoderPub = n.advertise<std_msgs::Int16>("rightEncoder", 1000);
	ros::Rate loop_rate(10);  //Specifices how often the while loop should execute 
	
	//TEMPORARY
	std_msgs::Int16 encoderCount;
	encoderCount.data = 0;
	bool encoderBuffEmpty = false;
	//TEMPORARY

	//Runs until the node is shutdown or a control-c is detected
	while (ros::ok()) {
		if (!encoderBuffEmpty) {
			encoderPub.publish(encoderCount);
		}
		ros::spinOnce(); //Allows the subscriber callback to execute if new data has been placed on the lmotor_cmd topic
		loop_rate.sleep();
	}
	ros::spin(); //Should never actually be reached
	return 0;
}
