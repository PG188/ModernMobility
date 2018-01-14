#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16.h"
#include <wiringSerial.h>

/* Description: ROS node for the arduinos that are working with the motors
 * - Publishes information about the encoders for usage by the differential
 * drive package received over I2C from the arduino. 
 * - Subscribes to the motor control commands (in effort) from the differential
 * drive library and sends it to the arduino for actuation
 */ 

int arduinoSerialPort = serialOpen("/dev/ttyACM0", 19200); //Enables serial comms

void velocity_cmd_callback(const std_msgs::Float32::ConstPtr& VelCommand) {
	char * VelCommand_float = (char *) &(VelCommand->data);
	serialPutchar(arduinoSerialPort, VelCommand_float[0]); //Sends LSB of float
	serialPutchar(arduinoSerialPort, VelCommand_float[1]); //Sends second byte of float..
	serialPutchar(arduinoSerialPort, VelCommand_float[2]);
	serialPutchar(arduinoSerialPort, VelCommand_float[3]);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "l_wheelController");   //Starts ROS for the node wheelController
	ros::NodeHandle n;   //Creates a node handle and actually initializes the node
	ros::Subscriber velCommandSub = n.subscribe("lwheel_vel_master", 1000, velocity_cmd_callback);
	//Adds a publisher that publishes a String message on the different_chatter topic, with a buffer size of 1000
	ros::Publisher encoderPub = n.advertise<std_msgs::Int16>("leftEncoder", 1000);
	ros::Rate loop_rate(10);  //Specifices how often the while loop should execute 
	
	std_msgs::Int16 encoderCount;
	encoderCount.data = 0;
	char readTemp = 0;

	//Runs until the node is shutdown or a control-c is detected
	while (ros::ok()) {
		if (serialDataAvail(arduinoSerialPort) >= 2) {
			readTemp  = serialGetchar(arduinoSerialPort);
			encoderCount.data = readTemp | (serialGetchar(arduinoSerialPort) << 8);
			encoderPub.publish(encoderCount);
		}
		ros::spinOnce(); //Allows the subscriber callbacks to execute if new data available
		loop_rate.sleep();
	}
	ros::spin(); //Should never actually be reached
	return 0;
}
