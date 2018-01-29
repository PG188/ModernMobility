//g++ -o rfcomm-server rfcomm-server.cpp -lbluetooth <--TYPE THIS COMMAND TO COMPILE

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <iostream>
#include <math.h>

//ROS stuff
#include "ros/ros.h"
#include <geometry_msgs/Point32.h>
<<<<<<< HEAD
#include <std_msgs/Float32.h>
=======
#include <std_msgs/Int32.h>
>>>>>>> 58a2ff836d0703f2ff1de43f0af338f3defa90d9
#include <sstream>

float S, E, M, num, x, y, o;

<<<<<<< HEAD
ros::Publisher bluetalk_pub;
geometry_msgs::Point32 msg;
=======
ros::Publisher blue_pose_pub;
ros::Publisher blue_cmd_pub;
geometry_msgs::Point32 pose;
std_msgs::Int32 cmd;
>>>>>>> 58a2ff836d0703f2ff1de43f0af338f3defa90d9

float toFloatNum(char b0, char b1, char b2, char b3){
	
	S = b0 >> 7;	//sign
	E = (((b0 & 0x7f) << 1) | ((b1 & 0x80) >> 7));	//exponent
	M = (((b1 & 0x7f) << 16) | (b2 << 8) | b3);	//mantissa
	
	num = pow(-1, S) * (1 + (M/pow(2,23))) * pow(2, E-127);	//convert to floating point number
	
	return num;
}

int interpretCmd(char *buf){
	
//=====ROS================================================
<<<<<<< HEAD
	
=======
	//geometry_msgs::Point32 pose;
	//std_msgs::Int32 cmd;
>>>>>>> 58a2ff836d0703f2ff1de43f0af338f3defa90d9
		
//=====BLUETOOTH STUFF====================================

	char data = *buf;
	ROS_INFO("In, data = %d",data);
	switch(data){
	
		case (char)0:
			printf("SmartWalker is navigating towards the user\n");
<<<<<<< HEAD
			//SET MSG VALUES
			msg.x = 1.0;
			msg.y = 8.8;
			msg.z = 9.9;

=======
			//call ROS function
			cmd.data = 0;
			
>>>>>>> 58a2ff836d0703f2ff1de43f0af338f3defa90d9
			return 0;
		break;
		
		case (char)1:
			printf("SmartWalker is parking\n");
<<<<<<< HEAD
			//SET MSG VALUES
			msg.x = 2.0;
			msg.y = 8.8;
			msg.z = 9.9;
=======
			//call ROS function
			cmd.data = 1;
>>>>>>> 58a2ff836d0703f2ff1de43f0af338f3defa90d9

			return 0;
		break;
		
		case (char)2:
			printf("SmartWalker has stopped\n");
<<<<<<< HEAD
			//SET MSG VALUES
			msg.x = 3.0;
			msg.y = 8.8;
			msg.z = 9.9;
=======
			//call ROS function
			cmd.data = 2;
>>>>>>> 58a2ff836d0703f2ff1de43f0af338f3defa90d9
			
			return 0;
		break;
		
		case (char)3:
			printf("Smartwalker is resuming\n");
<<<<<<< HEAD
			//SET MSG VALUES
			msg.x = 4.0;
			msg.y = 8.8;
			msg.z = 9.9;
=======
			//call ROS function
			cmd.data = 3;
>>>>>>> 58a2ff836d0703f2ff1de43f0af338f3defa90d9
			
			return 0;
		break;
		
		case (char)4:
			printf("Command cancelled\n");
<<<<<<< HEAD
			//SET MSG VALUES
			msg.x = 5.0;
			msg.y = 8.8;
			msg.z = 9.9;
=======
			//call ROS function
			cmd.data = 4;
>>>>>>> 58a2ff836d0703f2ff1de43f0af338f3defa90d9
			
			return 0;
		break;
		
		case (char)9:
			printf("Disconnecting...");
			return 1;
	}
}

int interpretPose(char *buf){
	
	x = toFloatNum(buf[0], buf[1], buf[2], buf[3]);
	y = toFloatNum(buf[4], buf[5], buf[6], buf[7]);
	o = toFloatNum(buf[8], buf[9], buf[10], buf[11]);	
	
	printf("float:\tx: %f, y: %f, o: %f\n\n", x, y, o);
	
	pose.x = x;
	pose.y = y;
	pose.z = o;
	return 0;
}

//REFERENCE: https://people.csail.mit.edu/albert/bluez-intro/x502.html
//REFERENCE: https://raspberrypi.stackexchange.com/questions/55850/rpi-bluetooth-headless-communication-with-android-phone-no-pairing/55884

int main(int argc, char **argv){

//=====ROS================================================

	ros::init(argc, argv, "talker");
	ros::NodeHandle bluetooth;
<<<<<<< HEAD
	bluetalk_pub = bluetooth.advertise<geometry_msgs::Point32>("bluetalk", 1000);
=======
	
	blue_cmd_pub = bluetooth.advertise<std_msgs::Int32>("blue_cmd", 1000);
	blue_pose_pub = bluetooth.advertise<geometry_msgs::Point32>("blue_pose", 1000);
>>>>>>> 58a2ff836d0703f2ff1de43f0af338f3defa90d9
	

//=====BLUETOOTH STUFF====================================


	bool connected = true;
	
	//Raspberry Pi's Bluetooth Address
	char address[18] = "B8:27:EB:DE:AB:67";

	struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
	char buf[1024] = { 0 };
	int s, client, bytes_read;
	socklen_t opt = sizeof(rem_addr);

	// allocate socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// bind socket to port 1 of the first available 
	// local bluetooth adapter
	loc_addr.rc_family = AF_BLUETOOTH;
	str2ba( address, &loc_addr.rc_bdaddr);
	loc_addr.rc_channel = (uint8_t) 1;
	bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));
	
	// put socket into listening mode
	listen(s, 1);
	printf("\nlistening...\n");

	// accept one connection
	client = accept(s, (struct sockaddr *)&rem_addr, &opt);
	printf("\nconnection accepted!\n");

	ba2str( &rem_addr.rc_bdaddr, buf );
	fprintf(stderr, "accepted connection from %s\n", buf);
	memset(buf, 0, sizeof(buf));

	//ros rate
	ros::Rate loop_rate(10);

	while (connected && ros::ok){
		// read data from the client
		bytes_read = read(client, buf, sizeof(buf));
		if(bytes_read == 1){
			int close = interpretCmd(buf);

			//ROS PRINT AND PUBLISH MSG================================
			ROS_INFO("msg: (%3.2f, %3.2f. %3.2f)", msg.x, msg.y, msg.z);
			bluetalk_pub.publish(msg);
			
			if(close == 1){
				connected = false;
				printf("connection terminated\n");
			}
			//printf("received [%s]\n", buf);
			blue_cmd_pub.publish(cmd);
			ROS_INFO("%d",cmd.data);
		}
		else{
			if(bytes_read == 12){
				interpretPose(buf);
				blue_pose_pub.publish(pose);
				ROS_INFO("%3.2f, %3.2f, %3.2f", pose.x, pose.y, pose.z);
			}
		}
		loop_rate.sleep();
	}

	// close connection
	close(client);
	close(s);

	return 0;
}
