//g++ -o rfcomm-server rfcomm-server.cpp -lbluetooth <--TYPE THIS COMMAND TO COMPILE

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <iostream>

//ROS stuff
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include <sstream>

ros::Publisher bluetalk_pub;

int interpretCmd(char *buf){
	
//=====ROS================================================
	int cmd;
	std_msgs::Int32 msg;
		


//=====BLUETOOTH STUFF====================================

	char data = *buf;
	ROS_INFO("In, data = %d",data);
	printf("In, data = %d",data);
	switch(data){
	
		case (char)0:
			printf("SmartWalker is navigating towards the user\n");
			//call ROS function
			msg.data = 0;
			bluetalk_pub.publish(msg);
			ROS_INFO("%d",msg.data);
			
			return 0;
		break;
		
		case (char)1:
			printf("SmartWalker is parking\n");
			//call ROS function
			msg.data = 1;
			bluetalk_pub.publish(msg);
			ROS_INFO("%d",msg.data);

			return 0;
		break;
		
		case (char)2:
			printf("SmartWalker has stopped\n");
			//call ROS function
			msg.data = 2;
			bluetalk_pub.publish(msg);
			ROS_INFO("%d",msg.data);
			
			return 0;
		break;
		
		case (char)3:
			printf("Smartwalker is resuming\n");
			//call ROS function
			msg.data = 3;
			bluetalk_pub.publish(msg);
			ROS_INFO("%d",msg.data);
			
			return 0;
		break;
		
		case (char)4:
			printf("Command cancelled\n");
			//call ROS function
			msg.data = 4;
			bluetalk_pub.publish(msg);
			ROS_INFO("%d",msg.data);
			
			return 0;
		break;
		
		case (char)9:
			printf("Disconnecting...");
			return 1;
	}
}

int interpretPose(char *buf){
	int x, y, o;
	
	x = (int)(buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3]);
	y = (int)(buf[4] << 24 | buf[5] << 16 | buf[6] << 8 | buf[7]);
	o = (int)(buf[8] << 24 | buf[9] << 16 | buf[10] << 8 | buf[11]);
	//publishPose(x, y, o);
	printf("x: %d, y: %d, o: %d\n", x, y, o);
}
	
	

//REFERENCE: https://people.csail.mit.edu/albert/bluez-intro/x502.html
//REFERENCE: https://raspberrypi.stackexchange.com/questions/55850/rpi-bluetooth-headless-communication-with-android-phone-no-pairing/55884

int main(int argc, char **argv)
{

//=====ROS================================================

	ros::init(argc, argv, "talker");
	ros::NodeHandle bluetooth;
	bluetalk_pub = bluetooth.advertise<std_msgs::Int32>("bluetalk", 1000);
	

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
			if(close == 1){
				connected = false;
				printf("connection terminated\n");
			}
			//printf("received [%s]\n", buf);
		}
		else{
			if(bytes_read == 12){
				interpretPose(buf);
			}
		}
		loop_rate.sleep();
	}

	// close connection
	close(client);
	close(s);



	return 0;
}
