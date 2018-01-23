//g++ -o rfcomm-server rfcomm-server.cpp -lbluetooth <--TYPE THIS COMMAND TO COMPILE

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <iostream>
   		
int interpretCmd(char *buf){

	char data = *buf;
	switch(data){
	
		case (char)0:
			printf("SmartWalker is navigating towards the user\n");
			//call ROS function
			//publishCmd(0);
			return 0;
		break;
		
		case (char)1:
			printf("SmartWalker is parking\n");
			//call ROS function
			//publishCmd(1);
			return 0;
		break;
		
		case (char)2:
			printf("SmartWalker has stopped\n");
			//call ROS function
			//publishCmd(2);
			return 0;
		break;
		
		case (char)3:
			printf("Smartwalker is resuming\n");
			//call ROS function
			//publishCmd(3);
			return 0;
		break;
		
		case (char)4:
			printf("Command cancelled\n");
			//call ROS function
			//publishCmd(4);
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
	bool connected = true;
	
	//Raspberry Pi's Bluetooth Address
	char address[18] = "B8:27:EB:F0:3C:27";

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
	printf("\nconnectiom accepted!\n");

	ba2str( &rem_addr.rc_bdaddr, buf );
	fprintf(stderr, "accepted connection from %s\n", buf);
	memset(buf, 0, sizeof(buf));

	while (connected){
	
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
	}

	// close connection
	close(client);
	close(s);

	return 0;
}
