//g++ -o BTReceive BTReceive.cpp -lbluetooth <--TYPE THIS COMMAND TO COMPILE

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <iostream>
#include <math.h>


float S, E, M, num, x, y, o;

float toFloatNum(char b0, char b1, char b2, char b3){
	
	S = b0 >> 7;	//sign
	E = (((b0 & 0x7f) << 1) | ((b1 & 0x80) >> 7));	//exponent
	M = (((b1 & 0x7f) << 16) | (b2 << 8) | b3);	//mantissa
	
	num = pow(-1, S) * (1 + (M/pow(2,23))) * pow(2, E-127);	//convert to floating point number
	
	return num;
}

int interpretCmd(char *buf){
	
	int cmd = -1;

	char data = *buf;
	
	switch(data){
	
		case (char)0:
			printf("Command Cancelled\n");
			//call function that sends cmd to master controller
			cmd = 0;
			
			return 0;
		break;
		
		case (char)1:
			printf("SmartWalker is heading to docking station\n");
			//call function that sends cmd to master controller
			cmd = 1;

			return 0;
		break;
		
		case (char)2:
			printf("SmartWalker is parking\n");
			//call function that sends cmd to master controller
			cmd = 2;
			
			return 0;
		break;
		
		case (char)3:
			printf("Extra command\n");
			//call function that sends cmd to master controller
			cmd = 3;
			
			return 0;
		break;
		
		case (char)4:
			printf("Extra command 2\n");
			//call function that sends cmd to master controller
			cmd = 4;
			
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
	
	return 0;
}

//REFERENCE: https://people.csail.mit.edu/albert/bluez-intro/x502.html
//REFERENCE: https://raspberrypi.stackexchange.com/questions/55850/rpi-bluetooth-headless-communication-with-android-phone-no-pairing/55884

int main(int argc, char **argv){

	bool connected = true;
	
	//Raspberry Pi's Bluetooth Address
	char address[18] = "B8:27:EB:DE:AB:67";

	struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
	char buf[1024] = { 0 };
	int bs, client, bytes_read;
	socklen_t opt = sizeof(rem_addr);

	// allocate socket
	bs = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// bind socket to port 1 of the first available 
	// local bluetooth adapter
	loc_addr.rc_family = AF_BLUETOOTH;
	str2ba( address, &loc_addr.rc_bdaddr);
	loc_addr.rc_channel = (uint8_t) 1;
	bind(bs, (struct sockaddr *)&loc_addr, sizeof(loc_addr));
	
	// put socket into listening mode
	listen(bs, 1);
	printf("\nlistening...\n");

	// accept one connection
	client = accept(bs, (struct sockaddr *)&rem_addr, &opt);
	printf("\nconnection accepted!\n");

	ba2str( &rem_addr.rc_bdaddr, buf );
	fprintf(stderr, "accepted connection from %s\n", buf);
	memset(buf, 0, sizeof(buf));

	while (connected){
		// read data from the client
		bytes_read = read(client, buf, sizeof(buf));
		if(bytes_read == 1){
			if (*buf != (char)9) {
				int close = interpretCmd(buf);
				if(close == 1){
					connected = false;
					printf("connection terminated\n");
				}
				//printf("received [%s]\n", buf);

			} else {
				break;
			}
		}
		else{
			if(bytes_read == 12){
				interpretPose(buf);
			}
		}
	}

	// close connection
	close(client);
	close(bs);

	return 0;
}
