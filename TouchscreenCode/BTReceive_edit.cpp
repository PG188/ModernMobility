//g++ -o BTReceive BTReceive.cpp -lbluetooth <--TYPE THIS COMMAND TO COMPILE

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <iostream>
#include <math.h>

//=====================================
/**
    C++ client example using sockets
*/
#include<iostream>    //cout
//#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
//#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent

using namespace std;
 
/**
    TCP Client class
*/
class tcp_client
{
private:
    int sock;
    std::string address;
    int port;
    struct sockaddr_in server;
     
public:
    tcp_client();
    bool conn(string, int);
    bool send_data(string data);
    string receive(int);
};
 
tcp_client::tcp_client()
{
    sock = -1;
    port = 8080;
    address = "172.17.50.37";
}
 
/**
    Connect to a host on a certain port number
*/
bool tcp_client::conn(string address , int port
)
{
    //create socket if it is not already created
    if(sock == -1)
    {
        //Create socket
        sock = socket(AF_INET , SOCK_STREAM , 0);
        if (sock == -1)
        {
            perror("Could not create socket");
        }
         
        cout<<"Socket created\n";
    }
    else    {   /* OK , nothing */  }
     
    //setup address structure
    if(inet_addr(address.c_str()) == -1)
    {
        struct hostent *he;
        struct in_addr **addr_list;
         
        //resolve the hostname, its not an ip address
        if ( (he = gethostbyname( address.c_str() ) ) == NULL)
        {
            //gethostbyname failed
            herror("gethostbyname");
            cout<<"Failed to resolve hostname\n";
             
            return false;
        }
         
        //Cast the h_addr_list to in_addr , since h_addr_list also has the ip address in long format only
        addr_list = (struct in_addr **) he->h_addr_list;
 
        for(int i = 0; addr_list[i] != NULL; i++)
        {
            //strcpy(ip , inet_ntoa(*addr_list[i]) );
            server.sin_addr = *addr_list[i];
             
            cout<<address<<" resolved to "<<inet_ntoa(*addr_list[i])<<endl;
             
            break;
        }
    }
     
    //plain ip address
    else
    {
        server.sin_addr.s_addr = inet_addr( address.c_str() );
    }
     
    server.sin_family = AF_INET;
    server.sin_port = htons( port );
     
    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("connect failed. Error");
        return 1;
    }
     
    cout<<"Connected\n";
    return true;
}
 
/**
    Send data to the connected host
*/
bool tcp_client::send_data(string data)
{
    //Send some data
    if( send(sock , data.c_str() , strlen( data.c_str() ) , 0) < 0)
    {
        perror("Send failed : ");
        return false;
    }
    cout<<"Data send\n";
     
    return true;
}
 
/**
    Receive data from the connected host
*/
string tcp_client::receive(int size=512)
{
    char buffer[size];
    string reply;
     
    //Receive a reply from the server
    if( recv(sock , buffer , sizeof(buffer) , 0) < 0)
    {
        puts("recv failed");
    }
     
    reply = buffer;
    return reply;
}

//=====================================

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

	//===TCP SOCKET STUFF==========
	
	tcp_client c;
    string host;
     
    //cout<<"Enter hostname : ";
    //cin>>host;
     
    //connect to host
    c.conn("172.17.50.37" , 8080);
     
    //send some data
    c.send_data("C++ client for phone data connected!");
     
    //receive and echo reply
    cout<<"----------------------------\n\n";
    cout<<c.receive(1024);
    cout<<"\n\n----------------------------\n\n";
     
    //done
    //return 0;

	//==============================================


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
