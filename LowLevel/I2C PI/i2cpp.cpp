/*
Instal i2c tools:
	sudo apt-get install i2c-tools

Instal wiringPi tools:
	sudo apt-get purge wiringpi
	hash -r
	sudo apt-get install git-core
	sudo apt-get update
	sudo apt-get upgrade
	cd
	git clone git://git.drogon.net/wiringPi
	cd ~/wiringPi
	git pull origin
	cd ~/wiringPi
	./build

Add to the bottom of config:
	sudo nano /bootconfig.txt

	dtparam=i2c1=on
	dtparam=i2c_arm=on 

Add to the botom of modules:
	sudo nano /etc/modules
	
	isnd-bcm2835
	ipv6
	i2c-dev

reboot and test i2c line with:
	ls /dev/*i2c*
	i2cdetect -y 1

build with:
	g++ i2cpp.cpp -lwiringPi -o i2cpp
*/
#include <iostream>
#include <wiringPiI2C.h>

using namespace std;

int Ard1 = wiringPiI2CSetup(0x04);

int motorDir = 0;										//1 = left 				10 = right 
int speedRight = 0;										//0 - 9 
int speedLeft = 0;										//0 - 9  
int data = 100000

int main(){
	
	data = data + (motorDir*100) + (speedRight*10) + speedLeft

	int data1 = data  

	if (send_signal = true)
	cout << "Output: "<< data << endl;
	int output = wiringPiI2CWrite(Ard1, data);			//send data

		
	int input = wiringPiI2CRead (Ard1);
	cout << "Intput: "<< input << endl;					//recive data
}











