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
	dtparam=i2c+arm=on 

reboot and test i2c line with:
	ls /dev/*i2c*
	i2cdetect -y 1

build with:
	g++ i2cpp.cpp -lwiringPi -o i2cpp
*/
#include <iostream>
#include <wiringPiI2C.h>

using namespace std;


int main(){
	int Ard1 = wiringPiI2CSetup(0x04);
	int data = 255; //values from 0 to 255 (8bit)
	cout << "Data: "<< data << endl;
	int output = wiringPiI2CWrite(Ard1, data);
}
