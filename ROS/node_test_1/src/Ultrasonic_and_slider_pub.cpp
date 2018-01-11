#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt16.h>
#include <wiringSerial.h>

int main(int argc, char** argv){
	int readTemp = 0;
	int numOfUSSensors = 4;
	sensor_msgs::PointCloud cloud;
	std_msgs::UInt16 lslider_pos_value;
	std_msgs::UInt16 rslider_pos_value;
	double magnitude[4] = {0}; //Holds values from sensors
	/*Format: {x,y,z,theta}
	 * x,y,z are in meters
	 * Theta is in radians */
	double const US_frames[4][4] = {
		{0,0,0,0}, //Should be left as zero
		{0,0,0,0},
		{0,0,0,0},
		{0,0,0,0}
	};
	
	//Publisher initialization
	ros::init(argc, argv, "Ultrasonic_and_slider_pub");
	ros::NodeHandle n;
	ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("US_cloud", 50);
	ros::Publisher lslider_pos = n.advertise<std_msgs::UInt16>("lslider_pos", 50);
	ros::Publisher rslider_pos = n.advertise<std_msgs::UInt16>("rslider_pos", 50);
	ros::Rate r(1.0);
	
	//Cloud setup
	cloud.header.frame_id = "US_frame";
	cloud.points.resize(numOfUSSensors);
	cloud.channels.resize(1);
	cloud.channels[0].name = "intensities";
	cloud.channels[0].values.resize(numOfUSSensors);
	for (unsigned int i = 0; i < numOfUSSensors; ++i) {
		cloud.channels[0].values[i] = 100;
	}

	//Sets up serial interface
	//First parameter is not correct, must be set to refer to correct arduino
	int arduinoSerialPort = serialOpen("/dev/ttyAMA0", 19200);

	while(n.ok()){
		//If four magnitude values available to be read, initiate the read, and store
		//the values in the magnitude array. Magnitude should be in METERS
		if (serialDataAvail(arduinoSerialPort) >= 12) {
			readTemp  = serialGetchar(arduinoSerialPort);
			lslider_pos_value.data = readTemp | (serialGetchar(arduinoSerialPort) << 8);
			readTemp  = serialGetchar(arduinoSerialPort);
			rslider_pos_value.data = readTemp | (serialGetchar(arduinoSerialPort) << 8);

			//Reads in ultrasonic mangitude values and converts them to meters
			for(unsigned int i = 0; i < numOfUSSensors; ++i) {
				readTemp  = serialGetchar(arduinoSerialPort);
				magnitude[i] = double((readTemp | (serialGetchar(arduinoSerialPort) << 8)))/100;
			}
			
			/*Transforms the magnitude values from each 
			 *sensor into a point (x,y,z) in the point cloud
			 *using the coordinate frame of each sensor. */
			for(unsigned int i = 0; i < numOfUSSensors; ++i) {
				cloud.points[i].x = float(US_frames[i][0] + magnitude[i]*cos(US_frames[i][3]));
				cloud.points[i].y = float(US_frames[i][1] + magnitude[i]*sin(US_frames[i][3]));
				cloud.points[i].z = float(US_frames[i][2]);
			}
			cloud.header.stamp = ros::Time::now();
			cloud_pub.publish(cloud);

			lslider_pos.publish(lslider_pos_value);
			rslider_pos.publish(rslider_pos_value);

			r.sleep();
		}
	}
}