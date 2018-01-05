#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char** argv){
	int numOfUSSensors = 4;
	sensor_msgs::PointCloud cloud;
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
	ros::init(argc, argv, "US_pub");
	ros::NodeHandle n;
	ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("US_cloud", 50);
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
	
	while(n.ok()){
		//If four magnitude values available to be read, initiate the read, and store
		//the values in the magnitude array. Magnitude should be in METERS
		if (true) { //If at least four values available. true is temporary
			
			//Read magnitude values in
			
			/*Transforms the magnitude values from each 
			 *sensor into a point (x,y,z) in the point cloud
			 *using the coordinate frame of each sensor. */
			for(unsigned int i = 0; i < numOfUSSensors; ++i) {
				cloud.points[i].x = US_frames[i][0] + magnitude[i]*cos(US_frames[i][3]);
				cloud.points[i].y = US_frames[i][1] + magnitude[i]*sin(US_frames[i][3]);
				cloud.points[i].z = US_frames[i][2];
			}
			cloud.header.stamp = ros::Time::now();
			cloud_pub.publish(cloud);
			r.sleep();
		}
	}
}