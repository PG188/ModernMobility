#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <stdlib.h>

class UltrasonicSlider_Pub {	
	public:
		UltrasonicSlider_Pub() {
			numOfUSSensors = 4;
			cloud_pub = n.advertise<sensor_msgs::PointCloud>("US_cloud", 50);
			lslider_pos = n.advertise<std_msgs::UInt16>("lslider_pos", 50);
			rslider_pos = n.advertise<std_msgs::UInt16>("rslider_pos", 50);
			lslider_sub = n.subscribe("SerialIn", 1, &UltrasonicSlider_Pub::serial_callback, this);
			//Cloud setup
			cloud.header.frame_id = "US_frame";
			cloud.points.resize(numOfUSSensors);
			cloud.channels.resize(1);
			cloud.channels[0].name = "intensities";
			cloud.channels[0].values.resize(numOfUSSensors);
			//Initializes arrays
			for (unsigned int i = 0; i < numOfUSSensors; ++i) {
				cloud.channels[0].values[i] = 100;
			}
			for (unsigned int i = 0; i < numOfUSSensors; ++i) {
				for (unsigned int j = 0; j < numOfUSSensors; ++j)
					US_frames[i][j] = 0;
				mag[i] = 0;
			}
		}
		
		void serial_callback(const std_msgs::String::ConstPtr& receiveSerial) {
			serialIn = (receiveSerial->data).data();
			lslider_pos_value.data = (serialIn[1] << 8) | serialIn[0];
			rslider_pos_value.data = (serialIn[3] << 8) | serialIn[2];
			mag[0] = double((serialIn[5] << 8) | serialIn[4])/100;
			mag[1] = double((serialIn[7] << 8) | serialIn[6])/100;
			mag[2] = double((serialIn[9] << 8) | serialIn[8])/100;
			mag[3] = double((serialIn[11] << 8) | serialIn[10])/100;
			/*Transforms the magnitude values from each 
			 *sensor into a point (x,y,z) in the point cloud
			 *using the coordinate frame of each sensor. */
			for(unsigned int i = 0; i < numOfUSSensors; ++i) {
				cloud.points[i].x = float(US_frames[i][0] + mag[i]*cos(US_frames[i][3]));
				cloud.points[i].y = float(US_frames[i][1] + mag[i]*sin(US_frames[i][3]));
				cloud.points[i].z = float(US_frames[i][2]);
			}
			cloud.header.stamp = ros::Time::now();
			cloud_pub.publish(cloud);
			lslider_pos.publish(lslider_pos_value);
			rslider_pos.publish(rslider_pos_value);
		}
		
	private:
		double US_frames[4][4];
		int numOfUSSensors;
		const char *serialIn;
		double mag[4];
		sensor_msgs::PointCloud cloud;
		std_msgs::UInt16 lslider_pos_value;
		std_msgs::UInt16 rslider_pos_value;
		ros::Subscriber lslider_sub;
		ros::Publisher cloud_pub;
		ros::Publisher lslider_pos;
		ros::Publisher rslider_pos;
		ros::NodeHandle n;
};

int main(int argc, char** argv){
	//Publisher initialization
	ros::init(argc, argv, "Ultrasonic_and_slider_pub");
	UltrasonicSlider_Pub Ultrsound_because_fuck_you;
	ros::spin();
	return 0;
}
