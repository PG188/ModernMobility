#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "nav_msgs/Odometry.h"


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
	ROS_INFO("Pose linear x:%f y: %f z:%f \n ", (odom->pose).pose.position.x, (odom->pose).pose.position.y, (odom->pose).pose.position.z);
	ROS_INFO("Pose rotation x:%f y: %f z:%f w:%f\n ", (odom->pose).pose.orientation.x, (odom->pose).pose.orientation.y, 
												(odom->pose).pose.orientation.z, (odom->pose).pose.orientation.w);
	ROS_INFO("Twist linear x:%f y: %f z:%f \n ", (odom->twist).twist.linear.x, (odom->twist).twist.linear.y, (odom->twist).twist.linear.z);
	ROS_INFO("Twist angular x:%f y: %f z:%f\n ", (odom->twist).twist.angular.x, (odom->twist).twist.angular.y,
													 (odom->twist).twist.angular.z);
}

// void tf_callback(const std_msgs::Bool::ConstPtr& use_slider_vel) {
// 	ROS_INFO("use_slider_vel arrived: %d \n",use_slider_vel->data);
// }

int main(int argc, char** argv){
	//ROS init
	ros::init(argc, argv, "diff_tf_test");
	ros::NodeHandle diff_tf_test;
	ros::Rate loop_rate(10);

	std_msgs::Int16 left, right;
	left.data = right.data = 0;

	//Publishers
	ros::Publisher left_pub = diff_tf_test.advertise<std_msgs::Int16>("lwheel", 1000);
	ros::Publisher right_pub = diff_tf_test.advertise<std_msgs::Int16>("rwheel", 1000);

	//Subscribers
	ros::Subscriber odom_sub = diff_tf_test.subscribe("odom", 1, odom_callback);
	//ros::Subscriber tf_sub = overlord_test_node.subscribe("tf", 1, tf_callback);

	while (ros::ok()) {
		ros::spinOnce();
		left_pub.publish(left);
		right_pub.publish(right);
		left.data += 1;
		right.data -= 1;
		loop_rate.sleep();
	}
	ros::spin(); //Should never actually be reached
	return 0;
}
