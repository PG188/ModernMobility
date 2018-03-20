#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_US_broadcaster");
  ros::NodeHandle n;

  ros::Rate r(10);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        //EDIT BASE POINT HERE
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"base_link", "Ultrasonic"));
    r.sleep();
  }
}
