Current files:
- Differential_drive is the prebuilt package that converts the velocity command from the navigation stack into individual motor torque commands to be executed by the motor controller
- node_test_1 is a package that contains the incomplete implementations for the nodes that will perform motor control and communication with the arduino connected to the ultrasound sensors. It also launches the differential_drive package, which interacts with the aforementioned nodes to create a complete control loop. 

Trying node_test_1: To try these, you must copy both the differential_drive and node_test_1 folders in your catkin workspace. You must then build them by running catkin_make in your catkin workspace directory. You must then run the launch file in the node_test_1 package using roslaunch. 

You may receive an error about 'Pyside'. To remedy this, execute the following commands which will install the correct files you need:
sudo add-apt-repository ppa:pyside
sudo apt-get update
sudo apt-get install python-pyside

This directory is for files related for ROS applications.


ROS PI SETUP:
Set up
1) Download SD Card Formatter:https://www.sdcard.org/downloads/formatter_4/
2) Download Win32DiskImager: https://sourceforge.net/projects/win32diskimager/
3) Download UBUNTU image: https://ubuntu-mate.org/download/
4) Run SD Card Formatter, select the drive that you have your SD card connected to (i.e. drive E:) and format the SD Card (it will be wiped) 
5) Extract the ubuntu-mate .img file
6) Run Win32DiskImager, select the ubuntu-mate .img file, and have it write to the drive with your SD card that you just formatted
7) Put SD card into pi, power it on, go through ubuntu-mate initial settings
8) Open terminal in ubuntu-mate and follow the ROS install instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu (use full desktop install)
