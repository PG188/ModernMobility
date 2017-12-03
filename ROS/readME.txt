This directory is for files related for ROS applications.


Set up
1) Download SD Card Formatter:https://www.sdcard.org/downloads/formatter_4/
2) Download Win32DiskImager: https://sourceforge.net/projects/win32diskimager/
3) Download UBUNTU image: https://ubuntu-mate.org/download/
4) Run SD Card Formatter, select the drive that you have your SD card connected to (i.e. drive E:) and format the SD Card (it will be wiped) 
5) Extract the ubuntu-mate .img file
6) Run Win32DiskImager, select the ubuntu-mate .img file, and have it write to the drive with your SD card that you just formatted
7) Put SD card into pi, power it on, go through ubuntu-mate initial settings
8) Open terminal in ubuntu-mate and follow the ROS install instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu (use full desktop install)
