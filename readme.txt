1. Software Environment
Ubuntu 16.04
ROS Kinetic

2. Data Prerequisite
Please download bito_tag_online_test.bag to make demo recur.

3. Program Startup
Terminal 1£º
roscore

Terminal 2£º 
catkin_make_isolated --install --use-ninja --pkg rviz_textured_quads
source devel_isolated/rviz_textured_quads/setup.bash
rosrun rviz_textured_quads tag_display.py

Terminal 3£º 
rosbag play bito_tag_online_test.bag --clock --pause 

Terminal 4£º 
rosrun rviz rviz

4. License Declare
This project is only for educational purpose, not for commercial.
Maintainer: Luyao Li, 286299392@qq.com, luyao6611661@gmail.com
