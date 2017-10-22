1. Software Environment
Ubuntu 16.04
ROS Kinetic

2. Data Prerequisite
Please download bito_tag_online_test.bag to make demo recur.

3. Demo
Demo video is too big to upload to Github. Please download the video according to the instruction in txt.

4. Program Startup
Terminal 1:
roscore

Terminal 2:
catkin_make_isolated --install --use-ninja --pkg rviz_textured_quads
source devel_isolated/rviz_textured_quads/setup.bash
rosrun rviz_textured_quads tag_display.py

Terminal 3:
rosbag play bito_tag_online_test.bag --clock --pause 

Terminal 4:
rosrun rviz rviz

5. License Declare
This project is only for educational purpose, not for commercial.
Maintainer: Luyao Li, 286299392@qq.com, luyao6611661@gmail.com
