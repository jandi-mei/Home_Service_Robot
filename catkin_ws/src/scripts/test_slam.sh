#!/bin/sh
xterm  -e  "bash -c ' source ../../devel/setup.bash;
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/workspace/Project5/Home_Service_Robot/catkin_ws/src/my_robot/worlds/OfficeBuild.world'" &
sleep 5
xterm  -e  "bash -c ' source /opt/ros/kinetic/setup.bash;
source ../../devel/setup.bash;
rosrun gmapping slam_gmapping _base_frame:=robot_footprint '" & 
sleep 5
xterm  -e  "bash -c ' source ../../devel/setup.bash; 
roslaunch turtlebot_rviz_launchers view_navigation.launch'" &
sleep 5
xterm  -e  "bash -c ' source ../../devel/setup.bash; 
roslaunch my_robot teleop.launch '"
