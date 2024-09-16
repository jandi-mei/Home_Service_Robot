#!/bin/sh
xterm  -e  "bash -c ' source ../../devel/setup.bash;
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/workspace/Project5/Home_Service_Robot/catkin_ws/src/my_robot/worlds/OfficeBuild.world'" &
echo "Launching Gazebo..."
sleep 10
xterm -hold -e  "bash -c ' source /opt/ros/kinetic/setup.bash;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch initial_pose_x:=-1.372050 initial_pose_y:=-1.629530 initial_pose_a:=1.071914'" & 
echo "Launching AMCL..."
sleep 10
echo "Launching Rviz..."
xterm  -e  "bash -c ' source ../../devel/setup.bash; 
roslaunch turtlebot_rviz_launchers view_navigation.launch'" &
sleep 20
echo "Launching pick_objects node..."
xterm  -e  "bash -c ' source devel/setup.bash; 
rosrun pick_objects pick_objects'"