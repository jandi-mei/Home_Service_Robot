#include <ros/ros.h>
#include "std_msgs/Bool.h" 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher marker_pub;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle nh;
  
  std_msgs::Bool msg;
  marker_pub = nh.advertise<std_msgs::Bool>("update_marker", 10);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position for the robot to reach
  goal.target_pose.pose.position.x = 1.82673482585;
  goal.target_pose.pose.position.y = -1.95311387757;
  // Define a orientation for the robot to reach
  goal.target_pose.pose.orientation.z = -0.664933158947;
  goal.target_pose.pose.orientation.w = 0.746902867937;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the base has reached the first goal");
    msg.data = true; // Set to true or false based on your logic
    marker_pub.publish(msg);
  }
  else
  {
    ROS_INFO("The base failed to move to the first goal for some reason");
  }


  // Wait 5 sec to pickup object before moving to the drop off location
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting 5 sec to pickup the item");
  }

  // Define a position for the robot to reach
  goal.target_pose.pose.position.x = 1.85670781147;
  goal.target_pose.pose.position.y = -2.99661060468;
  // Define a orientation for the robot to reach
  goal.target_pose.pose.orientation.z = -0.709040879889;
  goal.target_pose.pose.orientation.w = 0.705167377752;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending second goal");
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the base has reached the second goal");
    msg.data = true; // Set to true or false based on your logic
    marker_pub.publish(msg);
  }
  else
  {
    ROS_INFO("The base failed to move to the second goal for some reason");
  }
  return 0;
}