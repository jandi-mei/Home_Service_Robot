#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

class AddMarkers {
public:
    bool pickedUp;
    bool droppedOff;
    AddMarkers() : pickedUp(false), droppedOff(false){
        // Initialize the ROS node
        ros::NodeHandle nh;

        // Create a subscriber to the /update_marker topic
        marker_sub = nh.subscribe("/update_marker", 10, &AddMarkers::updateMarker, this);
        pose_sub = nh.subscribe("/amcl_pose", 10, &AddMarkers::poseCallback, this);

        // Create a publisher for the marker
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        // Initialize the marker

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map";
        // marker.header.stamp = ros::Time::now();
        
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "add_markers";
        marker.id = 0;

        // Set the marker type to ARROW
        marker.type = visualization_msgs::Marker::ARROW;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.5;
        marker.scale.y = 0.3;
        marker.scale.z = 0.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.a = 1.0; // Alpha
        marker.color.r = 1.0; // Red
        marker.color.g = 0.0; // Green
        marker.color.b = 0.0; // Blue
    }

    void updateMarker(const std_msgs::Bool::ConstPtr& msg) {
        ROS_INFO("Received: %d", msg->data);

        // Update the marker's position based on the robot's pose
        marker.header.stamp = ros::Time::now();

        if(!pickedUp)
        {
          removeMarker();
          pickedUp = true;
          printf("Arrived at the pick up location");
        }
        else
        {
          addDropOffMarker();
          droppedOff = true;
          printf("Arrived at the drop off location");
        }
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        // Extract the pose from the message
        const auto& pose = msg->pose.pose;

        // Update the marker's position based on the robot's pose
        marker.header.stamp = ros::Time::now();
        marker.pose.position.x = pose.position.x;
        marker.pose.position.y = pose.position.y;
        marker.pose.orientation.z = pose.orientation.z;
        marker.pose.orientation.w = pose.orientation.w;
    }

    void addInitialMarker()
    {
        // Update the marker's position based on the robot's pose
        marker.header.stamp = ros::Time::now();

        while (marker_pub.getNumSubscribers() < 1)
        {
          ROS_WARN_ONCE("Please create a subscriber to the marker");
          sleep(1);
        }
        
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL
        marker.action = visualization_msgs::Marker::ADD;
        
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = 1.82673482585;
        marker.pose.position.y = -1.95311387757;
        marker.pose.orientation.z = -0.664933158947;
        marker.pose.orientation.w = 0.746902867937;

        marker.lifetime = ros::Duration();

        // Publish the marker
        marker_pub.publish(marker);
    }

    void removeMarker()
    {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
    }

    void addDropOffMarker()
    {
      //Add the marker to drop off goal
      marker.action = visualization_msgs::Marker::ADD;     
      marker_pub.publish(marker);
    
    }

    void run() {
        // Keep the node running
        ros::spin();
    }

private:
    ros::Subscriber marker_sub;
    ros::Subscriber pose_sub;
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_final;
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  AddMarkers add_marker;
  add_marker.addInitialMarker();
  add_marker.run();
}