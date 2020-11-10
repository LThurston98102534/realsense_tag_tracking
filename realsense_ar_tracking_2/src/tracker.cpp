#include "tracker.h"

// Constructor
Tracker::Tracker(ros::NodeHandle nh)
    : nh_(nh), listener_(new(tf::TransformListener)), broadcaster_(new(tf::TransformBroadcaster)), refresh_rate_(default_refresh_rate)
{
    sub1_ = nh_.subscribe("ar_pose_marker", 1000, &Tracker::trackerCallback, this);


}

// Deconstructor
Tracker::~Tracker()
{

}

// Alvar ROS package Callback function
void Tracker::trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    // Create unique lock for safe access to member variables
    std::unique_lock<std::mutex> marker_pose_lock(marker_pose_mutex_);

    // Update member variable with most recent pose data
    marker_pose_ = *msg;

    marker_pose_lock.unlock();
    
}
    


 
// Main Alvar Tag tracking Function
/*
 * Function obtains the latest data from the Alvar ROS package
 * Then checks which object it can see (Fridge or Sealer Unit) and broadcasts the relative pose from the camera to the object's tag in the TF tree
*/
void Tracker::trackTags()
{
    // Set refresh rate to default value
    ros::Rate rate_limiter(refresh_rate_);
    while (ros::ok()) {
          
          // Create unique lock for safe access to member variables
          std::unique_lock<std::mutex> marker_pose_lock(marker_pose_mutex_);

          // Update member variable with most recent pose data
          marker_pose_for_calc_ = marker_pose_;

          marker_pose_lock.unlock();
          
          // Initialise temporary variables
          geometry_msgs::PoseStamped sensor_pose;
          float timestamp = 0.0;
          tf::Transform transform;
          
	  // Check each marker that the AR Tag Tracking package can see
          for (int i = 0; i < marker_pose_for_calc_.markers.size(); i++) {
            // If the marker ID = 4, the tag tracking software can see the Fridge
            if (marker_pose_for_calc_.markers.at(i).id == 4) {
                // DEBUGGING PRINTING STATEMENT - PRINTS THE OBJECT IT CAN SEE
                ROS_INFO("I see marker: [%d] which is the fridge!", marker_pose_for_calc_.markers.at(i).id);
                
                // Store the pose data from the Alvar message into variables for ease of use
                sensor_pose = marker_pose_for_calc_.markers.at(i).pose;
                sensor_pose.header.frame_id = marker_pose_for_calc_.markers.at(i).header.frame_id;

                // Generate a TF transform using the position and orientation data from the Alvar message
                transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
                transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));

                // Broadcast the transform to the TF tree
                broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), sensor_pose.header.frame_id, "fridge_tag"));
                
                broadcaster_->sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),ros::Time::now(),"fridge_tag", "fridge_link"));
                
                
                timestamp = marker_pose_for_calc_.markers.at(i).header.stamp.toSec();
            } else{
                // If the marker ID = 0, the tag tracking software can see the Sealer Unit
                if (marker_pose_for_calc_.markers.at(i).id == 0) {
                    // DEBUGGING PRINTING STAETMENT - PRINTS THE OBJECT IT CAN SEE
                    ROS_INFO("I see marker: [%d] which is the sealer!", marker_pose_for_calc_.markers.at(i).id);
                    
                    // Store the pose data from the Alvar message into variables for ease of use
                    sensor_pose = marker_pose_for_calc_.markers.at(i).pose;
                    sensor_pose.header.frame_id = marker_pose_for_calc_.markers.at(i).header.frame_id;

                    // Generate a TF transform using the position and orientation data from the Alvar message
                    transform.setOrigin(tf::Vector3(sensor_pose.pose.position.x, sensor_pose.pose.position.y, sensor_pose.pose.position.z));
                    transform.setRotation(tf::Quaternion(sensor_pose.pose.orientation.x, sensor_pose.pose.orientation.y, sensor_pose.pose.orientation.z, sensor_pose.pose.orientation.w));

                    // Broadcast the transform to the TF tree
                    broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), sensor_pose.header.frame_id, "sealer_tag"));
                    
                    broadcaster_->sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),ros::Time::now(),"sealer_tag", "sealer_link"));
                    
                    timestamp = marker_pose_for_calc_.markers.at(i).header.stamp.toSec();
                } else {
                    ROS_INFO("Not sure what I can see!");
                }
            }
          }

       // Delay thread for remainder of time in loop
       rate_limiter.sleep();

    }




}
