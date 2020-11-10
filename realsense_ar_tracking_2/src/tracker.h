#ifndef TRACKER_H
#define TRACKER_H

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sstream>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <condition_variable>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

const double default_refresh_rate = 5;

class Tracker{
public:
    Tracker(ros::NodeHandle nh); // Constructor

    ~Tracker(); // Deconstructor


    void trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg); // Alvar ROS package callback function


    void trackTags(); // Main Tag tracking function

    ros::NodeHandle nh_; // ROS Nodehandle

private:
    tf::TransformListener* listener_; // TF listener used to obtain relative transforms between frames in TF tree
    tf::TransformBroadcaster* broadcaster_; // TF broadcaster used to publish relative transforms between frames to the TF tree

    ros::Subscriber sub1_; // ROS subscriber

    ar_track_alvar_msgs::AlvarMarkers marker_pose_; // Member variables to store the Alvar ROS package data
    std::mutex marker_pose_mutex_;
    
    ar_track_alvar_msgs::AlvarMarkers marker_pose_for_calc_; // Member variables to manipulate Alvar data without affecting original data or causing data races.

    double refresh_rate_;

};







#endif // TRACKER_H
