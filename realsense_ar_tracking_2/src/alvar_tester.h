#ifndef ALVARTESTER_H
#define ALVARTESTER_H

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <iostream>
#include <limits>

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

class AlvarTester{
public:
    AlvarTester(ros::NodeHandle nh); // Constructor

    ~AlvarTester(); // Destructor


    void trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg); // Alvar package callback

    void testAlvar(); // Main loop used to obtain the Alvar package data and display the respective transforms to the terminal


    ros::NodeHandle nh_; // Node Handle

private:
    tf::TransformListener* listener_; // TF listener used to obtain the relative transform between frames in the TF tree
    tf::TransformBroadcaster* broadcaster_; // TF broadcaster used to broadcast relative transforms between frames in the TF tree

    ros::Subscriber sub1_; // ROS subscriber

    ar_track_alvar_msgs::AlvarMarkers marker_pose_; // Marker pose member variable updated during the callback
    std::mutex marker_pose_mutex_;
    
    ar_track_alvar_msgs::AlvarMarkers marker_pose_for_calc_; // Marker pose for calc variable used to conduct all the multiplication and mathematical functions with to prevent data races with the marker_pose_ variable

    int calib_pos_count_; // Counter of the number of calibration positions reached

    // Cumulative variables used to calculate the rolling average transform throughout the execution of the package
    double cumulative_rot_x_;
    double cumulative_rot_y_;
    double cumulative_rot_z_;
    double cumulative_rot_w_;

    double cumulative_trans_x_;
    double cumulative_trans_y_;
    double cumulative_trans_z_;

    tf::Transform camera_tag_transform_;
    std::mutex camera_tag_transform_mutex_;

};







#endif // ALVARTESTER_H
