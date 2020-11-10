#ifndef CALIBRATOR_H
#define CALIBRATOR_H

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

class Calibrator{
public:
    Calibrator(ros::NodeHandle nh); // Constructor

    ~Calibrator(); // Destructor


    void trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg); // Alvar package callback

    void calibrateCamera(); // Base to Camera Calibration function

    void calibrateSealerFridge(); // Sealer/Fridge tag to target calibration function

    void broadcastBaseCameraTransform(); // Broadcasting function run by another thread to publish calibrated transform

    void broadcastTagTargetTransform(); // Broadcasting function run by another thread to publish calibrated transform

    ros::NodeHandle nh_; // Nodehandle

private:
    tf::TransformListener* listener_; // TF listener used to obtain relative transforms from the TF tree
    tf::TransformBroadcaster* broadcaster_; // TF broadcaster used to publish relative transforms to the TF tree

    ros::Subscriber sub1_; // ROS subscriber used to subscribe to the Alvar package

    ar_track_alvar_msgs::AlvarMarkers marker_pose_; // Member variable used to store the data from the Alvar package
    std::mutex marker_pose_mutex_;
    
    ar_track_alvar_msgs::AlvarMarkers marker_pose_for_calc_; // Member variable used to manipulate the Alvar package data without affecting the actual data and without causing data races

    double refresh_rate_; 
    int calib_pos_count_; // Counter used to track the number of calibration positions used
    int arm_position_; // Variable used to track the intended arm position the user wishes to drive to
    int transform_set_; // flag used to indicate the base to camera transform has been calculated at least once
    std::mutex transform_set_mutex_;

    int sealer_transform_set_; // flag used to indicate the sealer tag to target transform has been calculated at least once
    std::mutex sealer_transform_set_mutex_;

    int fridge_transform_set_; // flag used to indicate the fridge tag to target transform has been calculated at least once
    std::mutex fridge_transform_set_mutex_;

    // Cumulative variables used to calculate the rolling average of the calibrated transforms
    double cumulative_rot_x_;
    double cumulative_rot_y_;
    double cumulative_rot_z_;
    double cumulative_rot_w_;

    double cumulative_trans_x_;
    double cumulative_trans_y_;
    double cumulative_trans_z_;

    // Member variables used to store the calculated average transforms
    tf::Transform average_base_camera_transform_;
    std::mutex base_camera_mutex_;


    tf::Transform average_tag_target_transform_;
    std::mutex tag_target_mutex_;

    tf::Transform camera_tag_transform_;
    std::mutex camera_tag_transform_mutex_;

};


#endif // CALIBRATOR_H
