# realsense_tag_tracking

Current ROS dependencies:

1) ar_track_alvar
Installation:
- Navigate to ~/catkin_ws and type:

$ sudo apt-get install ros-melodic-ar-track-alvar


- Navigate to ~/catkin_ws/src and type:

$ git clone https://github.com/ros-perception/ar_track_alvar.git



2) Realsense2 ROS Package
Installation instructions:

https://github.com/IntelRealSense/realsense-ros#installation-instructions


3) Realsense AR Tracking
Installation:

- Navigate to ~/Documents and type:

$ mkdir KUKA_AR_Tracking
$ cd KUKA_AR_Tracking
$ git clone https://github.com/LThurston98102534/Capstone.git

- Navigate to ~/catkin_ws/src and type:

$ ln -s ~/Documents/KUKA_AR_Tracking/Capstone/prototyping/realsense_ar_tracking_2/
