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
 
- Navigate to ~/catkin_ws/src and type:
 
$ git clone https://github.com/LThurston98102534/realsense_tag_tracking.git
 
$ cd realsense_tag_tracking
 
$ mv realsense_ar_tracking_2 ../
 
 
 
4) File Structure Setup
 
- Read the 'Repository File Descriptions.pdf' file attached to this repository and move respective files to their required locations as detailed in this document.
 
 
 
5) Example Launch Commands
 
- Navigate to ~/catkin_ws/ and type:
 
$ catkin build
 
$ roslaunch realsense_ar_tracking_2 realsense_track.launch
 
 
 
 
 
Additional description of the various files and their functionalities can be found both embedded in comments inside the files themselves as well as in the 'Repository File Descriptions.pdf' document.
