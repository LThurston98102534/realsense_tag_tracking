#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  // Initialise ROS node
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  // Create a TF broadcaster
  tf::TransformBroadcaster broadcaster;

  // Create a quaternion with calibrated base to camera results (SHOULD BE UPDATED WITH ANY RE-CALIBRATED RESULTS)
  tf::Quaternion base_camera_quaternion(-0.012, 0.007, 0.713, 0.701);
  base_camera_quaternion = base_camera_quaternion.normalize();

  while(n.ok()){

     // Broadcast relative base to camera transform calculated during the calibration process (VALUES SHOULD BE UPDATED WITH ANY RE-CALIBRATED RESULTS)
     broadcaster.sendTransform(tf::StampedTransform(tf::Transform(base_camera_quaternion, tf::Vector3(-0.145, 0.04725, 0.21525)),ros::Time::now(),"base_link", "camera_base_link"));
          
    
     // TO BE ADDED: RELATIVE TRANSFORMS BETWEEN SEALER/FRIDGE TAGS AND TARGET LOCATIONS
              


    r.sleep();
  }
}
