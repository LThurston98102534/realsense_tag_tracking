#include <ros/ros.h>
#include "calibrator.h"


int main(int argc, char** argv){
  // Initialise ROS node
  ros::init(argc, argv, "camera_calibrator");

  // ROS Node Handle
  ros::NodeHandle nh;

  // Create Calibrator Object
  std::shared_ptr<Calibrator> calibrator(new Calibrator(nh));

  // Calibration Thread
  std::thread t1(&Calibrator::calibrateCamera, calibrator);
  // Transform Broadcasting Thread
  std::thread t2(&Calibrator::broadcastBaseCameraTransform, calibrator);


  ros::spin();

  ros::shutdown();

  t1.join();
  t2.join();

  return 0;
}
