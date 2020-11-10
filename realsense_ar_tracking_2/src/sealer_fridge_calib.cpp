#include <ros/ros.h>
#include "calibrator.h"


int main(int argc, char** argv){
  // Initialise ROS Node
  ros::init(argc, argv, "sealer_fridge_calibrator");

  // ROS Node Handle
  ros::NodeHandle nh;

  // Create Calibrator Object
  std::shared_ptr<Calibrator> calibrator(new Calibrator(nh));

  // Calibration Thread
  std::thread t1(&Calibrator::calibrateSealerFridge, calibrator);

  // Transform Broadcasting Thread
  std::thread t2(&Calibrator::broadcastTagTargetTransform, calibrator);


  ros::spin();

  ros::shutdown();

  t1.join();
  t2.join();

  return 0;
}
