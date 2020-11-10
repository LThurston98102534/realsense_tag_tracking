#include <ros/ros.h>
#include "alvar_tester.h"


int main(int argc, char** argv){
  // Initialise ROS node
  ros::init(argc, argv, "alvar_tester");

  // ROS Node Handle
  ros::NodeHandle nh;

  // Create Calibrator Object
  std::shared_ptr<AlvarTester> alvar_tester(new AlvarTester(nh));

  // Calibration Thread
  std::thread t1(&AlvarTester::testAlvar, alvar_tester);


  ros::spin();

  ros::shutdown();

  t1.join();

  return 0;
}
