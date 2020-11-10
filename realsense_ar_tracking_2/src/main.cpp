#include "ros/ros.h"
#include "tracker.h"

int main(int argc, char **argv)
{
  // Initialise ROS Node
  ros::init(argc, argv, "realsenseARTagTracker");

  // ROS Node Handle
  ros::NodeHandle nh;

  // Create Tracker Object
  std::shared_ptr<Tracker> tracker(new Tracker(nh));

  // Tracking Thread
  std::thread t(&Tracker::trackTags,tracker);


  ros::spin();

  ros::shutdown();

  t.join();

  return 0;
}
