// Include the ROS C++ APIs
#include <ros/ros.h>

// Standard C++ entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master as a "node" called "hello_auros_node"
  ros::init(argc, argv, "hello_auros_node");

  // Start the node resource managers (communication, time, etc)
  ros::start();

  // Broadcast a simple log message
  ROS_INFO_STREAM("Hello, auROS!");

  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();

  // Stop the node's resources
  ros::shutdown();

  return 0;
}