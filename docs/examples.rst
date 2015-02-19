Examples
========

.. highlight:: c++

Foo ::

  // Include the ROS C++ APIs
  #include <ros/ros.h>

  // Standard C++ entry point
  int main(int argc, char** argv) {
    // Announce this program to the ROS master as a "node" called "hello_world_node"
    ros::init(argc, argv, "hello_world_node");
    // Start the node resource managers (communication, time, etc)
    ros::start();
    // Broadcast a simple log message
    ROS_INFO_STREAM("Hello, world!");
    // Process ROS callbacks until receiving a SIGINT (ctrl-c)
    ros::spin();
    // Stop the node's resources
    ros::shutdown();
    // Exit tranquilly
    return 0;
  }

Examples are present in the examples folder.
By default, auROS uses the ROS build system, which uses cmake. 
Typically, out of tree builds are used, so cleaning up only
involves removing the build directory.

To build the first example::

  cd examples/hello_auros
  mkdir build
  cd build
  cmake ..
  make
  
And run it (the default path is somewhat odd as a side-effect of the build setup)::

  roscore &
  ./devel/lib/hello_auros/hello_auros
