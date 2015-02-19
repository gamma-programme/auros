Examples
========

Standalone Nodes
----------------

Hello World
^^^^^^^^^^^
.. highlight:: c++

The following illustrates a basic C++ ROS node which does not subscribe to any topics. ::

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
  
Subscribing to Topics
^^^^^^^^^^^^^^^^^^^^^
.. highlight:: c++

The following example subscribes to the waypoint topic which publishes the waypoints currently loaded into the FCU. The callback will be executed whenever there is a change in the waypoint list. ::

  #include <ros/ros.h>
  #include <mavros/Waypoint.h>
  #include <mavros/WaypointList.h>

  void wp_cb(const mavros::WaypointList& wp_list)
  {
    std::vector<mavros::Waypoint> waypoints = wp_list.waypoints;
    
    for(int i = 0; i < waypoints.size(); i++) 
    {
      ROS_INFO("Waypoint loaded @ %f, %f", waypoints[i].x_lat, waypoints[i].y_long);
    }
  }

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "getting_waypoints");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/fcu/mission/waypoints", 1, wp_cb);
    ROS_INFO("Setup complete");
    ros::spin();
    return 0;
  }

Making Requests with Services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  
Integrating Existing Software
-----------------------------

Using ROS in software with an existing event loop is fairly simple. 
The ros::spinOnce() call will process all callbacks waiting in the queue. This function must be called faster than the highest rate subscribed topic or messages will be dropped.

For more information about more advanced topics (including callbacks across multiple threads) see http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning.

::

  // Include the ROS C++ APIs
  #include <ros/ros.h>
  
  void existing_event_loop() {
    read_sensor();
    
    do_some_processing();
    
    set_some_waypoints();
  }

  // Standard C++ entry point
  int main(int argc, char** argv) {
    // Announce this program to the ROS master as a "node" called "hello_world_node"
    ros::init(argc, argv, "hello_world_node");
    // Start the node resource managers (communication, time, etc)
    ros::start();
    
    existing_setup();
    
    while(condition) {
      existing_event_loop();
      ros::spinOnce();
    }
    
    existing_teardown();
    
    ros::shutdown();
    // Exit tranquilly
    return 0;
  }

Using CMake
^^^^^^^^^^^

The easiest way to build an existing 
  
Building Examples
---------------------

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
