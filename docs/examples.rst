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

One way to link an existing piece of software with ROS is using CMake (http://www.cmake.org/).
This is the build system used by ROS. An example of integration of an existing piece of software with ROS is below.

The following file should be saved as CMakeLists.txt and expects source code to be in a src subdirectory. 
For more information on how ROS uses CMake, see http://wiki.ros.org/catkin/CMakeLists.txt. ::

  cmake_minimum_required(VERSION 2.8.3)
  project(rospitch)

  find_package(catkin REQUIRED COMPONENTS
    mavros
  )
  
  # BUILD

  ## Specify additional locations of header files
  ## Your package locations should be listed before other locations
  # include_directories(include)
  include_directories(
    /opt/prti1516e/include
    ${catkin_INCLUDE_DIRS}
  )

  link_directories(
    /opt/prti1516e/lib/gcc41_64
    /opt/prti1516e/lib/gcc41_64/newtime
    /opt/prti1516e/jre/lib/amd64
    /opt/prti1516e/jre/lib/amd64/native_threads
    /opt/prti1516e/jre/lib/amd64/server
  )

  ## Declare a cpp executable
  add_executable(rospitch_node src/rospitch_node.cpp)

  ## Specify libraries to link a library or executable target against
  target_link_libraries(rospitch_node
    rti1516e64
    fedtime1516e64
    java
    jvm
    verify
    ${catkin_LIBRARIES}
  )

  # Install

  ## Mark executable scripts (Python etc.) for installation
  ## in contrast to setup.py, you can choose the destination
  install(PROGRAMS
    scripts/rospitch
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  ## Mark executables and/or libraries for installation
  install(TARGETS rospitch_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
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
