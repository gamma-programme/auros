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

The easiest way to build an existing piece of software with ROS is using CMake (http://www.cmake.org/).
This is the build system used by ROS. An example of integration an existing piece of software with ROS is below.

The following file should be saved as CMakeLists.txt. For more information on how ROS uses CMake, see http://wiki.ros.org/catkin/CMakeLists.txt.

  cmake_minimum_required(VERSION 2.8.3)
  project(rospitch)

  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    mavros
  )

  ## System dependencies are found with CMake's conventions
  # find_package(Boost REQUIRED COMPONENTS system)


  ## Uncomment this if the package has a setup.py. This macro ensures
  ## modules and global scripts declared therein get installed
  ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
  # catkin_python_setup()

  ################################################
  ## Declare ROS messages, services and actions ##
  ################################################

  ## To declare and build messages, services or actions from within this
  ## package, follow these steps:
  ## * Let MSG_DEP_SET be the set of packages whose message types you use in
  ##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
  ## * In the file package.xml:
  ##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
  ##   * If MSG_DEP_SET isn't empty the following dependencies might have been
  ##     pulled in transitively but can be declared for certainty nonetheless:
  ##     * add a build_depend tag for "message_generation"
  ##     * add a run_depend tag for "message_runtime"
  ## * In this file (CMakeLists.txt):
  ##   * add "message_generation" and every package in MSG_DEP_SET to
  ##     find_package(catkin REQUIRED COMPONENTS ...)
  ##   * add "message_runtime" and every package in MSG_DEP_SET to
  ##     catkin_package(CATKIN_DEPENDS ...)
  ##   * uncomment the add_*_files sections below as needed
  ##     and list every .msg/.srv/.action file to be processed
  ##   * uncomment the generate_messages entry below
  ##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

  ## Generate messages in the 'msg' folder
  # add_message_files(
  #   FILES
  #   Message1.msg
  #   Message2.msg
  # )

  ## Generate services in the 'srv' folder
  # add_service_files(
  #   FILES
  #   Service1.srv
  #   Service2.srv
  # )

  ## Generate actions in the 'action' folder
  # add_action_files(
  #   FILES
  #   Action1.action
  #   Action2.action
  # )

  ## Generate added messages and services with any dependencies listed here
  # generate_messages(
  #   DEPENDENCIES
  #   std_msgs  # Or other packages containing msgs
  # )

  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if you package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES rospitch
  #  CATKIN_DEPENDS mavros
  #  DEPENDS system_lib
  )

  ###########
  ## Build ##
  ###########

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

  #############
  ## Install ##
  #############

  # all install targets should use catkin DESTINATION variables
  # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

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

  ## Mark cpp header files for installation
  # install(DIRECTORY include/${PROJECT_NAME}/
  #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  #   FILES_MATCHING PATTERN "*.h"
  #   PATTERN ".svn" EXCLUDE
  # )

  ## Mark other files for installation (e.g. launch and bag files, etc.)
  # install(FILES
  #   # myfile1
  #   # myfile2
  #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # )

  #############
  ## Testing ##
  #############

  ## Add gtest based cpp test target and link libraries
  # catkin_add_gtest(${PROJECT_NAME}-test test/test_rospitch.cpp)
  # if(TARGET ${PROJECT_NAME}-test)
  #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  # endif()

  ## Add folders to be run by python nosetests
  # catkin_add_nosetests(test)
  
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
