Examples
========

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
