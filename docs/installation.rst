Installation
============
AUROS is installed by adding 

Prerequisites
-------------
* A fully updated installation of Ubuntu 14.04 x86_64 (64 bit)
* A working internet connection

.. note:: 

  The aircraft use the `minimal cd <http://archive.ubuntu.com/ubuntu/dists/trusty/main/installer-amd64/current/images/netboot/mini.iso>`_
  as the base, to avoid any unnecessary packages.



Quickstart::
  
  sudo wget http://gamma.mace.manchester.ac.uk/flexshare/repo/auros-platform.list -P /etc/apt/sources.list.d/
  wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
  wget https://raw.githubusercontent.com/gamma-programme/auros-packaging/master/auros.key -O - | sudo apt-key add -
  sudo apt-get update && sudo apt-get install auros-developer
.. note:: 
  If the ROS repository is already present in the system, apt will generate a warning on update.
  This is because the auros repository list also includes the ros repository.
  To fix this, removing the ros sources list::
    rm /etc/apt/TODO