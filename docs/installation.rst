Installation
============
AUROS can installed by either adding some packages to an existing Ubuntu 14.04 system, or using one of the prebuilt virtual machines.

Virtual Machines
----------------

Prerequisites
^^^^^^^^^^^^^
* Oracle Virtualbox (https://www.virtualbox.org) or other OVF compatible Virtual Machine software
* 4GB ram (the VM defaults to using 2GB, although may work with less)
* 10GB disk space
* The auros_vm.zip file from the GAMMA BFS

Quickstart
^^^^^^^^^^
Unpack the supplied zip file into a folder.
Open Virtualbox and select File -> Import Appliance, then select the OVF file in the unpacked folder.
The import may take a few minutes as the virtual disk image is copied into Virtualbox.

The resulting machine will appear as "provisioning_default" followed by a string of numbers representing the build id of the VM.

Upon starting the VM will boot into a desktop and automatically log in the auros user. The auros examples and documentation are preinstalled on the desktop.

To fetch any updates to the auros examples since the VM was created:

- Open the auros folder on the desktop by double clicking.
- Right click within the folder and select *Open Terminal Here*
- In the resulting terminal window, type ``git pull`` and press enter

Existing Systems
----------------

Prerequisites
^^^^^^^^^^^^^
* A fully updated installation of Ubuntu 14.04 x86_64 (64 bit)
* A working internet connection

.. note:: 

  The aircraft use `Ubuntu Server <http://www.ubuntu.com/download/server>`_
  as the base, to avoid any unnecessary packages.

Quickstart
^^^^^^^^^^

The following commands will install the basic auros development environment on an Ubuntu 14.04 system::

  sudo wget http://gamma.mace.manchester.ac.uk/flexshare/repo/auros-platform.list \
    -P /etc/apt/sources.list.d/
  
  wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
  wget https://raw.githubusercontent.com/gamma-programme/auros/master/packaging/auros-public.key \
    -O - | sudo apt-key add -
    
  sudo apt-get update && sudo apt-get install auros-developer
  echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
  
Open a fresh terminal to ensure the changes are applied. You may now want to try building one of the examples.
  
.. note:: 
  If the ROS repository is already present in the system, apt will generate a warning on update.
  This is because the auros repository list also includes the ros repository.
