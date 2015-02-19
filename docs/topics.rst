Topics
======

ROS nodes publish data using topics. These topics are represented by a path, such as
"/sensors/eo/visible/image_raw". Whilst topics can be trivially remapped within ROS, we recommend
using the following conventions for simplicity.

Sensors
-------
The root category for sensors not used by the flight control unit, such as cameras and thermal imagers.

.. data:: /sensors/eo/visible/image_raw
**Message:** :ref:`sensor_msgs/Image`

  Use this feed for general purpose onboard colour cameras (IDS UEye, GoPro Hero HD3).
  
.. data:: /sensors/eo/ir/image_raw
**Message:** :ref:`sensor_msgs/Image`

  Use this feed for a general purpose onboard Far Infra-Red feed (FLIR Tau 2).
  
.. data:: /sensors/eo/nir/image_raw
**Message:** :ref:`sensor_msgs/Image`

  Use this feed for a general purpose onboard Near Infra-Red camera (IDS UEye NIR).

Aircraft State
--------------
Topics used to obtain information about the status of the FCU and aircraft power buses.

.. data:: /fcs/state
**Message:** :ref:`mavros/State`

Can be used to obtain information about the flight controller's arming state and guidance modes. Unlikely to be relevant for the majority of applications.

.. data:: /fcs/battery
**Message:** :ref:`mavros/BatteryStatus`

Used to obtain information about the airframe battery status. 
On airframes with internal combustion engines, voltage returned will be that of the 12VDC bus. 
The remaining value is an estimate used internally by the FCS for failsafe trigger actions and should not be relied upon.

Aircraft Position
-----------------

.. data:: /fcs/global_position/global
**Message:** :ref:`sensor_msgs/NavSatFix`

Information about the aircraft global position, fused by FCU.

Applications
------------
.. data:: /apps/<application_name>/*
If you wish to take advantage of ROS's interprocess communication functionality by publishing your own services use this namespace. We recommend namespacing under your application name.
