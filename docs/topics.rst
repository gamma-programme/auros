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

Communications
--------------

.. data:: /comms/RadioStatus
**Message:** :ref:`mavros/RadioStatus`

Information about the current state of the communications link.

Aircraft State
--------------
Topics used to obtain information about the status of the FCU and aircraft power buses.

.. data:: /fcs/summary
**Message:** :ref:`mavros/VFR_HUD`

Summarises some important information on the current state of the aircraft, like airspeed, groundspeed, heading, altitude, etc.

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

.. data:: /fcs/global_position/rel_alt
**Message:** :ref:`std_msgs/Float64`

Information about the aircraft relative altitude in metres above ground level.

.. data:: /fcs/global_position/compass_hdg
**Message:** :ref:`std_msgs/Float64`

Information about the aircraft current heading in degrees.

.. data:: /fcs/global_position/gps_vel
**Message:** :ref:`geometry_msgs/Vector3Stamped`

Information about the aircraft ground velocity vector, as fused by FCU in m/s.

Applications
------------
.. data:: /apps/<application_name>/*

If you wish to take advantage of ROS's interprocess communication functionality by publishing your own services use this namespace. We recommend namespacing under your application name.

Mission
--------

.. data:: /mission/WaypointList
**Message:** :ref:`mavros/WaypointList`

Current waypoint list loaded into FCS.

.. data:: /mission/WaypointPull
**Message:** :ref:`mavros/WaypointPull`

Request waypoint from device. Aircraft responds with confirmation message.

.. data:: /mission/WaypointPush
**Message:** :ref:`mavros/WaypointPush`

Send waypoints to a device. Aircraft responds with confirmation message.

.. data:: /mission/WaypointClear
**Message:** :ref:`mavros/WaypointClear`

Clears waypoints stored in a device. Aircraft responds with confirmation message.

.. data:: /mission/WaypointSetCurrent
**Message:** :ref:`mavros/WaypointSetCurrent`

Sets current sequence number in the list. Aircraft responds with confirmation message.

.. data:: /mission/WaypointGOTO
**Message:** :ref:`mavros/WaypointGOTO`

Sends aircraft to a specific waypoint (currently only supported by APM firmware).