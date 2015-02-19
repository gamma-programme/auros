Services
========

Services are usually used by applications to request actions from the Flight Control System.

Mission
--------

.. _mavros/WaypointPull:
 
mavros/WaypointPull
-------------------

Request current waypoints from fcs.
	
	Includes :ref:`mavros/Waypoint`. ::

	# Requests waypoints from device
	#
	# Returns success status and received count

	---
	bool success
	uint32 wp_received

.. _mavros/WaypointPush:

mavros/WaypointPush
-------------------

Send waypoints to a device.

Includes :ref:`mavros/Waypoint`. ::

	# Send waypoints to device
	#
	# Returns success status and transfered count

	mavros/Waypoint[] waypoints
	---
	bool success
	uint32 wp_transfered

.. _mavros/WaypointClear:

mavros/WaypointClear
--------------------

Clears current list of waypoints of a device. ::

	# Request clear waypoint

	---
	bool success

.. _mavros/WaypointSetCurrent:

mavros/WaypointSetCurrent
-------------------------

Sets waypoint number to go to now. From the current list of waypoints ::

	# Request set current waypoint
	#
	# wp_seq - index in waypoint array

	uint16 wp_seq
	---
	bool success

.. _mavros/WaypointGOTO:

mavros/WaypointGOTO
-------------------

Sends platform to a specific location.

Includes :ref:`mavros/Waypoint`. ::

# Request go to waypoint
#
# Only supported FCU will return result

mavros/Waypoint waypoint
---
bool success
