Services
========

Services are used by applications to request actions from the Flight Control System.

Mission
--------
The mission services are used to make alterations to the aircraft's waypoints.

.. data:: /mission/WaypointPull
**Message:** :ref:`mavros/WaypointPull`

Request list of waypoints from FCS. Aircraft responds with confirmation message.

.. data:: /mission/WaypointPush
**Message:** :ref:`mavros/WaypointPush`

Send waypoints to FCS. Aircraft responds with confirmation message.

.. data:: /mission/WaypointClear
**Message:** :ref:`mavros/WaypointClear`

Clears waypoints stored in FCS. Aircraft responds with confirmation message.

.. data:: /mission/WaypointSetCurrent
**Message:** :ref:`mavros/WaypointSetCurrent`

Sets current sequence number in the FCS. Aircraft responds with confirmation message.

.. data:: /mission/WaypointGOTO
**Message:** :ref:`mavros/WaypointGOTO`

Sends aircraft to a specific waypoint.
