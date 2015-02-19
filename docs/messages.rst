Messages
========

.. _std_msgs/Header:

std_msgs/Header
---------------

Standard metadata for higher-level stamped data types. This is generally used to communicate timestamped data in a particular coordinate frame. ::

  # sequence ID: consecutively increasing ID 
  uint32 seq
  
  #Two-integer timestamp that is expressed as:
  # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
  # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
  # time-handling sugar is provided by the client library
  time stamp
  
  #Frame this data is associated with
  # 0: no frame
  # 1: global frame
  string frame_id

.. _std_msgs/Float64:

std_msgs/Float64
----------------

Message containing a single 64 bit float. ::

  float64 data

.. _sensor_msgs/Image:

sensor_msgs/Image
-----------------

Image data message.

Includes :ref:`std_msgs/Header`. ::

  # This message contains an uncompressed image
  # (0, 0) is at top-left corner of image
  #

  Header header        # Header timestamp should be acquisition time of image
                       # Header frame_id should be optical frame of camera
                       # origin of frame should be optical center of cameara
                       # +x should point to the right in the image
                       # +y should point down in the image
                       # +z should point into to plane of the image
                       # If the frame_id here and the frame_id of the CameraInfo
                       # message associated with the image conflict
                       # the behavior is undefined

  uint32 height         # image height, that is, number of rows
  uint32 width          # image width, that is, number of columns

  # The legal values for encoding are in file src/image_encodings.cpp

  string encoding       # Encoding of pixels -- channel meaning, ordering, size
                        # taken from the list of strings in include/sensor_msgs/image_encodings.h

  uint8 is_bigendian    # is this data bigendian?
  uint32 step           # Full row length in bytes
  uint8[] data          # actual matrix data, size is (step * rows)

.. _sensor_msgs/NavSatFix:

sensor_msgs/NavSatFix
---------------------

Navigation Satellite fix for Global Navigation Satellite System. Specified using the WGS 84 reference ellipsoid.

Includes :ref:`std_msgs/Header`, :ref:`sensor_msgs/NavSatStatus`. ::

  # header.stamp specifies the ROS time for this measurement (the
  #        corresponding satellite time may be reported using the
  #        sensor_msgs/TimeReference message).
  #
  # header.frame_id is the frame of reference reported by the satellite
  #        receiver, usually the location of the antenna.  This is a
  #        Euclidean frame relative to the vehicle, not a reference
  #        ellipsoid.
  Header header

  # satellite fix status information
  NavSatStatus status

  # Latitude [degrees]. Positive is north of equator; negative is south.
  float64 latitude

  # Longitude [degrees]. Positive is east of prime meridian; negative is west.
  float64 longitude

  # Altitude [m]. Positive is above the WGS 84 ellipsoid
  # (quiet NaN if no altitude is available).
  float64 altitude

  # Position covariance [m^2] defined relative to a tangential plane
  # through the reported position. The components are East, North, and
  # Up (ENU), in row-major order.
  #
  # Beware: this coordinate system exhibits singularities at the poles.

  float64[9] position_covariance

  # If the covariance of the fix is known, fill it in completely. If the
  # GPS receiver provides the variance of each measurement, put them
  # along the diagonal. If only Dilution of Precision is available,
  # estimate an approximate covariance from that.

  uint8 COVARIANCE_TYPE_UNKNOWN = 0
  uint8 COVARIANCE_TYPE_APPROXIMATED = 1
  uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
  uint8 COVARIANCE_TYPE_KNOWN = 3

  uint8 position_covariance_type

.. _sensor_msgs/NavSatStatus:

sensor_msgs/NavSatStatus
------------------------

Navigation Satellite fix status for Global Navigation Satellite System. ::

  # Whether to output an augmented fix is determined by both the fix
  # type and the last time differential corrections were received.  A
  # fix is valid when status >= STATUS_FIX.

  int8 STATUS_NO_FIX =  -1        # unable to fix position
  int8 STATUS_FIX =      0        # unaugmented fix
  int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
  int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

  int8 status

  # Bits defining which Global Navigation Satellite System signals were
  # used by the receiver.

  uint16 SERVICE_GPS =     1
  uint16 SERVICE_GLONASS = 2
  uint16 SERVICE_COMPASS = 4      # includes BeiDou.
  uint16 SERVICE_GALILEO = 8

  uint16 service
  
.. _geometry_msgs/Quaternion:

geometry_msgs/Quaternion
------------------------

Represents an orientation in free space in quaternion form::

  float64 x
  float64 y
  float64 z
  float64 w
  
.. _geometry_msgs/Vector3:

geometry_msgs/Vector3
---------------------

Message containing a 64-bit 3 element vector::

  # This represents a vector in free space. 

  float64 x
  float64 y
  float64 z

.. _geometry_msgs/Vector3Stamped:

geometry_msgs/Vector3Stamped
----------------------------

Message containing a Vector3 with reference coordinate frame and timestamp.

Includes :ref:`std_msgs/Header`, :ref:`geometry_msgs/Vector3`. ::

  Header header
  Vector3 vector

.. _mavros/State:

mavros/State
------------

Current autopilot state. Unlikely to be relevant for the majority of applications.

Includes :ref:`std_msgs/Header`. ::

  Header header
  bool armed
  bool guided
  string mode
  
.. _mavros/BatteryStatus:
  
mavros/BatteryStatus
--------------------

Used to obtain information about the airframe battery status. 
On airframes with internal combustion engines, this will return the voltage of the 12VDC bus. 
The remaining value is an estimate used internally by the FCS for failsafe trigger actions and should not be relied upon.

Includes :ref:`std_msgs/Header`. ::

  Header header
  float32 voltage # [V]
  float32 current # [A]
  float32 remaining # 0..1

.. _mavros/VFR_HUD:
  
mavros/VFR_HUD
--------------

Used to obtain a summary of the aircraft current state. Includes Airspeed [m/s], ground speed [m/s], heading [degrees], thorttle [0-1], altitude [m amsl], and climb rate [m/s].

Includes :ref:`std_msgs/Header`. ::

  # Metrics typically displayed on a HUD
  
  Header header
  float32 airspeed # m/s
  float32 groundspeed # m/s
  int16 heading # degrees 0..360
  float32 throttle # normalized to 0.0..1.0
  float32 altitude # MSL
  float32 climb # current climb rate m/s

.. _mavros/RadioStatus:
 
mavros/RadioStatus
------------------

Used to obtain current status of communications link betweek the aircraft and Auros.

Includes :ref:`std_msgs/Header`. ::

  # RADIO_STATUS message

  Header header
  uint8 rssi
  uint8 remrssi
  uint8 txbuf
  uint8 noise
  uint8 remnoise
  uint16 rxerrors
  uint16 fixed

.. _mavros/Waypoint

mavros/Waypoint
---------------

Auros representation of a mission item. See the mavlink documentation (https://pixhawk.ethz.ch/mavlink/) for more information, specifically MAV_CMD, CMD ID 16.

  # see enum MAV_FRAME
  uint8 frame
  uint8 FRAME_GLOBAL = 0
  uint8 FRAME_LOCAL_NED = 1
  uint8 FRAME_MISSION = 2
  uint8 FRAME_GLOBAL_REL_ALT = 3
  uint8 FRAME_LOCAL_ENU = 4

  # see enum MAV_CMD
  uint16 command
  uint16 NAV_WAYPOINT = 16
  uint16 NAV_LOITER_UNLIM = 17
  uint16 NAV_LOITER_TURNS = 18
  uint16 NAV_LOITER_TIME = 19
  uint16 NAV_RETURN_TO_LAUNCH = 20
  uint16 NAV_LAND = 21
  uint16 NAV_TAKEOFF = 22
  # TODO: ROI mode

  bool is_current
  bool autocontinue
  # meaning of this params described in enum MAV_CMD
  float32 param1
  float32 param2
  float32 param3
  float32 param4
  float64 x_lat
  float64 y_long
  float64 z_alt

.. _mavros/WaypointList:

mavros/WaypointList
--------------------

Used to obtain current waypoint list.

Includes :ref:`mavros/Waypoint`. ::

  mavros/Waypoint[] waypoints

