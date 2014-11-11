Introduction
============

auROS is a distribution of ROS tailored for use in unmanned aircraft (especially those supporting MAVLink).
It tries to make it easy to write applications that can update an aircraft's mission as it unfolds.
auROS is designed to run on a companion 'mission management' computer which is connected to the flight controller
but does not directly control the aircraft.

Features
--------

- Process data from visible & thermal sensors in a portable manner
- Change waypoints, get aircraft status & talk to ground stations
- Communicate across multiple unmanned platforms

Why auROS?
----------
auROS may seem daunting at first - there are several moving parts and it can take a bit of reading to 
be comfortable with how it all fits together. However, we've found that using auROS has several advantages:
- Quicker development
- Smaller, simpler codebases
- Many pre-existing mature, reusable packages
