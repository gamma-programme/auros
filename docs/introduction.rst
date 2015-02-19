Introduction
============

About this Document
-------------------

This document details the message structures, interprocess communication channels and provides samples for the auROS unmanned aircraft middleware.
It also acts as the interface control document for partners in the GAMMA Programme.

.. image:: images/cc.png
  :align: center
  :height: 32px
  :width: 88px
  :scale: 50%

This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License (http://creativecommons.org/licenses/by-sa/4.0/).

The document can be viewed online at 

http://auros.readthedocs.org.

Anyone is welcome to submit changes to this document for inclusion in the next version at

https://github.com/gamma-programme/auros.

What is auROS?
--------------

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
