Hierachy
========

sensors/eo
-----------
The root category for electro-optical sensors (visible cameras, thermal imagers).

.. data:: /visible/image_raw

  Use this feed for the best general purpose onboard colour camera (IDS UEye, GoPro Hero HD3).
  
.. data:: /ir/image_raw

  Use this feed for a general purpose onboard Far Infra-Red feed (FLIR Tau 2).
  
.. data:: /nir/image_raw

  Use this feed for a general purpose onboard Near Infra-Red camera (IDS UEye NIR).

sensors/atmos
--------------
Category for atmospheric sensors not included as part of the flight control unit.

.. data:: /five_hole
  
  Access to the 5 hole turbulence probe (Aeroprobe microADC).

apps/
-----
Use this namespace for your own services & topics. We recommend namespacing under your application name.