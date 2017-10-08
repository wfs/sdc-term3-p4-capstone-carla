.. RoboFolks documentation master file, created by
   sphinx-quickstart on Wed Oct  4 16:43:46 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. Welcome to RoboFolks documentation!
.. =====================================

Indices and tables
==================
 
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Classes and Functions
=====================
 
.. toctree::
   :maxdepth: 2
   :numbered: 2

.. module:: waypoint_loader

.. autoclass:: WaypointLoader
    :members:

.. module:: waypoint_updater

.. autoclass:: WaypointUpdater

.. module:: dbw_node 

.. autoclass:: DBWNode 
    :members: loop, cte_calc, transform_waypoints, get_euler, dbw_enabled_cb

.. module:: twist_controller

.. autoclass:: Controller
    :members: control, control_speed_based_on_proportional_throttle_brake

.. module:: pid

.. autoclass:: PID
    :members:

.. module:: yaw_controller

.. autoclass:: YawController
    :members:

