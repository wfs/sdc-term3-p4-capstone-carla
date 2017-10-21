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

Diagram Classes and Functions
=============================
 
.. toctree::
   :maxdepth: 2
   :numbered: 2

.. module:: waypoint_loader

.. autoclass:: WaypointLoader
    :members:

.. module:: tl_detector

.. autoclass:: TLDetector
    :members: image_cb

.. module:: tldetect

.. autoclass:: predictor

.. module:: waypoint_updater

.. autoclass:: WaypointUpdater
    :members: loop, get_waypoints

.. module:: dbw_node 

.. autoclass:: DBWNode 
    :members: loop, dbw_enabled_cb

