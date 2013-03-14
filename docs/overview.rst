sphero_ros Overview
==================

sphero_ros contains several packages for controlling and using a sphero with ROS. 
 
* sphero_driver: A pure python driver for sphero. The driver
  implements most of the Sphero 1.2 API.
* sphero_node: A ROS wrapper for the sphero_driver and exposes all the
  topics and services you would expect to find in a ROS node. For more
  information checkout the node documentation.
* sphero_description: A URDF for sphero. 

Quick Start
===========

Already familiar with ROS and want to get started? 

Pair the Sphero
--------------

Since Sphero has a PIN, the first step is to setup a new bluetooth
device.
 
* Shake the Sphero to turn it on.
* Go into the bluetooth setup screen and select the device in the list
  with Sphero in the name.
* Select the device and set the PIN option to '0000'.

Start the Node
--------------

Now start the Sphero node::

 rosrun sphero_node sphero.py

You should see::
  
 Searching for devices............
 Found Sphero device with address: XX:XX:XX:XX:XX:XX
 Paired with Sphero.

Your Sphero will turn GREEN.

If the pairing process fails, you will see::

 [ERROR] [WallTime: 1363235908.174193] Failed to connect to Sphero.
