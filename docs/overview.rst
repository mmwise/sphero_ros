sphero_ros Overview
===================

sphero_ros contains several packages for controlling and using a sphero with ROS. 

* sphero_bringup: A launch file to bring up the sphero_node and other
  relevant node for playing with odometry.
* sphero_description: A URDF for sphero. 
* sphero_driver: A pure python driver for sphero. The driver
  implements most of the Sphero 1.2 API.
* sphero_node: A ROS wrapper for the sphero_driver and exposes all the
  topics and services you would expect to find in a ROS node. For more
  information checkout the node documentation.

Quick Start
===========

Already familiar with ROS and want to get started? 

Install sphero_ros
------------------

Follow the regular groovy ROS installation instructions:

http://www.ros.org/wiki/groovy/Installation/Ubuntu

Then install sphero_ros::

  sudo apt-get install ros-groovy-sphero-bringup

Update the Sphero
-----------------

Make sure that the Sphero firmware is up to date. Currently sphero_ros
supports firmware 1.20 or later. There currently isn't a version check
in the driver but anyone is welcome to contribute!

* Pair the Sphero with an Android or iOS device.
* Open the Sphero App. 
* Select the info icon to check the current firmware version.
* Update if possible. 

Pair the Sphero
---------------

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

Using the Launch File
---------------------

Now that you have confirmed that the sphero_node starts properly, run
the launch file that loads the URDF and brings up robot_pose_ekf and
robot_state_publisher::

  roslaunch sphero_bringup sphero.launch

Your Sphero will turn GREEN.
