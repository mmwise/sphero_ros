sphero_ros Tutorials
====================

Quick Start
+++++++++++

Already familiar with ROS and want to get started?

Install sphero_ros
------------------

Follow the regular groovy ROS installation instructions:

http://www.ros.org/wiki/groovy/Installation/Ubuntu

Then install sphero_ros::

  sudo apt-get install sphero_ros                                                     
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

Start roscore
-------------

In a terminal, run::

  roscore

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

Sphero "Hello World"
++++++++++++++++++++

Make the Sphero move right away. Follow the steps in the Quick Start
section. Then get to publishing commands::

  rostopic pub cmd_vel geometry_msgs/Twist -r 10 '[1, 0, 0]' '[0, 0, 0]'

The Sphero should start rolling away at 1m/s. 

Change the Sphero color::
  
  rostopic pub set_color std_msgs/ColorRGBA 1 0 0

The Sphero should turn RED. 

Check Out the Data
++++++++++++++++++

See what the different sensors are publishing, for example the imu::

  rostopic echo imu

Or see what happens when you collide with the world subscribe to
collision::

  rostopic echo collision
