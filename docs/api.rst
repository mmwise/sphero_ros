.. _python_api:

sphero_ros Python & Node API
==================

Node API
--------

sphero.py
~~~~~~~~~

sphero.py provides a simple ROS wrapper for the sphero_driver.

Subscribed Topics
+++++++++++++++++

``disable_stabilization`` (std_msgs/Bool)
 Turns the auto stabilization of the Sphero on and off. True for off and False for on.
``cmd_vel`` (geometry_msgs/Twist)
 The commanded velocity of the Sphero. 
``set_angular_velocity`` (std_msgs/Float32)
 The angular velocity of the sphero.
``set_back_led`` (std_msgs/Float32)
 Changes the brightness of the back white LED in the Sphero. Completely off is 0.0 and full brightness 1.0.
``set_color`` (std_msgs/ColorRGBA)
 Sets the RGB color of the Sphero. 
``set_heading`` (std_msgs/Float32)
 Immediately changes the heading of the Sphero to the commanded angle. Will move in the shortest angular distance to the commanded heading.

Published Topics
+++++++++++++++++

``collision`` (sphero_node/SpheroCollision)
 Publishes collision data when the Sphero collides with the environment.
``diagnostics`` (diagnostic_msgs/DiagnosticArray)
 Publishes relevant information (i.e. battery status).
``imu`` (snesor_msgs/Imu)
 The Sphero imu data.
``odom`` (nav_msgs/Odometry)
 The odometric state of the Sphero.

Parameters
++++++++++

``~cmd_vel_timeout`` (float, default: 0.6)
 The time from when the last cmd_vel was published to set cmd_vel to zero.
``~connect_blue`` (int, default: 0)
 Set the blue value of the color the Sphero turns on connection.
``~connect_green`` (int, default: 255)
 Set the green value of the color the Sphero turns on connection.
``~connect_red`` (int, default: 0)
 Set the red value of the color the Sphero turns on connection.
``~diag_update_rate`` (float, default: 1.0)
 Sets the update rate for the diagnostics.

Python API
----------
The Python API for the underlying Sphero driver.

sphero_driver
~~~~~~~~~~~~~~
.. module:: sphero_driver

.. autoclass:: Sphero
   :members:
