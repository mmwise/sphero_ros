#!/usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2012, Melonee Wise
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************
#author: Melonee Wise

import roslib; roslib.load_manifest('sphero_node')
import rospy

import PyKDL
import math

from sphero_driver import Sphero

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from std_msgs.msg import ColorRGBA, Float32
import sys

class SpheroNode(object):

    def __init__(self, default_update_rate=50.0):
        rospy.init_node('sphero')
        self.update_rate = default_update_rate
        self.sampling_divisor = int(400/self.update_rate)

        self._init_pubsub()
        self._init_params()
        self.robot = Sphero()
        self.imu = Imu(header=rospy.Header(frame_id="imu_link"))
        self.imu.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.imu.angular_velocity_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.imu.linear_acceleration_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.last_cmd_vel_time = rospy.Time.now()

    def _init_pubsub(self):
        self.odom_pub = rospy.Publisher('odom', Odometry)
        self.imu_pub = rospy.Publisher('imu', Imu)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel)
        self.color_sub = rospy.Subscriber('~set_color', ColorRGBA, self.set_color)
        self.color_sub = rospy.Subscriber('~set_back_led', Float32, self.set_back_led)

    def _init_params(self):
        self.connect_color_red = rospy.get_param('~connect_red',0)
        self.connect_color_blue = rospy.get_param('~connect_blue',0)
        self.connect_color_green = rospy.get_param('~connect_green',255)
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))


    def start(self):
        try:
            self.robot.connect()
        except:
            sys.exit(1)
        self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, False)
        self.robot.add_streaming_callback(self.parse_data)
        self.robot.set_rgb_led(self.connect_color_red,self.connect_color_green,self.connect_color_blue,0,False) #turn the ball green for connection
        self.robot.set_stablization(0,False)
        self.robot.start()

    def spin(self):
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if  (now - self.last_cmd_vel_time) > self.cmd_vel_timeout:
                self.cmd_heading = 0
                self.cmd_speed = 0
                self.robot.roll(int(self.cmd_speed),int(self.cmd_heading),0,False)

            r.sleep()
        
        
        
    def stop(self):    
        self.robot.disconnect()
        self.robot.join()

    def parse_data(self, data):
        now = rospy.Time.now()
        self.imu.header.stamp = now
        (self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w) = PyKDL.Rotation.RPY(data["IMU_ROLL_FILTERED"]/180.0*math.pi,
                                                                                                                              data["IMU_PITCH_FILTERED"]/180*math.pi,
                                                                                                                              data["IMU_YAW_FILTERED"]/180*math.pi).GetQuaternion()


        #TODO: Figure out units
        self.imu.linear_acceleration.x = data["ACCEL_X_FILTERED"]/4096
        self.imu.linear_acceleration.y = data["ACCEL_Y_FILTERED"]/4096
        self.imu.linear_acceleration.z = data["ACCEL_Z_FILTERED"]/4096
        self.imu.angular_velocity.x = data["GYRO_X_FILTERED"]
        self.imu.angular_velocity.y = data["GYRO_Y_FILTERED"]
        self.imu.angular_velocity.z = data["GYRO_Z_FILTERED"]

        self.imu_pub.publish(self.imu)
        #TODO: parse the EMF into something.. 

    def cmd_vel(self, msg):
        self.last_cmd_vel_time = rospy.Time.now()
        self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.x,msg.linear.y))*180/math.pi
        self.cmd_speed = math.sqrt(math.pow(msg.linear.x,2)+math.pow(msg.linear.y,2))
        self.robot.roll(int(self.cmd_speed),int(self.cmd_heading),1,False)

    def normalize_angle_positive(self, angle):
        return math.fmod(math.fmod(angle, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi);
    
    def set_color(self, msg):
        self.robot.set_rgb_led(int(msg.r*255),int(msg.g*255),int(msg.b*255),0,False)

    def set_back_led(self,msg):
        self.robot.set_back(msg.data,False)


if __name__ == '__main__':
    s = SpheroNode()
    while not rospy.is_shutdown():
        s.start()
        s.spin()
    s.stop()

