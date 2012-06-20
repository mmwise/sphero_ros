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

from sphero_driver import Sphero

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class SpheroNode(object):

    def __init__(self, default_update_rate=50.0):
        rospy.init_node('sphero')
        self.default_update_rate = default_update_rate
        self.sampling_divisor = int(400/default_update_rate)

        self._init_pubsub()
        self.robot = Sphero()
        
    def _init_pubsub(self):
        self.odom_pub = rospy.Publisher('odom', Odometry)
        self.odom_pub = rospy.Publisher('imu', Imu)

    def _initi_params(self):
        pass

    def start(self):
        try:
            self.robot.connect()
        except:
            pass
        self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, False)
        self.robot.add_streaming_callback(self.parse_data)
        self.robot.start()
        
    def stop(self):    
        self.robot.disconnect()
        self.robot.join()

    def parse_data(self, data):
        print "Got data"
        print data

if __name__ == '__main__':
    s = SpheroNode()
    while not rospy.is_shutdown():
        s.start()
        s.spin()
    s.stop()

