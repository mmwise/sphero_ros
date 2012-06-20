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

import bluetooth
import sys
import struct
import time
import operator
import threading

#These are the message response code that can be return by Sphero.
MRSP = dict(
  ORBOTIX_RSP_CODE_OK = 0x00,           #Command succeeded
  ORBOTIX_RSP_CODE_EGEN = 0x01,         #General, non-specific error
  ORBOTIX_RSP_CODE_ECHKSUM = 0x02,      #Received checksum failure
  ORBOTIX_RSP_CODE_EFRAG = 0x03,        #Received command fragment
  ORBOTIX_RSP_CODE_EBAD_CMD = 0x04,     #Unknown command ID
  ORBOTIX_RSP_CODE_EUNSUPP = 0x05,      #Command currently unsupported
  ORBOTIX_RSP_CODE_EBAD_MSG = 0x06,     #Bad message format
  ORBOTIX_RSP_CODE_EPARAM = 0x07,       #Parameter value(s) invalid
  ORBOTIX_RSP_CODE_EEXEC = 0x08,        #Failed to execute command
  ORBOTIX_RSP_CODE_EBAD_DID = 0x09,     #Unknown device ID
  ORBOTIX_RSP_CODE_POWER_NOGOOD = 0x31, #Voltage too low for refash operation
  ORBOTIX_RSP_CODE_PAGE_ILLEGAL = 0x32, #Illegal page number provided
  ORBOTIX_RSP_CODE_FLASH_FAIL = 0x33,   #Page did not reprogram correctly
  ORBOTIX_RSP_CODE_MA_CORRUPT = 0x34,   #Main application corrupt
  ORBOTIX_RSP_CODE_MSG_TIMEOUT = 0x35)  #Msg state machine timed out

REQ = dict(
  WITH_RESPONSE =[0xff, 0xff],
  WITHOUT_RESPONSE =[0xff, 0xfe],
  CMD_PING = [0x00, 0x01],
  CMD_VERSION = [0x00, 0x02],
  CMD_SET_BT_NAME = [0x00, 0x10],
  CMD_GET_BT_NAME = [0x00, 0x11],
  CMD_SET_AUTO_RECONNECT = [0x00, 0x12],
  CMD_GET_AUTO_RECONNECT = [0x00, 0x13],
  CMD_GET_PWR_STATE = [0x00, 0x20],
  CMD_SET_PWR_NOTIFY = [0x00, 0x21],
  CMD_SLEEP = [0x00, 0x22],
  CMD_GOTO_BL = [0x00, 0x30],
  CMD_RUN_L1_DIAGS = [0x00, 0x40],
  CMD_RUN_L2_DIAGS = [0x00, 0x41],
  CMD_CLEAR_COUNTERS = [0x00, 0x42],
  CMD_ASSIGN_COUNTER = [0x00, 0x50],
  CMD_POLL_TIMES = [0x00, 0x51],
  CMD_SET_HEADING = [0x02, 0x01],
  CMD_SET_STABILIZ = [0x02, 0x02],
  CMD_SET_ROTATION_RATE = [0x02, 0x03],
  CMD_SET_APP_CONFIG_BLK = [0x02, 0x04],
  CMD_GET_APP_CONFIG_BLK = [0x02, 0x05],
  CMD_SET_DATA_STRM = [0x02, 0x11],
  CMD_CFG_COL_DET = [0x02, 0x12],
  CMD_SET_RGB_LED = [0x02, 0x20],
  CMD_SET_BACK_LED = [0x02, 0x21],
  CMD_GET_RGB_LED = [0x02, 0x22],
  CMD_ROLL = [0x02, 0x30],
  CMD_BOOST = [0x02, 0x31],
  CMD_SET_RAW_MOTORS = [0x02, 0x33],
  CMD_SET_MOTION_TO = [0x02, 0x34],
  CMD_GET_CONFIG_BLK = [0x02, 0x40],
  CMD_SET_DEVICE_MODE = [0x02, 0x42],
  CMD_SET_CFG_BLOCK = [0x02, 0x43],
  CMD_GET_DEVICE_MODE = [0x02, 0x44],
  CMD_RUN_MACRO = [0x02, 0x50],
  CMD_SAVE_TEMP_MACRO = [0x02, 0x51],
  CMD_SAVE_MACRO = [0x02, 0x52],
  CMD_DEL_MACRO = [0x02, 0x53],
  CMD_INIT_MACRO_EXECUTIVE = [0x02, 0x54],
  CMD_ABORT_MACRO = [0x02, 0x55],
  CMD_GET_MACRO_STATUS = [0x02, 0x56],
  CMD_SET_MACRO_STATUS = [0x02, 0x57])

STRM = dict(
  GYRO_H_FILTERED    = 0x00000001,
  GYRO_M_FILTERED    = 0x00000002,
  GYRO_L_FILTERED    = 0x00000004,
  LEFT_EMF_FILTERED  = 0x00000020,
  RIGHT_EMF_FILTERED = 0x00000040,
  MAG_Z_FILTERED     = 0x00000080,
  MAG_Y_FILTERED     = 0x00000100,
  MAG_X_FILTERED     = 0x00000200,
  GYRO_Z_FILTERED    = 0x00000400,
  GYRO_Y_FILTERED    = 0x00000800,
  GYRO_X_FILTERED    = 0x00001000,
  ACCEL_Z_FILTERED   = 0x00002000,
  ACCEL_Y_FILTERED   = 0x00004000,
  ACCEL_X_FILTERED   = 0x00008000,
  IMU_YAW_FILTERED   = 0x00010000,
  IMU_ROLL_FILTERED  = 0x00020000,
  IMU_PITCH_FILTERED = 0x00040000,
  LEFT_EMF_RAW       = 0x00200000,
  RIGHT_EMF_RAW      = 0x00400000,
  MAG_Z_RAW          = 0x00800000,
  MAG_Y_RAW          = 0x01000000,
  MAG_X_RAW          = 0x02000000,
  GYRO_Z_RAW         = 0x04000000, 
  GYRO_Y_RAW         = 0x08000000,
  GYRO_X_RAW         = 0x10000000,
  ACCEL_Z_RAW        = 0x20000000,
  ACCEL_Y_RAW        = 0x40000000,
  ACCEL_X_RAW        = 0x80000000)  

class BTError(Exception):
  pass

class BTInterface(object):

  def __init__(self, target_name = 'Sphero', port = 1):
      self.target_name = target_name
      self.port = port
      self.found_device = False
      self.tries = 0
      self.target_address = None
      self.sock = None

  def connect(self):
    sys.stdout.write("Searching for devices....")
    sys.stdout.flush()
    nearby_devices = bluetooth.discover_devices()

    while(self.tries < 5):
      if len(nearby_devices)>0:
        for bdaddr in nearby_devices:
          if bluetooth.lookup_name(bdaddr) is not None:
            print bluetooth.lookup_name(bdaddr)
            #look for a device name that starts with Sphero
            if bluetooth.lookup_name(bdaddr).startswith(self.target_name):
              self.found_device = True
              self.target_address = bdaddr
              break
      if self.found_device:      
        break
      self.tries = self.tries + 1     
      sys.stdout.write("....")
      sys.stdout.flush()

    if self.target_address is not None:
      sys.stdout.write("\nFound Sphero device with address: %s\n" %  (self.target_address))
      sys.stdout.flush()
    else:
      raise BTError("No Sphero devices found.")

    self.sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    self.sock.connect((bdaddr,self.port))
    sys.stdout.write("Paired with Sphero.\n")
    sys.stdout.flush()
    return True

  def send(self, data):
    self.sock.send(data)

  def recv(self, num_bytes):
    return self.sock.recv(num_bytes)

  def close(self):
    self.sock.close()

class Sphero(object, threading.Thread):

  def __init__(self, target_name = 'Sphero'):
    self.target_name = target_name
    self.bt = None
    self.is_connected = False
    self.stream_mask = None
    self.seq = 0
    self.raw_data_buf = []
    self._communication_lock = threading.Lock()
    self._stream_callback_queue = []

    
  def connect(self):
    self.bt = BTInterface(self.target_name)
    self.is_connected = self.bt.connect()
    time.sleep(0.1)
  
  def get_seq(self):
    self.seq = self.seq + 1
    if self.seq > 0xff:
      self.seq = 0
    return self.seq

  def pack_cmd(self, req ,cmd):
    return req + [self.get_seq()] + [len(cmd)+1] + cmd
    
  def data2hexstr(self, data):
    return ' '.join([ ("%02x"%ord(d)) for d in data])

  def create_mask_list(self, mask):
    #save the mask
    self.sample_mask = mask
    sorted_STRM = sorted(STRM.iteritems(), key=operator.itemgetter(1), reverse=True)
    #create a list containing the keys that are part of the mask
    self.mask_list = [key  for key, value in sorted_STRM if value & self.sample_mask]

  def add_streaming_callback(self, callback):
    self._stream_callback_queue.append(callback)

  def remove_streaming_callback(self, callback):
    if callback in self._stream_callback_queue:
      self._stream_callback_queue.remove(callback)

  def ping(self, response):
    """
    The Ping command is used to verify both a solid data link with the
    Client and that Sphero is awake and dispatching commands.

    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_PING'],[]), response)

  def get_version(self, response):
    """
    The Get Versioning command returns a whole slew of software and
    hardware information.

    :param response: request response back from Sphero.
    """
    self.send(REQ['CMD_VERSION'], response)

  def set_device_name(self, name, response):
    """
    This assigned name is held internally and produced as part of the
    Get Bluetooth Info service below. Names are clipped at 48
    characters in length to support UTF-8 sequences; you can send
    something longer but the extra will be discarded. This field
    defaults to the Bluetooth advertising name.

    :param name: 48 character name.
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_SET_BT_NAME'],[name]), response)

  def get_bt_name(self, response):
    """
    This returns the textual name (in ASCII) that the Bluetooth module
    advertises. It also returns the BTA Bluetooth Address or MAC ID
    for this device. Both values are returned in ASCII and have field
    widths of 16 characters, with unused trailing characters set to
    00h.

    :param response: request response back from Sphero.
    """
    self.send(REQ['CMD_GET_BT_NAME'], response)

  def set_auto_reconnect(self, enable, time, response):
    """
    This configures the control of the Bluetooth module in its attempt
    to automatically reconnect with the last iPhone device. This is a
    courtesy behavior since the iPhone Bluetooth stack doesn't initiate
    automatic reconnection on its own.  The two parameters are simple:
    flag = 00h to disable or 01h to enable, and time = the number of
    seconds after power-up in which to enable auto reconnect mode. For
    example, if time = 30 then the module will be attempt reconnecting 30
    seconds after waking up.

    :param enable: 00h to disable or 01h to enable auto reconnecting.
    :param time: the number of seconds after power-up in which to
    enable auto reconnect mode
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_SET_AUTO_RECONNECT'],[enable,time]), response)

  def get_auto_reconnect(self, response):
    """
    This returns the Bluetooth auto reconnect values as defined in the
    Set Auto Reconnect command.
    
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_GET_AUTO_RECONNECT'],[]), reponse)

  def get_power_state(self, reponse):
    """
    This returns the current power state and some additional
    parameters to the Client.

    :param response: request response back from Sphero.
    """
    self.send(REQ['CMD_GET_PWR_STATE'], response)

  def set_power_notify(self, response):
    """
    This enables Sphero to asynchronously notify the Client
    periodically with the power state or immediately when the power
    manager detects a state change. Timed notifications arrive every 10
    seconds until they're explicitly disabled or Sphero is unpaired. The
    flag is as you would expect, 00h to disable and 01h to enable.

    :param enable: 00h to disable and 01h to enable power notifications.
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_SET_PWR_NOTIFY'],[enable]), response)

  def go_to_sleep(time, macro, response):
    """
    This puts Sphero to sleep immediately with two parameters: the
    first is the number of seconds after which it will automatically
    re-awaken. If set to zero, this feature is disabled. If non-zero then
    the MACRO parameter allows an optional system macro to be run upon
    wakeup. If this is set to zero, no macro is executed.

    :param time: number of seconds wait before auto re-awake.
    :param macro: macro number to run when re-awakened. 
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_SLEEP'],[(time>>8), (time & 0xff), macro]), response)

  def run_l1_diags(self, response):
    """
    This is a developer-level command to help diagnose aberrant
    behavior. Most system counters, process flags, and system states
    are decoded into human readable ASCII. There are two responses to
    this command: a Simple Response followed by a large async message
    containing the results of the diagnostic tests.

    :param response: request response back from Sphero.
    """
    self.send(REQ['CMD_RUN_L1_DIAGS'], response)

  def run_l2_diags(self, response):
    """
    This is a developers-only command to help diagnose aberrant
    behavior. It is much less impactful than the Level 1 command as it
    doesn't interfere with the current operation it simply returns a
    current set of statistics to the client.

    :param response: request response back from Sphero.
    """
    self.send(REQ['CMD_RUN_L2_DIAGS'], reponse)

  def clear_counters(self, response):
    """
    This is a developers-only command to clear the various system
    counters described in the level 2 diag.

    :param response: request response back from Sphero.
    """
    self.send(REQ['CMD_CLEAR_COUNTERS'], response)

  def assign_counter_value(self, counter, response):
    """
    Sphero contains a 32-bit counter that increments every millisecond
    when it's not in the Idle state. It has no absolute meaning and is
    reset for various reasons. This command assigns the counter a
    specific value for subsequent sampling.
    
    :param counter: value to set the counter to.
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_ASSIGN_COUNTER'],[((counter>>24) & 0xff), ((counter>>16) & 0xff), ((counter>>8) & 0xff) ,(counter & 0xff)]), response)

  def poll_packet_times(self, time, response):
    """
    This command helps the Client application profile the transmission
    and processing latencies in Sphero so that a relative
    synchronization of timebases can be performed. It allows the
    Client to reconcile time stamped messages from Sphero to its own
    time stamped events.

    The scheme is as follows: the Client sends the command with the
    Client Tx time (T1) filled in. Upon receipt of the packet, the
    command processor in Sphero copies that time into the response
    packet and places the current value of the millisecond counter
    into the Sphero Rx time field (T2). Just before the transmit
    engine streams it into the Bluetooth module, the Sphero Tx time
    value (T3) is filled in. If the Client then records the time at
    which the response is received (T4) the relevant time segments can
    be computed from the four time stamps T1-T4:

    * The value offset represents the maximum-likelihood time offset
      of the Client clock to Sphero's system clock.  
        * offset = 1/2*[(T2 - T1) + (T3 - T4)]

    * The value delay represents the round-trip delay between the
      Client and Sphero:
        * delay = (T4 - T1) - (T3 - T2)

    :param time: client TX time.
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_POLL_TIME'],[((time>>24) & 0xff), ((time>>16) & 0xff), ((time>>8) & 0xff), (time & 0xff)]), response)

  def set_heading(self, heading, response):
    """
    This allows the client to adjust the orientation of Sphero by
    commanding a new reference heading in degrees, which ranges from 0
    to 359. You will see the ball respond immediately to this command
    if stabilization is enabled.

    :param heading: heading in degrees from 0 to 359 (motion will be
    shortest angular distance to heading command)
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_SET_HEADING'],[(heading>>8),(heading & 0xff)]), response)

  def set_stablization(self, enable, response):
    """
    This turns on or off the internal stabilization of Sphero, in
    which the IMU is used to match the ball's orientation to its
    various set points. The flag value is as you would expect, 00h for
    off and 01h for on.

    :param enable: 00h for off and 01h for on (on by default).
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_SET_STABILIZ'],[enable]), response)

  def set_rotation_rate(self, rate, response):
    """
    This allows you to control the rotation rate that Sphero will use
    to meet new heading commands. The commanded value is in units of
    0.784 degrees/sec. So, setting a value of c8h will set the
    rotation rate to 157 degrees/sec. A value of 255 jumps to the
    maximum and a value of 1 is the minimum.

    :param rate: rotation rate in units of 0.784degrees/sec (setting
    this value will not cause the device to move only set the rate it
    will move in other funcation calls).

    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_SET_ROTATION_RATE'],[rate]), response)

  def set_app_config_blk(self, app_data, response):
    """
    This allows you to write a 32 byte block of data from the
    configuration block that is set aside for exclusive use by
    applications.

    :param app_data: block set aside for application.
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_SET_APP_CONFIG_BLK'],[((app_data>>24) & 0xff), ((app_data>>16) & 0xff), ((app_data>>8) & 0xff), (app_data & 0xff)]), response)

  def get_app_config_blk(self, response):
    """
    This allows you to retrieve the application configuration block
    that is set aside for exclusive use by applications.

    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_GET_APP_CONFIG_BLK'], []), response)

  def set_data_strm(self, sample_div, sample_frames, sample_mask, pcnt, response):
    """
    Currently the control system runs at 400Hz and because it's pretty
    unlikely you will want to see data at that rate, N allows you to
    divide that down. sample_div = 2 yields data samples at 200Hz,
    sample_div = 10, 40Hz, etc. Every data sample consists of a
    "frame" made up of the individual sensor values as defined by the
    sample_mask. The sample_frames value defines how many frames to
    collect in memory before the packet is emitted. In this sense, it
    controls the latency of the data you receive. Increasing
    sample_div and the number of bits set in sample_mask drive the
    required throughput. You should experiment with different values
    of sample_div, sample_frames and sample_mask to see what works
    best for you.

    :param sample_div: divisor of the maximum sensor sampling rate.
    :param sample_frames: number of sample frames emitted per packet.
    :param sample_mask: bitwise selector of data sources to stream.
    :param pcnt: packet count (set to 0 for unlimited streaming).
    :param response: request response back from Sphero.
    """
    data = self.pack_cmd(REQ['CMD_SET_DATA_STRM'], \
           [(sample_div>>8), (sample_div & 0xff), (sample_frames>>8), (sample_frames & 0xff), ((sample_mask>>24) & 0xff), ((sample_mask>>16) & 0xff),((sample_mask>>8) & 0xff), (sample_mask & 0xff), pcnt])
    self.create_mask_list(sample_mask)
    self.stream_mask = sample_mask
    self.send(data, response)

  def set_filtered_data_strm(self, sample_div, sample_frames, pcnt, response):
    mask = 0
    for key,value in STRM.iteritems():
      if 'FILTERED' in key:
        mask = mask|value
    self.set_data_strm(sample_div, sample_frames, mask, pcnt, response)

  def set_raw_data_strm(self, sample_div, sample_frames, pcnt, response):
    mask = 0
    for key,value in STRM.iteritems():
      if 'RAW' in key:
        mask = mask|value
    self.set_data_strm(sample_div, sample_frames, mask, pcnt, response)

  def set_all_data_strm(self, sample_div, sample_frames, pcnt, response):
    mask = 0
    for value in STRM.itervalues():
        mask = mask|value
    self.set_data_strm(sample_div, sample_frames, mask, pcnt, response)
  
  def config_collision_detect(magnitude, ignore_time, response):
    """
    This command either enables or disables asynchronous message
    generation when a collision is detected. The magnitude parameter
    is the minimum normalized magnitude that the collision must equal
    in order to be "detected" and generate a message. A value of 00h
    disables this feature. The Ignore Time parameter disables
    collision detection for a period of time after the async message
    is generated, preventing message overload to the client. The value
    is in milliseconds.
    
    :param magnitude: normalize from 0-255 (units?).  
    :param ignore_time: time in milliseconds to disable collisions after a
    collision.
    """
    self.send(self.pack_cmd(REQ['CMD_CFG_COL_DET'],[magnitude, (ignore_time>>8), (ignore_time & 0xff)] ), response)

  def set_rgb_led(self, red, green, blue, save, response):
    """
    This allows you to set the RGB LED color. The composite value is
    stored as the "application LED color" and immediately driven to
    the LED (if not overridden by a macro or orbBasic operation). If
    FLAG is true, the value is also saved as the "user LED color"
    which persists across power cycles and is rendered in the gap
    between an application connecting and sending this command.

    :param red: red color value.
    :param green: green color value.
    :param blue: blue color value.
    :param save: 01h for save (color is saved as "user LED color").
    """
    self.send(self.pack_cmd(REQ['CMD_SET_RGB_LED'],[red, green, blue, save]), response)

  def set_back_led(self, brightness, response):
    """
    This allows you to control the brightness of the back LED. The
    value does not persist across power cycles.
    
    :param brightness: 0-255, off-on (the blue LED on hemisphere of the Sphero).
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_SET_BACK_LED'],[brightness]), response)

  def get_rgb_led(self, response):
    """
    This retrieves the "user LED color" which is stored in the config
    block (which may or may not be actively driven to the RGB LED).

    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_GET_RGB_LED'],[]), response)

  def roll(self, speed, heading, state, response):
    """
    This commands Sphero to roll along the provided vector. Both a
    speed and a heading are required; the latter is considered
    relative to the last calibrated direction. A state Boolean is also
    provided (on or off). The client convention for heading follows the 360
    degrees on a circle, relative to the ball: 0 is straight ahead, 90
    is to the right, 180 is back and 270 is to the left. The valid
    range is 0..359.

    :param speed: 0-255 value representing 0-max speed of the sphero.
    :param heading: heading in degrees from 0 to 359.
    :param state: 00h for off (braking) and 01h for on (driving). 
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_ROLL'],[speed, (heading>>8), (heading & 0xff), state]), response)

  def boost(self, time, heading, response):
    """
    This commands Sphero to meet the provided heading, disable
    stabilization and ramp the motors up to full-speed for a period of
    time. The Time parameter is the duration in tenths of a
    second. Setting it to zero enables constant boost until a Set
    Stabilization command is received.

    :param time: duration of boost in tenths of seconds.
    :param heading: the heading to travel while boosting.
    :param response: request response back from Sphero.
    """
    self.send(self.pack_cmd(REQ['CMD_BOOST'], [time, (heading>>8), (heading & 0xff)]), response)

  def set_raw_motor_values(l_mode, l_power, r_mode, r_power, response):
    """
    This allows you to take over one or both of the motor output
    values, instead of having the stabilization system control
    them. Each motor (left and right) requires a mode (see below) and
    a power value from 0- 255. This command will disable stabilization
    if both modes aren't "ignore" so you'll need to re-enable it via
    CID 02h once you're done.
    
    :param mode: 0x00 - off, 0x01 - forward, 0x02 - reverse, 0x03 - brake, 0x04 - ignored.
    :param power: 0-255 scalar value (units?).
    """
    self.send(self.pack_cmd(REQ['CMD_RAW_MOTORS'], [l_mode, l_power, r_mode, r_power]), response)

  def send(self, data, response):
    """
    Packets are sent from Client -> Sphero in the following byte format:
    -------------------------------------------------------
    | SOP1 | SOP2 | DID | CID | SEQ | DLEN | <data> | CHK |
    -------------------------------------------------------

    * SOP1 - start packet 1 - Always 0xff.  SOP2 - start packet 2 -
    * Set to 0xff when an acknowledgement is expected, 0xfe otherwise.
    * DID - Device ID 
    * CID - Command ID 
    * SEQ - Sequence Number - This client field is echoed in the 
      response for all synchronous commands (and ignored by Sphero when SOP2 = 0xfe) 
    * DLEN - Data
    * Length - Number of bytes through the end of the packet.  
    * <data>
    * CHK - Checksum - The modulo 256 sum of all the bytes from the
      DID through the end of the data payload, bit inverted (1's complement).
    """
    #compute the checksum
    #modulo 256 sum of data bit inverted
    checksum =~ sum(data) % 256
    #if expecting response back from the sphero
    if response:
      output = REQ['WITH_RESPONSE'] + data + [checksum]
    else:
      output = REQ['WITHOUT_RESPONSE'] + data + [checksum]
    #pack the msg
    msg = ''.join(struct.pack('B',x) for x in output)
    #send the msg
    with self._communication_lock.lock():
      self.bt.send(msg)

  def run(self):
    start = time.time()
    self.set_raw_data_strm(40, 1 , 0, False)
    self.strm_thread.start()
    red=0
    while(time.time() < start + 20.0):
      time.sleep(0.02)
      self.set_rgb_led(red,0,0,False)
      red = red + 0.02

    self.is_connected = False
    self.strm_thread.join()
    
    sys.exit(1)

  

  def recv(self, num_bytes):
    while self.is_connected:
      with self._communication_lock.lock(): 
        self.raw_data_buf += self.bt.recv(num_bytes)
      data = self.raw_data_buf
      while len(data)>5:
        if data[:2] == [chr(0xff),chr(0xff)]:
          # response packet
          data_length = ord(data[3])
          if data_length+4 <= len(data):
            print "data long enough to parse"
            data_packet = data[:(4+data_length)]
            data = data[(4+data_length):]
            print "Response packet", self.data2hexstr(data_packet)          
          else:
            break
        elif data[:2] == [chr(0xff), chr(0xfe)]:
          # streaming packet
          data_length = (ord(data[3])<<8)+ord(data[4])          
          if data_length+5 <= len(data):
            data_packet = data[:(5+data_length)]
            self.parse_data_strm(data_packet, data_length)
            data = data[(5+data_length):]
          else:
            # the remainder of the packet isn't long enough
            break
        else:
          raise RuntimeError("Bad SOF : " + self.data2hexstr(data))
      self.raw_data_buf=data

  def parse_data_strm(self, data, data_length):
    output={}
    for i in range((data_length-1)/2):
      unpack = struct.unpack_from('>h', ''.join(data[5+2*i:]))
      output[self.mask_list[i]] = unpack[0]
      print self.mask_list[i], unpack[0]
    print output

  def disconnect(self):
    self.is_connected = False
    self.bt.close()


