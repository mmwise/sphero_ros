#!/usr/bin/python
import bluetooth
import struct
import time
import sphero_driver
import sys
sphero = sphero_driver.Sphero()
sphero.connect()
sphero.set_raw_data_strm(40, 1 , 0, False)

sphero.start()
time.sleep(2)
print "SETTING RED LED"
sphero.set_rgb_led(255,0,0,0,False)
time.sleep(1)
print "SETTING GREEN LED"
sphero.set_rgb_led(0,255,0,0,False)
time.sleep(1)
print "SETTING BLUE LED"
sphero.set_rgb_led(0,0,255,0,False)
time.sleep(3)
sphero.join()
sphero.disconnect()
sys.exit(1)



