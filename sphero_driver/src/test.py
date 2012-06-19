#!/usr/bin/python
import bluetooth
import struct
import time
import sphero_driver
sphero = sphero_driver.Sphero()
sphero.connect()
sphero.set_stablization(0,False)
sphero.start()
sleep(100)
sphero.join()
sphero.disconnect()




