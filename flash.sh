#!/bin/bash

esptool.py --port /dev/ttyUSB0 write_flash -fm dio 0x00000 /home/kabeibl/project-posix/nodemcu/main-0x00000.bin
esptool.py --port /dev/ttyUSB0 write_flash -fm dio 0x10000 /home/kabeibl/project-posix/nodemcu/main-0x10000.bin