#!/bin/bash
WORKSPACE="/home/kabeibl/Repositories/CliBox"
LINK_TARGET="CliBox"

esptool.py --port /dev/ttyUSB0 write_flash -fm dio 0x00000 $WORKSPACE/$LINK_TARGET-0x00000.bin
esptool.py --port /dev/ttyUSB0 write_flash -fm dio 0x10000 $WORKSPACE/$LINK_TARGET-0x10000.bin