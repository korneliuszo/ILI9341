#!/usr/bin/env python3

import usb.core
import usb.util
import sys
import struct

dev = usb.core.find(idVendor=0x03eb, idProduct=0x2040)
if dev is None:
    print("Not found!")
    exit(1)

data=struct.pack("BBBB",int(sys.argv[1]),int(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4]))
dev.ctrl_transfer(usb.util.CTRL_RECIPIENT_INTERFACE|usb.util.CTRL_TYPE_VENDOR|usb.util.CTRL_OUT,3,data_or_wLength=data)
