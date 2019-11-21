#!/usr/bin/env python3

import usb.core
import usb.util
import sys
from PIL import Image, ImageOps
import struct

dev = usb.core.find(idVendor=0x03eb, idProduct=0x2040)
if dev is None:
    print("Not found!")
    exit(1)

dev.set_configuration()
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]
ep = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_OUT)


im = Image.open(sys.argv[1])
im = im.convert("RGB")

data=struct.pack("<HHHH",0,0,im.width,im.height)
dev.ctrl_transfer(usb.util.CTRL_RECIPIENT_INTERFACE|usb.util.CTRL_TYPE_VENDOR|usb.util.CTRL_OUT,1,data_or_wLength=data)

data = []
for r,g,b in im.getdata():
    color=((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    data.extend(struct.pack(">H",color))

ep.write(data)
