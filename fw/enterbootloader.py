#!/usr/bin/env python3

import usb.core
import usb.util

dev = usb.core.find(idVendor=0x03eb, idProduct=0x2040)
if dev is None:
    exit(0)
try:
    dev.ctrl_transfer(usb.util.CTRL_RECIPIENT_INTERFACE|usb.util.CTRL_TYPE_VENDOR|usb.util.CTRL_OUT,0)
except:
    pass
