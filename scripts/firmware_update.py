#*******************************************************************************
# This file is part of the RPico_CDC_UART distribution.
# Copyright (c) 2025 Igor Marinescu (igor.marinescu@gmail.com).
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, version 3.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#******************************************************************************

import serial
import time
import sys
import argparse
import os
import shutil

#-------------------------------------------------------------------------------
# Default Configuration

DEVICE_NAME_USB  = "/dev/ttyACM0"
MOUNT_POINT = "/media/igor/RPI-RP2/"
BAUDRATE = 1200


#-------------------------------------------------------------------------------
# Command line parser

parser = argparse.ArgumentParser(prog='firmware_update.py',
            description='RPico Firmware Update')

parser.add_argument('firmware',
            help='Firmware to update') 
parser.add_argument('-u', '--usb', default=DEVICE_NAME_USB, dest='usb_device',
            help='USB device (default: %(default)s)')
parser.add_argument('-b', '--baudrate', default=BAUDRATE, type=int, dest='baudrate',
            help='Baudrate (default: %(default)s)')
parser.add_argument('-m', '--mount_point', default=MOUNT_POINT, dest='mount_point',
            help='Mount point (default: %(default)s)')

args = parser.parse_args()

DEVICE_NAME_USB  = args.usb_device
BAUDRATE = args.baudrate
MOUNT_POINT = args.mount_point
FIRMWARE = args.firmware

#print("USB device:", DEVICE_NAME_USB)
#print("Baudrate:", BAUDRATE)
#print("Mount point:", MOUNT_POINT)


# Check if firmware exists
if not os.path.isfile(FIRMWARE):
    print("Firmware does not exist", FIRMWARE)
    exit()

# Check if device not yet mounted (not yet in boot)
print("Check if device mounted", MOUNT_POINT)
if not os.path.exists(MOUNT_POINT):

    print("Device not yet mounted", MOUNT_POINT)
    print("Trying to jump to boot using", DEVICE_NAME_USB)

    # Check if USB device exists
    if not os.path.exists(DEVICE_NAME_USB):
        print("Cannot jump to boot")
        print("USB device not found", DEVICE_NAME_USB)
        exit()

    # Jump to boot: open USB device, send a dummy byte and close it
    ser_tx = serial.Serial(DEVICE_NAME_USB, BAUDRATE)
    ser_tx.write(bytes(1))
    ser_tx.flush()
    time.sleep(1.0)
    ser_tx.close()
    
    # Wait until device is mounted (but no longer as 10 sec)
    for i in range(9):
        if os.path.exists(MOUNT_POINT):
            break
        time.sleep(1.0)

if not os.path.exists(MOUNT_POINT):    
    print("Cannot do a firmware update")
    print("Device not mounted", MOUNT_POINT)
    exit()

print("Device mounted", MOUNT_POINT)
print("Copy", FIRMWARE, "-->", MOUNT_POINT)
shutil.copy(FIRMWARE, MOUNT_POINT)

print("Done")
