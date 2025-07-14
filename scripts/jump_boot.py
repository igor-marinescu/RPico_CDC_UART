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
BAUDRATE = 1200

#-------------------------------------------------------------------------------
# Command line parser

parser = argparse.ArgumentParser(prog='jump_boot.py',
            description='RPico Jump to Bott')

parser.add_argument('-u', '--usb', default=DEVICE_NAME_USB, dest='usb_device',
            help='USB device (default: %(default)s)')
parser.add_argument('-b', '--baudrate', default=BAUDRATE, type=int, dest='baudrate',
            help='Baudrate (default: %(default)s)')

args = parser.parse_args()

DEVICE_NAME_USB  = args.usb_device
BAUDRATE = args.baudrate

# Jump to boot: open USB device, send a dummy byte and close it
ser_tx = serial.Serial(DEVICE_NAME_USB, BAUDRATE)
ser_tx.write(bytes(1))
ser_tx.flush()
time.sleep(1.0)
ser_tx.close()
