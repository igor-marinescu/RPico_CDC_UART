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

#-------------------------------------------------------------------------------
# Configuration
#-------------------------------------------------------------------------------
DEVICE_NAME_UART = '/dev/ttyUSB0'
DEVICE_NAME_USB  = '/dev/ttyACM0'
BAUDRATE = 115200
TEST_RANGE_FROM = 1
TEST_RANGE_TO = 2050
MAX_RX_BUFFER = 2050
MIN_TIMEOUT_S = 0.05

DEVICE_NAME_TX = DEVICE_NAME_USB
DEVICE_NAME_RX = DEVICE_NAME_UART
TEST_PREFIX = "usb->uart"

#-------------------------------------------------------------------------------
# Display a range in a hex-dump format
def hex_dump(b):
    text_line = ""
    for idx, val in enumerate(b):
        if (idx % 16 == 0) and (idx > 0):
            print(text_line)
            text_line = ""
        text_line += '{:02X} '.format(val)
    if(text_line):
        print(text_line)

#-------------------------------------------------------------------------------
# Create a template of test data
def create_test_data(len):
    test_data = bytearray(len)
    for i in range(len):
        test_data[i] = (i + len) % 256
    return test_data

#-------------------------------------------------------------------------------
# Command arguments: 
# <program> [device_tx] [test_range_from] [test_range_to] [baudrate]
#  argv[0]    argv[1]        argv[2]         argv[3]       argv[4]

# argv[1] = device_tx
if len(sys.argv) > 1:
    if sys.argv[1] == "uart":
        DEVICE_NAME_TX = DEVICE_NAME_UART
        DEVICE_NAME_RX = DEVICE_NAME_USB
        TEST_PREFIX = "uart->usb"

# argv[2] = test_range_from
if len(sys.argv) > 2:
    TEST_RANGE_FROM = int(sys.argv[2])

# argv[3] = test_range_from
if len(sys.argv) > 3:
    TEST_RANGE_TO = int(sys.argv[3])

# argv[4] = baudrate
if len(sys.argv) > 4:
    BAUDRATE = int(sys.argv[4])

#-------------------------------------------------------------------------------
ser_tx = serial.Serial(DEVICE_NAME_TX, BAUDRATE)
ser_rx = serial.Serial(DEVICE_NAME_RX, BAUDRATE, timeout=0)
print(ser_tx.name, "-->", ser_rx.name)

for i in range(TEST_RANGE_FROM, TEST_RANGE_TO):

    # Calculate time required to send data:
    #   1 bit = 1/BAUDRATE
    #   1 byte (10bits = start + 8 data + stop) = 10/BAUDRATE
    #   i bytes = (10/BAUDRATE) * i
    # Add a 1.5 factor as reserve and limit the minimal time to 50ms

    send_time = (10.0/BAUDRATE) * i * 1.5
    if send_time < MIN_TIMEOUT_S:
        send_time = MIN_TIMEOUT_S

    test_data = create_test_data(i)

    ser_tx.write(test_data)
    ser_tx.flush()
    time.sleep(send_time)
    rx_bytes = ser_rx.read(MAX_RX_BUFFER)

    #if i == 30:
    #    test_data[10] = 0

    if (len(rx_bytes) == i) and (rx_bytes == test_data):
        print(TEST_PREFIX, i, '({:02X}): Ok,'.format(i), "Timeout:", send_time)
    else:
        print(TEST_PREFIX, i, '({:02X}): Failed,'.format(i), "Timeout:", send_time)
        print("Sent:", len(test_data), "bytes:")
        hex_dump(test_data)
        print("Received:", len(rx_bytes), "bytes:")
        hex_dump(rx_bytes)
        break

ser_tx.close()
ser_rx.close()
