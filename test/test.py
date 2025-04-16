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

#-------------------------------------------------------------------------------
# Default Configuration

DEVICE_NAME_UART = '/dev/ttyUSB0'
DEVICE_NAME_USB  = '/dev/ttyACM0'
BAUDRATE = 115200
TEST_RANGE_FROM = 1
TEST_RANGE_TO = 2050
MAX_RX_BUFFER = 2050
MIN_TIMEOUT_S = 0.05

#-------------------------------------------------------------------------------
def hex_dump(b):
    """ Display a range in a hex-dump format
    """
    text_line = ""
    for idx, val in enumerate(b):
        if (idx % 16 == 0) and (idx > 0):
            print(text_line)
            text_line = ""
        text_line += '{:02X} '.format(val)
    if(text_line):
        print(text_line)

#-------------------------------------------------------------------------------
def create_test_data(len):
    """ Create a template of test data
    """
    test_data = bytearray(len)
    for i in range(len):
        test_data[i] = (i + len) % 256
    return test_data

#-------------------------------------------------------------------------------
# Command line parser
parser = argparse.ArgumentParser(prog='test.py',
            description='Test RPico_CDC_UART firmware.')

parser.add_argument('send_device',
            help='Device that sends data (UART or USB)')
parser.add_argument('-a', '--uart', default=DEVICE_NAME_UART, dest='uart_device',
            help='UART device (default: %(default)s)')
parser.add_argument('-u', '--usb', default=DEVICE_NAME_USB, dest='usb_device',
            help='USB device (default: %(default)s)')
parser.add_argument('-b', '--baudrate', default=BAUDRATE, type=int, dest='baudrate',
            help='Baudrate (default: %(default)s)')
parser.add_argument('-f', '--from', default=TEST_RANGE_FROM, type=int, dest='from_value',
            help='Test range from (default: %(default)s)')
parser.add_argument('-t', '--to', default=TEST_RANGE_TO, type=int, dest='to_value',
            help='Test range to (default: %(default)s)')

args = parser.parse_args()

if (args.send_device != 'USB') and (args.send_device != 'UART'):
    parser.error("send_device can be USB or UART")

if (args.from_value < 1) or (args.to_value > MAX_RX_BUFFER) or (args.to_value < args.from_value):
    parser.error(f"FROM_VALUE and TO_VALUE must be between {TEST_RANGE_FROM} and {TEST_RANGE_TO}")

DEVICE_NAME_UART = args.uart_device
DEVICE_NAME_USB  = args.usb_device
BAUDRATE = args.baudrate
TEST_RANGE_FROM = args.from_value
TEST_RANGE_TO = args.to_value
SEND_DEVICE = args.send_device

print("UART device:", DEVICE_NAME_UART)
print("USB device:", DEVICE_NAME_USB)
print("Baudrate:", BAUDRATE)
print("From:", TEST_RANGE_FROM)
print("To:", TEST_RANGE_TO)
print("Send device:", SEND_DEVICE)

DEVICE_NAME_TX = DEVICE_NAME_UART
DEVICE_NAME_RX = DEVICE_NAME_USB
TEST_PREFIX = "UART->USB"

if SEND_DEVICE == 'USB':
    DEVICE_NAME_TX = DEVICE_NAME_USB
    DEVICE_NAME_RX = DEVICE_NAME_UART
    TEST_PREFIX = "USB->UART"

#-------------------------------------------------------------------------------
# Test Code

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

    # Intentionally inject an error (to test if we detect it)
    #if i == 30:
    #    test_data[10] = 0

    if (len(rx_bytes) == i) and (rx_bytes == test_data):
        #sys.stdout.write('{} {:5d} ({:02X}): Ok \r'.format(TEST_PREFIX, i, i))
        #sys.stdout.flush()
        print('{} {:5d} ({:02X}): Ok'.format(TEST_PREFIX, i, i))
    else:
        print('{} {:5d} ({:02X}): Failed'.format(TEST_PREFIX, i, i))
        print('Sent {:d} bytes:'.format(len(test_data)))
        hex_dump(test_data)
        print('Received {:d} bytes:'.format(len(rx_bytes)))
        hex_dump(rx_bytes)
        break

ser_tx.close()
ser_rx.close()
print("End of test")