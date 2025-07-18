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
import os.path
import random

#-------------------------------------------------------------------------------
# Default Configuration
BAUDRATE = 115200
TEST_RANGE_FROM = 1
TEST_RANGE_TO = 2050
MAX_RX_BUFFER = 2050
MIN_TIMEOUT_S = 0.01

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
def create_test_data(start_idx, count):
    """ Create a template of test data
    """
    test_data = bytearray(count)
    for i in range(count):
        test_data[i] = (start_idx + i) % 256
    return test_data

#-------------------------------------------------------------------------------
def dev_send_receive(test_txt_prefix, dev_tx, dev_rx, start_idx, bytes_count, no_check):
    """ Create a buffer of test-data of size bytes_count and send it on dev_tx.
        Receive data using dev_rx, if flag no_check is 0, check if the data
        sent is the same as data received."""

    # Calculate time required to send data:
    #   1 bit = 1/BAUDRATE
    #   1 byte (10bits = start + 8 data + stop) = 10/BAUDRATE
    #   [bytes_count] bytes = (10/BAUDRATE) * bytes_count
    # Add a 1.5 factor as reserve and limit the minimal time to 50ms
    send_time = (10.0/BAUDRATE) * bytes_count * 1.5
    if send_time < MIN_TIMEOUT_S:
        send_time = MIN_TIMEOUT_S

    test_data = create_test_data(start_idx, bytes_count)

    dev_tx.write(test_data)
    dev_tx.flush()
    time.sleep(send_time)
    rx_bytes = dev_rx.read(MAX_RX_BUFFER)

    # Intentionally inject an error (to test if we detect it)
    #if bytes_count == 30:
    #    test_data[10] = 0

    if no_check == 0:

        if (len(rx_bytes) == bytes_count) and (rx_bytes == test_data):
            #sys.stdout.write('{} {:5d} ({:02X}): Ok \r'.format(test_txt_prefix, start_idx, start_idx))
            #sys.stdout.flush()
            print('{} {:5d} ({:02X}): Ok'.format(test_txt_prefix, start_idx, start_idx))
        else:
            print('{} {:5d} ({:02X}): Failed'.format(test_txt_prefix, start_idx, start_idx))
            print('Sent {:d} bytes:'.format(len(test_data)))
            hex_dump(test_data)
            print('Received {:d} bytes:'.format(len(rx_bytes)))
            hex_dump(rx_bytes)
            return False
    else:
            print('{} {:5d} ({:02X}): (no check)'.format(test_txt_prefix, bytes_count, bytes_count))
    return True

#-------------------------------------------------------------------------------
# Command line parser
parser = argparse.ArgumentParser(prog='test.py',
            description='Test RPico_CDC_UART firmware.')

parser.add_argument('tx_dev',
            help='Device that sends data, ex: /dev/ttyUSB0 (USB-UART converter), /dev/ttyACM0 (TinyUSB)')
parser.add_argument('rx_dev',
            help='Device that receives data, ex: /dev/ttyUSB0 (USB-UART converter), /dev/ttyACM0 (TinyUSB)')
parser.add_argument('-b', '--baudrate', default=BAUDRATE, type=int, dest='baudrate',
            help='Baudrate (default: %(default)s)')
parser.add_argument('-f', '--from', default=TEST_RANGE_FROM, type=int, dest='from_value',
            help='Test range from (default: %(default)s)')
parser.add_argument('-t', '--to', default=TEST_RANGE_TO, type=int, dest='to_value',
            help='Test range to (default: %(default)s)')
parser.add_argument('-n', '--no-check', default=0, type=int, dest='no_check',
            help='Do not check the result (default: %(default)s)')
parser.add_argument('-m', '--test-mode', default=0, type=int, dest='test_mode',
            help='Test mode: 0=normal, 1=tx,rx,tx,rx.., 2=random(tx/rx), 3=random(pack_len) (default: %(default)s)')

args = parser.parse_args()

# Check if send and receive devices exists
if not os.path.exists(args.tx_dev):
    parser.error(f"Send device (tx_dev) not found: {args.tx_dev} ")
if not os.path.exists(args.rx_dev):
    parser.error(f"Receive device (rx_dev) not found: {args.rx_dev} ")

if (args.from_value < 1) or (args.to_value > MAX_RX_BUFFER) or (args.to_value < args.from_value):
    parser.error(f"FROM_VALUE and TO_VALUE must be between {TEST_RANGE_FROM} and {TEST_RANGE_TO}")

BAUDRATE = args.baudrate
TEST_RANGE_FROM = args.from_value
TEST_RANGE_TO = args.to_value
TX_DEV = args.tx_dev
RX_DEV = args.rx_dev

print("Tx Device:", TX_DEV)
print("Rx device:", RX_DEV)
print("Baudrate:", BAUDRATE)
print("From:", TEST_RANGE_FROM)
print("To:", TEST_RANGE_TO)

#-------------------------------------------------------------------------------
ser_tx = serial.Serial(TX_DEV, BAUDRATE, timeout=0)
ser_rx = serial.Serial(RX_DEV, BAUDRATE, timeout=0)

# Give time (~10ms) to pico to configure the UART interface
time.sleep(0.01)

# Prepare random generator
random.seed()

for i in range(TEST_RANGE_FROM, TEST_RANGE_TO):

    # Default ser_tx->ser_rx
    d_tx, d_rx = ser_tx, ser_rx
    test_txt_prefix = TX_DEV + "->" + RX_DEV
    start_idx = i
    bytes_count = i

    # Test Mode 1: Send: ser_tx, ser_rx, ser_tx, ser_rx, ser_tx, ser_rx
    if args.test_mode == 1:
        if i % 2:
            d_tx, d_rx = ser_rx, ser_tx
            test_txt_prefix = RX_DEV + "->" + TX_DEV

    # Test Mode 2: random(ser_tx, ser_rx)
    elif args.test_mode == 2:
        if random.randint(1, 10) > 5:
            d_tx, d_rx = ser_rx, ser_tx
            test_txt_prefix = RX_DEV + "->" + TX_DEV

    # Test Mode 3: random(pack_len)
    elif args.test_mode == 3:
        bytes_count = random.randrange(1, TEST_RANGE_TO)

    # Test Mode 4: random(ser_tx, ser_rx) & random(ser_tx, ser_rx)
    elif args.test_mode == 4:
        bytes_count = random.randrange(1, TEST_RANGE_TO)
        if random.randint(1, 10) > 5:
            d_tx, d_rx = ser_rx, ser_tx
            test_txt_prefix = RX_DEV + "->" + TX_DEV

    if not dev_send_receive(test_txt_prefix, d_tx, d_rx, start_idx, bytes_count, args.no_check):
        break

ser_tx.close()
ser_rx.close()
print("End of test")