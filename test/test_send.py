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

#-------------------------------------------------------------------------------
# Default Configuration
BAUDRATE = 115200
MIN_TIMEOUT_S = 0.05
BIT_CNT = 9
MAX_RX_BUFFER = 256
DATA_HBLB = True

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
    if BIT_CNT == 9:
        test_data = bytearray(len * 2)
        for i in range(len):
            if DATA_HBLB:
                test_data[(i * 2) + 0] = i % 2
                test_data[(i * 2) + 1] = (i + len) % 256
            else:
                test_data[(i * 2) + 0] = (i + len) % 256
                test_data[(i * 2) + 1] = i % 2
        return test_data
    
    test_data = bytearray(len)
    for i in range(len):
        test_data[i] = (i + len) % 256
    return test_data

#-------------------------------------------------------------------------------
def  calculate_send_time(bit_cnt, frame_cnt):
    """ Calculate time required to send data:
        1 bit = 1/BAUDRATE
        1 frame = 1 start + bit_cnt + 1 stop = (2 + bit_cnt)/BAUDRATE
        frame_cnt frames = ((2 + bit_cnt)/BAUDRATE) * frame_cnt
        Add a 1.5 factor as reserve and limit the minimal time to 50ms
    """
    send_time = ((2.0 + bit_cnt)/BAUDRATE) * (frame_cnt) * 1.5
    if send_time < MIN_TIMEOUT_S:
        send_time = MIN_TIMEOUT_S
    return send_time

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

args = parser.parse_args()

# Check if send and receive devices exists
if not os.path.exists(args.tx_dev):
    parser.error(f"Send device (tx_dev) not found: {args.tx_dev} ")
if not os.path.exists(args.rx_dev):
    parser.error(f"Receive device (rx_dev) not found: {args.rx_dev} ")

BAUDRATE = args.baudrate
TX_DEV = args.tx_dev
RX_DEV = args.rx_dev

print("Tx Device:", TX_DEV)
print("Rx device:", RX_DEV)
print("Baudrate:", BAUDRATE)

TEST_PREFIX = TX_DEV + "->" + RX_DEV

#-------------------------------------------------------------------------------
# Test Code

ser_tx = serial.Serial(TX_DEV, BAUDRATE)
ser_rx = serial.Serial(RX_DEV, BAUDRATE, timeout=0)

# Give time (~10ms) to pico to configure the UART interface
time.sleep(0.01)

frame_cnt = 10
test_data = create_test_data(10)
print('Send {:d} bytes:'.format(len(test_data)))
hex_dump(test_data)

ser_tx.write(test_data)
ser_tx.flush()
time.sleep(calculate_send_time(BIT_CNT, frame_cnt))
rx_bytes = ser_rx.read(MAX_RX_BUFFER)

print('Received {:d} bytes:'.format(len(rx_bytes)))
hex_dump(rx_bytes)

ser_tx.close()
ser_rx.close()
print("End of test")