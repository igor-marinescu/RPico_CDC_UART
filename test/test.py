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
MIN_TIMEOUT_S = 0.01
BIT_CNT = 9
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
def create_test_data(elements_cnt):
    """ Create a template of test data
    """
    if BIT_CNT > 8:
        test_data = bytearray(elements_cnt * 2)
        for i in range(elements_cnt):
            if DATA_HBLB:
                test_data[(i * 2) + 0] = i % 2
                test_data[(i * 2) + 1] = (i + elements_cnt) % 256
            else:
                test_data[(i * 2) + 0] = (i + elements_cnt) % 256
                test_data[(i * 2) + 1] = i % 2
        return test_data
    
    test_data = bytearray(elements_cnt)
    for i in range(elements_cnt):
        test_data[i] = (i + elements_cnt) % 256
    return test_data

#-------------------------------------------------------------------------------
def  calculate_send_time(bit_cnt, elements_cnt):
    """ Calculate time required to send data:
        1 bit = 1/BAUDRATE
        1 frame = 1 start + bit_cnt + 1 stop = (2 + bit_cnt)/BAUDRATE
        elements_cnt frames = ((2 + bit_cnt)/BAUDRATE) * elements_cnt
        Add a 1.5 factor as reserve and limit the minimal time to 50ms
    """
    send_time = ((2.0 + bit_cnt)/BAUDRATE) * (elements_cnt) * 1.1
    if send_time < MIN_TIMEOUT_S:
        send_time = MIN_TIMEOUT_S
    return send_time

#-------------------------------------------------------------------------------
def dev_send_receive(test_text, dev_tx, dev_rx, elements_cnt, no_check):
    """ Create a buffer of test-data of size bytes_count and send it on dev_tx.
        Receive data using dev_rx, if flag no_check is 0, check if the data
        sent is the same as data received."""

    test_data = create_test_data(elements_cnt)

    dev_tx.write(test_data)
    dev_tx.flush()
    time.sleep(calculate_send_time(BIT_CNT, elements_cnt))

    # Calculate expected count of data to be received (based on bit count)
    # Add a reserve of 4 bytes for the buffer size
    max_rx_bytes = (elements_cnt + 4)
    rx_bytes_expect = elements_cnt
    if BIT_CNT > 8:
        max_rx_bytes = max_rx_bytes * 2
        rx_bytes_expect = rx_bytes_expect * 2

    rx_bytes = dev_rx.read(max_rx_bytes)

    # Intentionally inject an error (to test if we detect it)
    #if elements_cnt == 30:
    #    test_data[10] = 0

    if no_check == 0:

        # Less bytes received? Sleep more and try to receive again
        if len(rx_bytes) < rx_bytes_expect:
            time.sleep(calculate_send_time(BIT_CNT, elements_cnt))
            rx_bytes2 = dev_rx.read(max_rx_bytes)
            if len(rx_bytes2) > 0:
                #print('More bytes received: {:d} + {:d}'.format(len(rx_bytes), len(rx_bytes2)))
                rx_bytearray = bytearray(rx_bytes)
                rx_bytearray.extend(rx_bytes2)
                rx_bytes = bytes(rx_bytearray)

        if (len(rx_bytes) != rx_bytes_expect) or (rx_bytes != test_data):

            print(test_text, '<-- Failed')
            print('Sent {:d} bytes:'.format(len(test_data)))
            hex_dump(test_data)
            print('Received {:d} bytes:'.format(len(rx_bytes)))
            hex_dump(rx_bytes)
            return False

    #print('{} {:5d}: Ok'.format(test_txt_prefix, elements_cnt))
    return True

#-------------------------------------------------------------------------------
def generate_progress_bar(from_val, to_val, current_val):
    """ Generate a progress bar """
    prc = int(((current_val - from_val) * 100) / (to_val - from_val));
    idx = int(((current_val - from_val) * 40) / (to_val - from_val));
    txt = '[' + '#'*idx + '_'*(40 - idx) + ']'
    return ' {:3d}% {}'.format(prc, txt);

#-------------------------------------------------------------------------------
# Command line parser
parser = argparse.ArgumentParser(prog='test.py',
            description='Test RPico_CDC_UART firmware.')

parser.add_argument('dev1',
            help='Serial Device 1 (Ex: /dev/ttyUSB0 for USB-UART converter, /dev/ttyACM0 for TinyUSB)')
parser.add_argument('dev2',
            help='Serial Device 2 (Ex: /dev/ttyUSB0 for USB-UART converter, /dev/ttyACM0 for TinyUSB)')
parser.add_argument('-b', '--baudrate', default=BAUDRATE, type=int, dest='baudrate',
            help='Baudrate (default: %(default)s)')
parser.add_argument('-f', '--from', default=TEST_RANGE_FROM, type=int, dest='from_value',
            help='Test range from (default: %(default)s)')
parser.add_argument('-t', '--to', default=TEST_RANGE_TO, type=int, dest='to_value',
            help='Test range to (default: %(default)s)')
parser.add_argument('-n', '--no-check', default=0, type=int, dest='no_check',
            help='Do not check the result (default: %(default)s)')
parser.add_argument('-m', '--test-mode', default=0, type=int, dest='test_mode',
            help='Test mode: 0=D1->D2, 1=D1/D2, 2=rand(D1/D2), 3=rand(pack_len), 4=rand(D1/D2,pack_len) (default: %(default)s)')

args = parser.parse_args()

# Check if serial devices exist
if not os.path.exists(args.dev1):
    parser.error(f"Device 1 (dev1) not found: {args.dev1} ")
if not os.path.exists(args.dev2):
    parser.error(f"Device 2 (dev2) not found: {args.dev2} ")

if (args.from_value < 1) or (args.to_value > TEST_RANGE_TO) or (args.to_value < args.from_value):
    parser.error(f"FROM_VALUE and TO_VALUE must be between {TEST_RANGE_FROM} and {TEST_RANGE_TO}")

BAUDRATE = args.baudrate
TEST_RANGE_FROM = args.from_value
TEST_RANGE_TO = args.to_value
dev1_name = args.dev1
dev2_name = args.dev2

print("Device 1:", dev1_name)
print("Device 2:", dev2_name)
print("Baudrate:", BAUDRATE)
print("From:", TEST_RANGE_FROM, "To:", TEST_RANGE_TO)
print("No check:", args.no_check)
print("Test Mode:", args.test_mode)

#-------------------------------------------------------------------------------
# Test Code

dev1 = serial.Serial(dev1_name, BAUDRATE, timeout=0)
dev2 = serial.Serial(dev2_name, BAUDRATE, timeout=0)

# Give time (~10ms) to pico to configure the UART interface
time.sleep(0.01)

test_passed = True
for i in range(TEST_RANGE_FROM, TEST_RANGE_TO + 1):

    # Default dev1->dev2
    dev_tx, dev_rx = dev1, dev2
    dev_tx_name, dev_rx_name = dev1_name, dev2_name
    elements_count = i

    # Test Mode 1: dev1->dev2, dev2->dev1, dev1->dev2, dev2->dev1, ...
    if args.test_mode == 1:
        if i % 2:
            dev_tx, dev_rx = dev2, dev1
            dev_tx_name, dev_rx_name = dev2_name, dev1_name

    # Test Mode 2: random(dev1, dev2)
    elif args.test_mode == 2:
        if random.randint(1, 10) > 5:
            dev_tx, dev_rx = dev2, dev1
            dev_tx_name, dev_rx_name = dev2_name, dev1_name

    # Test Mode 3: random(packet_len)
    elif args.test_mode == 3:
        elements_count = random.randrange(1, TEST_RANGE_TO)

    # Test Mode 4: random(dev1, dev2) & random(packet_len)
    elif args.test_mode == 4:
        elements_count = random.randrange(1, TEST_RANGE_TO)
        if random.randint(1, 10) > 5:
            dev_tx, dev_rx = dev2, dev1
            dev_tx_name, dev_rx_name = dev2_name, dev1_name

    # Display progress bar
    progress = generate_progress_bar(TEST_RANGE_FROM, TEST_RANGE_TO, i)
    txt_test = '{} {:5d} {}->{} (pack.size: {})'.format(progress, i, dev_tx_name, dev_rx_name, elements_count)
    print(txt_test, sep=' ', end='\r', flush=True)

    # Execute test
    if not dev_send_receive(txt_test, dev_tx, dev_rx, elements_count, args.no_check):
        test_passed = False
        break

dev1.close()
dev2.close()

print()
print("End of test, Result:", test_passed)

if not test_passed:
    sys.exit(1)
sys.exit()
