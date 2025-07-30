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
MIN_TIMEOUT_S = 0.01
DEF_BAUD_RATE = 115200
MIN_RANGE_FROM = 1
MAX_RANGE_TO = 2050

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
def generate_progress_bar(from_val, to_val, current_val):
    """ Generate a progress bar 
    """
    prc = int(((current_val - from_val) * 100) / (to_val - from_val));
    idx = int(((current_val - from_val) * 40) / (to_val - from_val));
    txt = '[' + '#'*idx + '_'*(40 - idx) + ']'
    return ' {:3d}% {}'.format(prc, txt);

#-------------------------------------------------------------------------------
class SerTestConfig:
    """ Serial-Test Configuration """

    def __init__(self):
        """ Serial-Test default configuration
        """
        self.dev1_name = '/dev/ttyACM0'
        self.dev2_name = '/dev/ttyACM1'
        self.baud_rate = DEF_BAUD_RATE
        self.range_from = MIN_RANGE_FROM
        self.range_to = MAX_RANGE_TO
        self.bit_cnt = 9
        self.data_hblb = True
        self.no_check = False
        self.test_mode = 0

    def print(self):
        """ Print Serial-Test configuration
        """
        print("Device 1:", self.dev1_name)
        print("Device 2:", self.dev2_name)
        print("Baud-Rate:", self.baud_rate)
        print("DataBitCnt:", self.bit_cnt)
        print("Big-Endian:", self.data_hblb)
        print("From:", self.range_from, "To:", self.range_to)
        print("No check:", self.no_check)
        print("Test Mode:", self.test_mode)

#-------------------------------------------------------------------------------
class SerTest:
    """ Serial-Test Class """

    def __init__(self, cfg):
        self.cfg = cfg

    def create_test_data(self, elements_cnt):
        """ Create a template of test data
        """
        if self.cfg.bit_cnt > 8:
            test_data = bytearray(elements_cnt * 2)
            for i in range(elements_cnt):
                if self.cfg.data_hblb:
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

    def  calculate_send_time(self, elements_cnt):
        """ Calculate time required to send data:
            1 bit = 1/Baud-Rate
            1 frame = 1 start + bit_cnt + 1 stop = (2 + bit_cnt)/Baud-Rate
            elements_cnt frames = ((2 + bit_cnt)/Baud-Rate) * elements_cnt
            Add a 1.1 factor as reserve and limit the minimal time to 50ms
        """
        send_time = ((2.0 + self.cfg.bit_cnt)/self.cfg.baud_rate) * (elements_cnt) * 1.1
        if send_time < MIN_TIMEOUT_S:
            send_time = MIN_TIMEOUT_S
        return send_time

    def dev_send_receive(self, test_text, dev_tx, dev_rx, elements_cnt):
        """ Create a buffer of test-data of size bytes_count and send it on dev_tx.
            Receive data using dev_rx, if flag no_check is 0, check if the data
            sent is the same as data received.
        """
        test_data = self.create_test_data(elements_cnt)

        dev_tx.write(test_data)
        dev_tx.flush()
        time.sleep(self.calculate_send_time(elements_cnt))

        # Calculate expected count of data to be received (based on bit count)
        # Add a reserve of 4 bytes for the buffer size
        max_rx_bytes = (elements_cnt + 4)
        rx_bytes_expect = elements_cnt
        if self.cfg.bit_cnt > 8:
            max_rx_bytes = max_rx_bytes * 2
            rx_bytes_expect = rx_bytes_expect * 2

        rx_bytes = dev_rx.read(max_rx_bytes)

        # Intentionally inject an error (to test if we detect it)
        #if elements_cnt == 30:
        #    test_data[10] = 0

        if self.cfg.no_check == 0:

            # Less bytes received? Sleep more and try to receive again
            if len(rx_bytes) < rx_bytes_expect:
                time.sleep(self.calculate_send_time(elements_cnt))
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

    def execute(self):
        """ Execute serial test 
        """
        cfg = self.cfg

        dev1 = serial.Serial(cfg.dev1_name, cfg.baud_rate, timeout=0)
        dev2 = serial.Serial(cfg.dev2_name, cfg.baud_rate, timeout=0)

        # Give time (~10ms) to pico to configure the UART interface
        time.sleep(0.01)

        test_passed = True
        for i in range(cfg.range_from, cfg.range_to + 1):

            # Default dev1->dev2
            dev_tx, dev_rx = dev1, dev2
            dev_tx_name, dev_rx_name = cfg.dev1_name, cfg.dev2_name
            elements_count = i

            # Test Mode 1: dev1->dev2, dev2->dev1, dev1->dev2, dev2->dev1, ...
            if cfg.test_mode == 1:
                if i % 2:
                    dev_tx, dev_rx = dev2, dev1
                    dev_tx_name, dev_rx_name = cfg.dev2_name, cfg.dev1_name

            # Test Mode 2: random(dev1, dev2)
            elif cfg.test_mode == 2:
                if random.randint(1, 10) > 5:
                    dev_tx, dev_rx = dev2, dev1
                    dev_tx_name, dev_rx_name = cfg.dev2_name, cfg.dev1_name

            # Test Mode 3: random(packet_len)
            elif cfg.test_mode == 3:
                elements_count = random.randrange(1, cfg.range_to)

            # Test Mode 4: random(dev1, dev2) & random(packet_len)
            elif cfg.test_mode == 4:
                elements_count = random.randrange(1, cfg.range_to)
                if random.randint(1, 10) > 5:
                    dev_tx, dev_rx = dev2, dev1
                    dev_tx_name, dev_rx_name = cfg.dev2_name, cfg.dev1_name

            # Display progress bar
            progress = generate_progress_bar(cfg.range_from, cfg.range_to, i)
            txt_test = '{} {:5d} {}->{} (pack.size: {})'\
                .format(progress, i, dev_tx_name, dev_rx_name, elements_count)
            print(txt_test, sep=' ', end='\r', flush=True)

            # Execute test
            if not self.dev_send_receive(txt_test, dev_tx, dev_rx, elements_count):
                test_passed = False
                break

        dev1.close()
        dev2.close()

        return test_passed


#-------------------------------------------------------------------------------
if __name__ == '__main__':

    # Command line parser
    parser = argparse.ArgumentParser(prog='sertest.py',
                description='Test RPico_CDC_UART firmware.')

    parser.add_argument('dev1',
                help='Serial Device 1 (Ex: /dev/ttyUSB0 for USB-UART converter, /dev/ttyACM0 for TinyUSB)')
    parser.add_argument('dev2',
                help='Serial Device 2 (Ex: /dev/ttyUSB0 for USB-UART converter, /dev/ttyACM0 for TinyUSB)')
    parser.add_argument('-b', '--baudrate', default=DEF_BAUD_RATE, type=int, dest='baudrate',
                help='Baud-Rate (default: %(default)s)')
    parser.add_argument('-f', '--from', default=MIN_RANGE_FROM, type=int, dest='from_value',
                help='Test range from (default: %(default)s)')
    parser.add_argument('-t', '--to', default=MAX_RANGE_TO, type=int, dest='to_value',
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

    if (args.from_value < 1) or (args.to_value > MAX_RANGE_TO) or (args.to_value < args.from_value):
        parser.error(f"FROM_VALUE and TO_VALUE must be between {MIN_RANGE_FROM} and {MAX_RANGE_TO}")

    ser_test_config = SerTestConfig()
    ser_test_config.baud_rate = args.baudrate
    ser_test_config.range_from = args.from_value
    ser_test_config.range_to = args.to_value
    ser_test_config.dev1_name = args.dev1
    ser_test_config.dev2_name = args.dev2
    ser_test_config.no_check = args.no_check
    ser_test_config.test_mode = args.test_mode
    ser_test_config.print()

    ser_test = SerTest(ser_test_config)
    res = ser_test.execute()

    print("End of test, Result:", res)
    if not res:
        sys.exit(1)
    sys.exit()
