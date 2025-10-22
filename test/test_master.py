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

import sys
import sertest
import os
import cmake_func
import time
import argparse

DEV1 = '/dev/ttyACM0'
DEV2 = '/dev/ttyACM1'

# Standard Baud-rates 9600, 14400, 19200, 38400, 57600, 115200, 230400
# Test Modes:
#   | Test Mode | Send Direction         | Packet Length |
#   | --------- | ---------------------- | ------------- |
#   | 0         | dev1->dev2             | incremented   |
#   | 1         | dev1->dev2, dev2->dev1 | incremented   |
#   | 2         | random                 | incremented   |
#   | 3         | dev1->dev2             | random        |
#   | 4         | random                 | random        | 

#-------------------------------------------------------------------------------
test_case_list_1 = [
    #                       pio-                        no-    test-  
    # dev1 dev2     baud   clkdiv  from  to  bit  hblb  check  mode build
    #   0     1        2      3      4    5    6    7    8     9
    (DEV1, DEV2,     238, 65535,     1,   50,  9,   1,   0,    0,   True),   # 0
    (DEV2, DEV1,     238, 65535,     1,   50,  9,   1,   0,    0,   False),  # 1
    (DEV1, DEV2,     238, 65535,     1,   50,  9,   1,   0,    4,   False),  # 2

    (DEV1, DEV2,    9600,     0,     1,  300,  9,   1,   0,    0,   True),   # 3
    (DEV2, DEV1,    9600,     0,     1,  300,  9,   1,   0,    0,   False),  # 4
    (DEV1, DEV2,    9600,     0,     1,  300,  9,   1,   0,    4,   False),  # 5

    (DEV1, DEV2,   14400,     0,     1,  500,  9,   1,   0,    1,   True),   # 6
    (DEV2, DEV1,   14400,     0,     1,  500,  9,   1,   0,    2,   False),  # 7
    (DEV1, DEV2,   14400,     0,     1,  500,  9,   1,   0,    3,   False),  # 8

    (DEV1, DEV2,   19200,     0,     1,  500,  9,   1,   0,    0,   True),   # 9
    (DEV2, DEV1,   19200,     0,     1,  500,  9,   1,   0,    0,   False),  # 10
    (DEV1, DEV2,   19200,     0,     1,  500,  9,   1,   0,    4,   False),  # 11

    (DEV1, DEV2,   38400,     0,     1,  600,  9,   1,   0,    1,   True),   # 12
    (DEV2, DEV1,   38400,     0,     1,  600,  9,   1,   0,    2,   False),  # 13
    (DEV1, DEV2,   38400,     0,     1,  600,  9,   1,   0,    3,   False),  # 14

    (DEV1, DEV2,   57600,     0,     1,  600,  9,   1,   0,    1,   True),   # 15
    (DEV2, DEV1,   57600,     0,     1,  600,  9,   1,   0,    2,   False),  # 16
    (DEV1, DEV2,   57600,     0,     1,  600,  9,   1,   0,    3,   False),  # 17

    (DEV1, DEV2,  115200,     0,     1, 1000,  9,   1,   0,    0,   True),   # 18
    (DEV2, DEV1,  115200,     0,     1, 1000,  9,   1,   0,    0,   False),  # 19
    (DEV1, DEV2,  115200,     0,     1, 1000,  9,   1,   0,    1,   False),  # 20
    (DEV1, DEV2,  115200,     0,     1, 1000,  9,   1,   0,    2,   False),  # 21
    (DEV1, DEV2,  115200,     0,     1, 1000,  9,   1,   0,    4,   False),  # 22

    (DEV1, DEV2,  230400,     0,     1, 1000,  9,   1,   0,    0,   True),   # 23
    (DEV2, DEV1,  230400,     0,     1, 1000,  9,   1,   0,    0,   False),  # 24
    (DEV1, DEV2,  230400,     0,     1, 1000,  9,   1,   0,    1,   False),  # 25
    (DEV1, DEV2,  230400,     0,     1, 1000,  9,   1,   0,    2,   False),  # 26
    (DEV1, DEV2,  230400,     0,     1, 1000,  9,   1,   0,    4,   False),  # 27

    (DEV1, DEV2, 1562500,     5,     1, 1000,  9,   1,   0,    0,   True),   # 23
    (DEV2, DEV1, 1562500,     5,     1, 1000,  9,   1,   0,    0,   False),  # 24
    (DEV1, DEV2, 1562500,     5,     1, 1000,  9,   1,   0,    1,   False),  # 25
    (DEV1, DEV2, 1562500,     5,     1, 1000,  9,   1,   0,    2,   False),  # 26
    (DEV1, DEV2, 1562500,     5,     1, 1000,  9,   1,   0,    4,   False),  # 27
]

test_case_list_2 = [
    #                      pio-                        no-    test-  
    # dev1 dev2    baud   clkdiv  from  to   bit  hblb  check  mode build
    #   0     1       2      3      4    5     6    7    8     9
    (DEV1, DEV2,  781250,    20,     1, 1000,  3,   1,   0,    0,   True),   # 0
    (DEV2, DEV1,  781250,    20,     1, 1000,  3,   1,   0,    0,   False),  # 1
    (DEV2, DEV1,  781250,    20,     1, 1000,  3,   1,   0,    4,   False),  # 2

    (DEV1, DEV2,  781250,    20,     1, 1000,  4,   1,   0,    0,   True),   # 3
    (DEV2, DEV1,  781250,    20,     1, 1000,  4,   1,   0,    0,   False),  # 4
    (DEV2, DEV1,  781250,    20,     1, 1000,  4,   1,   0,    4,   False),  # 5

    (DEV1, DEV2,  781250,    20,     1, 1000,  5,   1,   0,    0,   True),   # 6
    (DEV2, DEV1,  781250,    20,     1, 1000,  5,   1,   0,    0,   False),  # 7
    (DEV2, DEV1,  781250,    20,     1, 1000,  5,   1,   0,    4,   False),  # 8

    (DEV1, DEV2,  781250,    20,     1, 1000,  6,   1,   0,    0,   True),   # 9
    (DEV2, DEV1,  781250,    20,     1, 1000,  6,   1,   0,    0,   False),  # 10
    (DEV2, DEV1,  781250,    20,     1, 1000,  6,   1,   0,    4,   False),  # 11

    (DEV1, DEV2,  781250,    20,     1, 1000,  7,   1,   0,    0,   True),   # 12
    (DEV2, DEV1,  781250,    20,     1, 1000,  7,   1,   0,    0,   False),  # 13
    (DEV2, DEV1,  781250,    20,     1, 1000,  7,   1,   0,    4,   False),  # 14

    (DEV1, DEV2,  781250,    20,     1, 1000,  8,   1,   0,    0,   True),   # 15
    (DEV2, DEV1,  781250,    20,     1, 1000,  8,   1,   0,    0,   False),  # 16
    (DEV2, DEV1,  781250,    20,     1, 1000,  8,   1,   0,    4,   False),  # 17

    (DEV1, DEV2,  781250,    20,     1, 1000,  9,   1,   0,    0,   True),   # 18
    (DEV2, DEV1,  781250,    20,     1, 1000,  9,   1,   0,    0,   False),  # 19
    (DEV2, DEV1,  781250,    20,     1, 1000,  9,   1,   0,    4,   False),  # 20

    (DEV1, DEV2,  781250,    20,     1, 1000, 10,   1,   0,    0,   True),   # 21
    (DEV2, DEV1,  781250,    20,     1, 1000, 10,   1,   0,    0,   False),  # 22
    (DEV2, DEV1,  781250,    20,     1, 1000, 10,   1,   0,    4,   False),  # 23

    (DEV1, DEV2,  781250,    20,     1, 1000, 11,   1,   0,    0,   True),   # 24
    (DEV2, DEV1,  781250,    20,     1, 1000, 11,   1,   0,    0,   False),  # 25
    (DEV2, DEV1,  781250,    20,     1, 1000, 11,   1,   0,    4,   False),  # 26

    (DEV1, DEV2,  781250,    20,     1, 1000, 12,   1,   0,    0,   True),   # 27
    (DEV2, DEV1,  781250,    20,     1, 1000, 12,   1,   0,    0,   False),  # 28
    (DEV2, DEV1,  781250,    20,     1, 1000, 12,   1,   0,    4,   False),  # 29

    (DEV1, DEV2,  781250,    20,     1, 1000, 16,   1,   0,    0,   True),   # 30
    (DEV2, DEV1,  781250,    20,     1, 1000, 16,   1,   0,    0,   False),  # 31
    (DEV2, DEV1,  781250,    20,     1, 1000, 16,   1,   0,    4,   False),  # 32
]

# Test case to execute
test_case_list_exe = test_case_list_2

#-------------------------------------------------------------------------------
def change_cmake(file_name, baud_rate, pio_clkdiv, bit_cnt, data_hblb):
    """ Modify the CMake file
    """
    # Check if file exists
    if not os.path.isfile(file_name):
        return False

    # Read in the file
    with open(file_name, 'r') as file:
        filedata = file.read()

    filedata = cmake_func.change_set_var_value(filedata, 'use_uart_baudrate', str(baud_rate))
    if not filedata:
        return False

    filedata = cmake_func.change_set_var_value(filedata, 'use_pio_clkdiv', str(pio_clkdiv))
    if not filedata:
        return False

    filedata = cmake_func.change_set_var_value(filedata, 'use_uart_data_bit', str(bit_cnt))
    if not filedata:
        return False
    
    filedata = cmake_func.change_set_var_value(filedata, 'use_uart_data_hblb', str(data_hblb))
    if not filedata:
        return False

    # Write the file out again
    with open(file_name, 'w') as file:
        file.write(filedata)

    return True

#-------------------------------------------------------------------------------
# Check if serial devices exist
if not os.path.exists(DEV1):
    print("Device 1 (dev1) not found:", DEV1)
    sys.exit(1)
if not os.path.exists(DEV2):
    print("Device 2 (dev2) not found:", DEV2)
    sys.exit(1)

#-------------------------------------------------------------------------------

# Command line parser
parser = argparse.ArgumentParser(prog='test_master.py',
            description='Test-Master RPico_CDC_UART firmware.')

parser.add_argument('-f', '--from-idx', default=0, type=int, dest='test_from_idx',
            help='Test Index range From (default: %(default)s)')

parser.add_argument('-t', '--to-idx', default=-1, type=int, dest='test_to_idx',
            help='Test Index range To (default: %(default)s)')

parser.add_argument('-b', '--build', default=-1, type=int, dest='force_f_build',
            help='Force first build (default: %(default)s)')

args = parser.parse_args()

test_from_idx = args.test_from_idx
test_to_idx = args.test_to_idx
force_f_build = args.force_f_build

if test_to_idx == -1:
    test_to_idx = len(test_case_list_exe) - 1

if (test_from_idx < 0) or (test_from_idx > test_to_idx):
    parser.error(f"Test Index range From must be between 0 and {len(test_case_list_exe)}")

if (test_to_idx < 0) or (test_to_idx >= len(test_case_list_exe)):
    parser.error(f"Test Index range To must be between 0 and {len(test_case_list_exe)}")

if test_to_idx < test_from_idx:
    parser.error(f"Test Index range From must be < Test Index range To")

test_case_list = test_case_list_exe[test_from_idx:test_to_idx + 1]

#-------------------------------------------------------------------------------
result = True
for idx, test_case in enumerate(test_case_list):

    print()
    print("***************** Test {:d} ******************".format(test_from_idx + idx))

    # Create Serial Test Configuration
    ser_test_config = sertest.SerTestConfig()
    ser_test_config.dev1_name =  test_case[0]
    ser_test_config.dev2_name =  test_case[1]
    ser_test_config.baud_rate =  test_case[2]
    ser_test_config.pio_clkdiv = test_case[3]
    ser_test_config.range_from = test_case[4]
    ser_test_config.range_to =   test_case[5]
    ser_test_config.bit_cnt =    test_case[6]
    ser_test_config.data_hblb =  test_case[7]
    ser_test_config.no_check =   test_case[8]
    ser_test_config.test_mode =  test_case[9]
    build = test_case[10]

    # Cmake change and rebuild?
    if (force_f_build > 0) or ((force_f_build == -1) and build):
        if not change_cmake('../src/CMakeLists.txt', \
            ser_test_config.baud_rate, \
            ser_test_config.pio_clkdiv, \
            ser_test_config.bit_cnt, \
            ser_test_config.data_hblb):
            print("Error: Cannot change cmake")
            result = False
            break

        print("Cmake successfully changed")

        ret = os.system("make -C ../build/ deployall")
        if ret != 0:
            print("Error executing make")
            result = False
            break

        # Make a pause for the devices to reboot
        time.sleep(5.0)

    force_f_build = -1

    ser_test_config.print()
    ser_test = sertest.SerTest(ser_test_config)
    result = ser_test.execute()

    # Sometimes the buffer contains some old data and the first test will fail
    # If the test failed with te first attempt, make one more additional attempt
    if not result[0] and (result[1] == ser_test_config.range_from):
        print("First test failed, make one more attempt")
        ser_test = sertest.SerTest(ser_test_config)
        result = ser_test.execute()

    print()
    print("End of test, Result:", result)
    if not result[0]:
        break

if not result:
    sys.exit(1)
sys.exit()
