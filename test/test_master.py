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

import os
import sys
import time
import argparse
import sertest
import cmake_func

#   | Test Mode | Send Direction         | Packet Length |
#   | --------- | ---------------------- | ------------- |
#   | 0         | dev1->dev2             | incremented   |
#   | 1         | dev2->dev1             | incremented   |
#   | 2         | dev1->dev2, dev2->dev1 | incremented   |
#   | 3         | random                 | incremented   |
#   | 4         | dev1->dev2             | random        |
#   | 5         | dev2->dev1             | random        |
#   | 6         | random                 | random        | 

test_def1 = (
    # Device1:
    '/dev/ttyACM0',
    # Device2:
    '/dev/ttyACM1',
    # Test Cases List:
    [
    #     0  |   1  |  2 | 3 |     4     |       5
    #        | pio- | length |           |
    #   baud |clkdiv| from-to| test-mode | data bits count
    (     238, 65535, 1,   50, [0, 1, 6], [3, 4, 5, 8, 9, 16]),
    (    2400,     0, 1,   50, [0, 1, 6], [3, 6, 8, 9, 10, 16]),
    (    4800,     0, 1,   50, [0, 1, 6], [3, 6, 8, 9, 10, 16]),
    (    9600,     0, 1,  300, [0, 1, 6], [3, 5, 7, 8, 9, 11, 16]),
    (   19200,     0, 1,  500, [0, 1, 6], [3, 7, 9, 14]),
    (   38400,     0, 1,  600, [0, 1, 6], [3, 5, 8, 9, 16]),
    (   57600,     0, 1,  600, [0, 1, 6], [3, 7, 9, 13]),
    (  115200,     0, 1, 1000, [0, 1, 6], [3, 4, 8, 9, 16]),
    (  230400,     0, 1, 1000, [0, 1, 6], [3, 5, 7, 9, 16]),
    (  460800,     0, 1, 1000, [0, 1, 6], [3, 4, 8, 9, 15]),
    (  921600,     0, 1, 1000, [0, 1, 6], [3, 5, 8, 9, 16]),
    ( 1562500,    10, 1, 1000, [0, 1, 6], [16, 9, 8, 6]),
    ]
)

test_def2 = (
    # Device1:
    '/dev/ttyACM0',
    # Device2:
    '/dev/ttyUSB0',
    # Test Cases List:
    [
    #     0  |   1  |  2 | 3 |     4     |    5
    #        | pio- | length |           | data bits
    #   baud |clkdiv| from-to| test-mode |  count
    (    2400,     0, 1,   50, [0, 1, 6], [7, 8]),
    (    4800,     0, 1,   50, [0, 1, 6], [7, 8]),
    (    9600,     0, 1,  300, [0, 1, 6], [7, 8]),
    (   14400,     0, 1,  500, [0, 1, 6], [7, 8]),
    (   19200,     0, 1,  500, [0, 1, 6], [7, 8]),
    (   38400,     0, 1,  600, [0, 1, 6], [7, 8]),
    (   57600,     0, 1,  600, [0, 1, 6], [7, 8]),
    (  115200,     0, 1, 1000, [0, 1, 6], [7, 8]),
    (  230400,     0, 1, 1000, [0, 1, 6], [7, 8]),
    (  460800,     0, 1, 1000, [0, 1, 6], [7, 8]),
    (  921600,     0, 1, 1000, [0, 1, 6], [7, 8]),
    ]
)

#-------------------------------------------------------------------------------
def generate_test_cases(arg_test_def):
    """ From test definition (test_def) generate a test case list 
        (test_case_list) to be passed to sertest module:

        test_def = (
            'dev1',
            'dev2',
            [
            |<---0-->|<----1---->|<-2->|<-3->|<-----4----->|<--------5-------->|
            (baudrate, pio_clkdiv, from,   to, [test_modes], [data_bits_counts])
                ...
            ]
        )

        test_case_list = [
            |<-0->|<-1->|<---2-->|<----3---->|<-4->|<-5->|<---6-->|<----7--->|<----8--->|
            (dev1, dev2, baudrate, pio_clkdiv, from,   to, bit_cnt, test_mode, build_req)
            ...
        ]
        """
    test_case_list = []

    arg_dev1 = arg_test_def[0]
    arg_dev2 = arg_test_def[1]
    arg_test_def_list = arg_test_def[2]

    for rec in arg_test_def_list:
        for bit_cnt in rec[5]:
            build_req = True
            for test_mode in rec[4]:
                test_case_list.append(
                    #     0          1       2      3        4       5      6         7         8
                    #    dev1,     dev2,   baud, pio_clk,  from,     to, bit_cnt, test_mode, build_req)
                    (arg_dev1, arg_dev2, rec[0], rec[1], rec[2], rec[3], bit_cnt, test_mode, build_req))
                build_req = False

    return test_case_list

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
# Command line parser
parser = argparse.ArgumentParser(prog='test_master.py',
            description='Test-Master RPico_CDC_UART firmware.')

parser.add_argument('-s', '--set', default=0, type=int, dest='test_set',
            help='Test set to use while generating test cases (default: %(default)s)')

args = parser.parse_args()

test_set = args.test_set

test_def = test_def1
if test_set == 2:
    test_def = test_def2

#-------------------------------------------------------------------------------
# Check if serial devices exist
if not os.path.exists(test_def[0]):
    print("Device 1 (dev1) not found:", test_def[0])
    sys.exit(1)
if not os.path.exists(test_def[1]):
    print("Device 2 (dev2) not found:", test_def[1])
    sys.exit(1)

#-------------------------------------------------------------------------------
test_case_list = generate_test_cases(test_def)

force_f_build = -1
result = True
for idx, test_case in enumerate(test_case_list):

    print()
    print("***************** Test {:d} ******************".format(idx))

    # Create Serial Test Configuration
    ser_test_config = sertest.SerTestConfig()
    ser_test_config.dev1_name =  test_case[0]
    ser_test_config.dev2_name =  test_case[1]
    ser_test_config.baud_rate =  test_case[2]
    ser_test_config.pio_clkdiv = test_case[3]
    ser_test_config.range_from = test_case[4]
    ser_test_config.range_to =   test_case[5]
    ser_test_config.bit_cnt =    test_case[6]
    ser_test_config.test_mode =  test_case[7]
    ser_test_config.data_hblb =  1
    ser_test_config.no_check =   0
    build = test_case[8]

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

        # In case one of the devices is /dev/ttyUSB*
        # then we need to deploy just in /dev/ttyACM0
        deploy_text = "deployall"
        if ser_test_config.dev1_name.startswith("/dev/ttyUSB") \
        or ser_test_config.dev2_name.startswith("/dev/ttyUSB"):
            deploy_text = "deploy"

        ret = os.system("make -C ../build/ " + deploy_text)
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
