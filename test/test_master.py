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

# Test PIO-UART<->PIO-UART
import test_master_list1
test_case_list_exe = test_master_list1.test_case_list
dev1 = test_master_list1.DEV1
dev2 = test_master_list1.DEV2

# Test PIO-UART<->FTDI232
#import test_master_list3
#test_case_list_exe = test_master_list3.test_case_list
#dev1 = test_master_list3.DEV1
#dev2 = test_master_list3.DEV2

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
if not os.path.exists(dev1):
    print("Device 1 (dev1) not found:", dev1)
    sys.exit(1)
if not os.path.exists(dev2):
    print("Device 2 (dev2) not found:", dev2)
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
