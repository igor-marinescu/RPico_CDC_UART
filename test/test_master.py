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

DEV1 = '/dev/ttyACM0'
DEV2 = '/dev/ttyACM1'

## 9600, 14400, 19200, 38400, 57600, 115200, 128000, 256000

#-------------------------------------------------------------------------------
test_case_list = [
    #                                       no-  test-  
    # dev1 dev2  baud  from   to  bit hblb  check mode build
    #  0    1      2     3    4    5    6     7    8     9
    (DEV1, DEV2,   9600, 1, 1000,  5, True,   0,   0,  True),
    (DEV2, DEV1,   9600, 1, 1000,  5, True,   0,   0,  False),
    (DEV1, DEV2,   9600, 1, 1000,  5, True,   1,   0,  False),
    (DEV2, DEV1,   9600, 1, 1000,  5, True,   1,   0,  False),
    (DEV1, DEV2,   9600, 1, 1000,  5, True,   2,   0,  False),
    (DEV2, DEV1,   9600, 1, 1000,  5, True,   2,   0,  False),
    (DEV1, DEV2,   9600, 1, 1000,  5, True,   3,   0,  False),
    (DEV2, DEV1,   9600, 1, 1000,  5, True,   3,   0,  False),
    (DEV1, DEV2,   9600, 1, 1000,  5, True,   4,   0,  False),
    (DEV2, DEV1,   9600, 1, 1000,  5, True,   4,   0,  False),
    (DEV1, DEV2, 115200, 1, 1000,  8, True,   0,   0,  True),
    (DEV2, DEV1, 115200, 1, 1000,  8, True,   0,   0,  False),
    (DEV1, DEV2, 115200, 1, 1000,  9, True,   0,   0,  True),
    (DEV2, DEV1, 115200, 1, 1000,  9, True,   0,   0,  False),
]

#-------------------------------------------------------------------------------
def change_cmake(file_name, baud_rate, bit_cnt, data_hblb):
    """ A test function to test the above functions.
    """
    # Check if file exists
    if not os.path.isfile(file_name):
        return False

    # Read in the file
    with open(file_name, 'r') as file:
        filedata = file.read()

    filedata = cmake_func.change_key_value(filedata, 'add_compile_definitions', 'CONFIG_UART_BAUDRATE', str(baud_rate))
    if not filedata:
        return False

    filedata = cmake_func.change_key_value(filedata, 'add_compile_definitions', 'CONFIG_UART_DATA_BIT', str(bit_cnt))
    if not filedata:
        return False
    
    filedata = cmake_func.enable_key(filedata, 'add_compile_definitions', 'CONFIG_UART_DATA_HBLB', data_hblb)
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
result = True
for idx, test_case in enumerate(test_case_list):

    print()
    print("***************** Test {:d} ******************".format(idx))

    # Create Serial Test Configuration
    ser_test_config = sertest.SerTestConfig()
    ser_test_config.dev1_name =  test_case[0]
    ser_test_config.dev2_name =  test_case[1]
    ser_test_config.baud_rate =  test_case[2]
    ser_test_config.range_from = test_case[3]
    ser_test_config.range_to =   test_case[4]
    ser_test_config.bit_cnt =    test_case[5]
    ser_test_config.data_hblb =  test_case[6]
    ser_test_config.no_check =   test_case[7]
    ser_test_config.test_mode =  test_case[8]
    ser_test_config.print()

    # Cmake change and rebuild?
    if test_case[9]:
        if not change_cmake('../src/CMakeLists.txt', \
            ser_test_config.baud_rate, \
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

        # Make a pause for the device to reboot
        time.sleep(5.0)

    ser_test = sertest.SerTest(ser_test_config)
    result = ser_test.execute()

    print()
    print("End of test, Result:", result)
    if not result:
        break

if not result:
    sys.exit(1)
sys.exit()
