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

DEV1 = '/dev/ttyACM0'
DEV2 = '/dev/ttyACM1'

#-------------------------------------------------------------------------------
test_case_list = [
    # dev1 dev2  baud  from  to  bit hblb nocheck testmode
    #  0    1      2     3   4    5    6     7    8
    (DEV1, DEV2, 115200, 1, 2050, 9, True, False, 0),
    (DEV2, DEV1, 115200, 1, 2050, 9, True, False, 0),
    (DEV1, DEV2, 115200, 1, 2050, 9, True, False, 1),
    (DEV2, DEV1, 115200, 1, 2050, 9, True, False, 1),
    (DEV1, DEV2, 115200, 1, 2050, 9, True, False, 2),
    (DEV2, DEV1, 115200, 1, 2050, 9, True, False, 2),
    (DEV1, DEV2, 115200, 1, 2050, 9, True, False, 3),
    (DEV2, DEV1, 115200, 1, 2050, 9, True, False, 3),
    (DEV1, DEV2, 115200, 1, 2050, 9, True, False, 4),
    (DEV2, DEV1, 115200, 1, 2050, 9, True, False, 4),
]

#-------------------------------------------------------------------------------
#if change_cmake('../src/CMakeLists.txt'):
    #os.system("make -C ../build/")
    #pass

#-------------------------------------------------------------------------------
result = True
for idx, test_case in enumerate(test_case_list):

    print("-------- Test {:d} ---------".format(idx))

    # Create Serial Test Configuration
    ser_test_config = sertest.SerTestConfig()
    ser_test_config.dev1_name = test_case[0]
    ser_test_config.dev2_name = test_case[1]
    ser_test_config.baud_rate = test_case[2]
    ser_test_config.range_from = test_case[3]
    ser_test_config.range_to = test_case[4]
    ser_test_config.bit_cnt = test_case[5]
    ser_test_config.data_hblb = test_case[6]
    ser_test_config.no_check = test_case[7]
    ser_test_config.test_mode = test_case[8]
    ser_test_config.print()

    ser_test = sertest.SerTest(ser_test_config)
    result = ser_test.execute()

    print()
    print("End of test, Result:", result)

if not result:
    sys.exit(1)
sys.exit()
