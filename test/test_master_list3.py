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

DEV1 = '/dev/ttyACM0'
DEV2 = '/dev/ttyUSB0'

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
test_case_list = [
    #                       pio-                        no-    test-  
    # dev1 dev2     baud   clkdiv  from   to   bit  hblb  check  mode build
    #   0     1        2      3      4     5     6    7    8     9
    (DEV1, DEV2,    9600,     0,     1,  300,    7,   1,   0,    0,   True),   # 0
    (DEV2, DEV1,    9600,     0,     1,  300,    7,   1,   0,    0,   False),  # 1
    (DEV2, DEV1,    9600,     0,     1,  100,    7,   1,   0,    4,   False),  # 2

    (DEV1, DEV2,    9600,     0,     1,  300,    8,   1,   0,    0,   True),   # 3
    (DEV2, DEV1,    9600,     0,     1,  300,    8,   1,   0,    0,   False),  # 4
    (DEV2, DEV1,    9600,     0,     1,  100,    8,   1,   0,    4,   False),  # 5

    (DEV1, DEV2,   14400,     0,     1,  300,    7,   1,   0,    0,   True),   # 6
    (DEV2, DEV1,   14400,     0,     1,  300,    7,   1,   0,    0,   False),  # 7
    (DEV2, DEV1,   14400,     0,     1,  100,    7,   1,   0,    4,   False),  # 8

    (DEV1, DEV2,   14400,     0,     1,  300,    8,   1,   0,    0,   True),   # 9
    (DEV2, DEV1,   14400,     0,     1,  300,    8,   1,   0,    0,   False),  # 10
    (DEV2, DEV1,   14400,     0,     1,  100,    8,   1,   0,    4,   False),  # 11

    (DEV1, DEV2,   19200,     0,     1,  500,    7,   1,   0,    0,   True),   # 12
    (DEV2, DEV1,   19200,     0,     1,  500,    7,   1,   0,    0,   False),  # 13
    (DEV2, DEV1,   19200,     0,     1,  500,    7,   1,   0,    4,   False),  # 14

    (DEV1, DEV2,   19200,     0,     1,  500,    8,   1,   0,    0,   True),   # 15
    (DEV2, DEV1,   19200,     0,     1,  500,    8,   1,   0,    0,   False),  # 16
    (DEV2, DEV1,   19200,     0,     1,  500,    8,   1,   0,    4,   False),  # 17

    (DEV1, DEV2,   38400,     0,     1,  500,    7,   1,   0,    0,   True),   # 18
    (DEV2, DEV1,   38400,     0,     1,  500,    7,   1,   0,    0,   False),  # 19
    (DEV2, DEV1,   38400,     0,     1,  500,    7,   1,   0,    4,   False),  # 20

    (DEV1, DEV2,   38400,     0,     1,  500,    8,   1,   0,    0,   True),   # 21
    (DEV2, DEV1,   38400,     0,     1,  500,    8,   1,   0,    0,   False),  # 22
    (DEV2, DEV1,   38400,     0,     1,  500,    8,   1,   0,    4,   False),  # 23

    (DEV1, DEV2,   57600,     0,     1,  500,    7,   1,   0,    0,   True),   # 24
    (DEV2, DEV1,   57600,     0,     1,  500,    7,   1,   0,    0,   False),  # 25
    (DEV2, DEV1,   57600,     0,     1,  500,    7,   1,   0,    4,   False),  # 26

    (DEV1, DEV2,   57600,     0,     1,  500,    8,   1,   0,    0,   True),   # 27
    (DEV2, DEV1,   57600,     0,     1,  500,    8,   1,   0,    0,   False),  # 28
    (DEV2, DEV1,   57600,     0,     1,  500,    8,   1,   0,    4,   False),  # 29

    (DEV1, DEV2,  115200,     0,     1, 1000,    7,   1,   0,    0,   True),   # 30
    (DEV2, DEV1,  115200,     0,     1, 1000,    7,   1,   0,    0,   False),  # 31
    (DEV2, DEV1,  115200,     0,     1, 1000,    7,   1,   0,    4,   False),  # 32

    (DEV1, DEV2,  115200,     0,     1, 1000,    8,   1,   0,    0,   True),   # 33
    (DEV2, DEV1,  115200,     0,     1, 1000,    8,   1,   0,    0,   False),  # 34
    (DEV2, DEV1,  115200,     0,     1, 1000,    8,   1,   0,    4,   False),  # 35

    (DEV1, DEV2,  230400,     0,     1, 1000,    7,   1,   0,    0,   True),   # 36
    (DEV2, DEV1,  230400,     0,     1, 1000,    7,   1,   0,    0,   False),  # 37
    (DEV2, DEV1,  230400,     0,     1, 1000,    7,   1,   0,    4,   False),  # 38

    (DEV1, DEV2,  230400,     0,     1, 1000,    8,   1,   0,    0,   True),   # 39
    (DEV2, DEV1,  230400,     0,     1, 1000,    8,   1,   0,    0,   False),  # 40
    (DEV2, DEV1,  230400,     0,     1, 1000,    8,   1,   0,    4,   False),  # 41
]
