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

def hex_dump(b):
    text_line = ""
    for idx, val in enumerate(b):
        if (idx % 16 == 0) and (idx > 0):
            print(text_line)
            text_line = ""
        text_line += '{:02X} '.format(val)
    if(text_line):
        print(text_line)

def create_test_data(len):
    print("create_test_data(", len, ")")
    test_data = bytearray(len)
    for i in range(len):
        test_data[i] = (i + len) % 256
    return test_data
        
print("hex_dump", 0)
hex_dump(range(0))
        
print("hex_dump", 1)
hex_dump(range(1))

print("hex_dump", 10)
hex_dump(range(10))

print("hex_dump", 15)
hex_dump(range(15))

print("hex_dump", 16)
hex_dump(range(16))

print("hex_dump", 17)
hex_dump(range(17))

print("hex_dump", 31)
hex_dump(range(31))

print("hex_dump", 32)
hex_dump(range(32))

print("hex_dump", 33)
hex_dump(range(33))

print("hex_dump", 100)
hex_dump(range(100))

print("hex_dump", 255)
hex_dump(range(255))

#td = create_test_data(1)
#hex_dump(td)

#td = create_test_data(5)
#hex_dump(td)

#td = create_test_data(10)
#hex_dump(td)

#td = create_test_data(20)
#hex_dump(td)

#td = create_test_data(300)
#hex_dump(td)
