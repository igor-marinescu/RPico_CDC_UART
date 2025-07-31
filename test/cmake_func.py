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

import string
import os
import sys

#-------------------------------------------------------------------------------
def change_key_value(str, op_str, key, val):
    """ Search in <str> a <key> which belongs to an <op_str> operation 
        and change its value to <val>.

        Example: add_compile_definitions(CONFIG_UART_BAUDRATE=2152000)
                 |<--------- op_str---->|<------ key ------->|<-val->|

        Returns the corrected string or None in case if the <key> is not found.
    """
    pos_start = 0

    while True:

        # Search the <key>
        pos_start = str.find(key, pos_start)
        pos_new = pos_start
        if pos_new <= 0:
            # Key not found
            return None

        # Look to the left from the <key> for the '(' (also skip spaces)
        pos_new -= 1
        while (pos_new >= 0) and (str[pos_new] in string.whitespace):
            pos_new -= 1

        if pos_new < 0:
            return None

        # '(' found? (possible the start of the key)
        if str[pos_new] != '(':
            # Nope, something else, ignore this <key> occurrence, continue looking
            pos_start += 1
            continue

        # Look to the left from '(' for the <op_str> (also skip spaces)
        pos_new2 = pos_new - 1
        while (pos_new2 >= 0) and (str[pos_new2] in string.whitespace):
            pos_new2 -= 1

        # <op_str> found?
        pos_new2 += 1
        if (pos_new2 < len(op_str)) or not str.startswith(op_str, pos_new2 - len(op_str)):
            # Nope, something else, ignore this occurrence, continue looking
            pos_start += 1
            continue

        # Search where the key ends: position of ')' symbol
        pos_end = str.find(')', pos_start)
        if pos_end < 0:
            return None

        # Replace with new 'key=value'
        return (str[:pos_new + 1] + key + '=' + val + str[pos_end:])

#-------------------------------------------------------------------------------
def enable_key(str, op_str, key, enable):
    """ Search in <str> a <key> which belongs to an <op_str> operation 
        and enable or disable it (by adding/removing '#' symbol in front of operation).

        Example: add_compile_definitions(CONFIG_UART_DATA_HBLB)
                 |<--------- op_str---->|<------- key ------->|

        Will add '#' in case if the operation must be disabled:
                #add_compile_definitions(CONFIG_UART_DATA_HBLB)

        Returns the corrected string or None in case if the <key> is not found.
    """
    pos_start = 0
    line_feed = ['\r', '\n']

    while True:

        # Search the <key>
        pos_start = str.find(key, pos_start)
        pos_new = pos_start
        if pos_new <= 0:
            # Key not found
            return None

        # Look to the left from the <key> for the '(' (also skip spaces)
        pos_new -= 1
        while (pos_new >= 0) and (str[pos_new] in string.whitespace):
            pos_new -= 1

        if pos_new < 0:
            return None

        # '(' found? (possible the start of the key)
        if str[pos_new] != '(':
            # Nope, something else, ignore this <key> occurrence, continue looking
            pos_start += 1
            continue

        # Look to the left from '(' for the <op_str> (also skip spaces)
        pos_new2 = pos_new - 1
        while (pos_new2 >= 0) and (str[pos_new2] in string.whitespace):
            pos_new2 -= 1

        # <op_str> found?
        pos_new2 += 1
        if (pos_new2 < len(op_str)) or not str.startswith(op_str, pos_new2 - len(op_str)):
            # Nope, something else, ignore this occurrence, continue looking
            pos_start += 1
            continue

        # Skip any spaces from the left of <op_str>
        pos_new2 -= (len(op_str) + 1)
        while (pos_new2 >= 0) and (str[pos_new2] in string.whitespace) \
            and (str[pos_new2] not in line_feed):
            pos_new2 -= 1

        # Special case: Start of file and already enabled?
        if pos_new2 < 0:
            if not enable:
                # Disable (add '#')
                return ('#' + str)
            # Already enabled, nothing to do
            return str

        # Start of line?
        if str[pos_new2] in line_feed:
            if not enable:
                # Disable (add '#')
                return (str[:pos_new2 + 1] + '#' + str[pos_new2 + 1:])
            # Already enabled, nothing to do
            return str
        elif str[pos_new2] == '#':
            # Enable (remove '#')
            if enable:
                return (str[:pos_new2] + str[pos_new2 + 1:])
            # Already disabled, nothing to do
            return str
        else:
            # Unexpected character, ignore this occurrence, continue looking
            pos_start += 1
            continue

        # Replace with new 'key=value'
        #return (str[:pos_new + 1] + key + '=' + val + str[pos_end:])
        return None

#-------------------------------------------------------------------------------
def test_change_cmake(file_name):
    """ A test function to test the above functions.
    """
    # Check if file exists
    if not os.path.isfile(file_name):
        return False

    # Read in the file
    with open(file_name, 'r') as file:
        filedata = file.read()

    filedata = change_key_value(filedata, 'add_compile_definitions', 'CONFIG_UART_BAUDRATE', str(215211))
    if not filedata:
        return False

    filedata = change_key_value(filedata, 'add_compile_definitions', 'CONFIG_UART_DATA_BIT', str(20))
    if not filedata:
        return False
    
    filedata = enable_key(filedata, 'add_compile_definitions', 'CONFIG_UART_DATA_HBLB', False)
    if not filedata:
        return False

    # Write the file out again
    with open(file_name, 'w') as file:
        file.write(filedata)

    return True

#-------------------------------------------------------------------------------
if __name__ == '__main__':

    if len(sys.argv) < 2:
        print("File argument missing")
        sys.exit(1)

    if test_change_cmake(sys.argv[1]):
        print("File successfully modified")
    else:
        print("Error modifying file")
