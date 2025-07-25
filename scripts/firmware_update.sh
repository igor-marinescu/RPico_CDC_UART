#!/usr/bin/bash

DEVICE_NAME="/dev/ttyACM0"

# Script invoked with no arguments? Show help.
if [[ "$#" -eq 0 ]]; then
    echo "RPico firmware update using picotool."
    echo "Script arguments:"
    echo "$0 <firmware> [device]"
    echo "  firmware - firmware to update"
    echo "  device - device to be updated (default: ) $DEVICE_NAME"
    exit 1
fi

FIRMWARE_FILE=$1

# Check if firmware exists
if [[ ! -f "$FIRMWARE_FILE" ]]; then
    echo "[ERROR] Firmware file not found: $FIRMWARE_FILE"
    exit 1
fi

# Device specified?
if [[ "$#" -ge 2 ]]; then
    DEVICE_NAME=$2
fi

# Check if picotool is available
PICO_TOOL=$(which picotool)
if [[ ! $? -eq 0 ]]; then
    echo "[ERROR] picotool not found."
    exit 1
fi

# Check if device in Run-Mode
lsusb | grep -q "TinyUSB Device"
if [[ $? -eq 0 ]]; then
    echo "TinyUSB Device found, assuming this is $DEVICE_NAME. Try to jump to Boot."

    if [[ ! -c "$DEVICE_NAME" ]]; then
        echo "[ERROR] Device not found: $DEVICE_NAME"
        exit 1
    fi

    # Set baudrate of the device to 1200, make the device to jump to boot
    stty -F $DEVICE_NAME 1200
    # Wait 1 second
    sleep 1
fi

# Check if device in Bootsel mode
lsusb | grep "Raspberry Pi RP2 Boot"
if [[ ! $? -eq 0 ]]; then
    echo "[ERROR] Raspberry Pi RP2 Boot not found."
fi

echo "Raspberry Pi RP2 Boot found."
echo "Update device with firmware: $FIRMWARE_FILE"
picotool load $FIRMWARE_FILE
if [[ $? -eq 0 ]]; then
    echo "Success, reboot device."
    picotool reboot
else
    echo "[ERROR] Failed to load firmware using picotool."
fi
