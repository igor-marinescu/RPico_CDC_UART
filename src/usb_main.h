/*******************************************************************************
 * This file is part of the RPico_CDC_UART distribution.
 * Copyright (c) 2025 Igor Marinescu (igor.marinescu@gmail.com).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Inspired/used as example from tinyusb.org: 
 * Copyright (c) 2019 Ha Thach / The MIT License (MIT)
 ******************************************************************************/

#ifndef USB_MAIN_H
#define USB_MAIN_H

//******************************************************************************
// Includes
//******************************************************************************
#include "tusb.h"

//******************************************************************************
// Defines
//******************************************************************************

//******************************************************************************
// Exported Functions
//******************************************************************************

// Init USB Task
void usb_main(void);

// Check if received data available and copy to buffer, return cnt of copied bytes
uint32_t usb_get_rx(uint8_t * buff, uint32_t buff_len);

// Set a buffer of data to be send on USB, return cnt of bytes written in tx buffer
uint32_t usb_set_tx(const uint8_t * buff, uint32_t buff_len);

// Return the count of line code changes
uint32_t usb_has_line_coding_changed(void);

// Copy the currently used line code settings
uint32_t usb_get_line_coding(cdc_line_coding_t * ptr_line_coding);

//******************************************************************************
#endif /* UART_DRV_H */
