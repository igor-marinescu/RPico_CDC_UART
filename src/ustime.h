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
 ******************************************************************************/
/*******************************************************************************
 * ustime - calculates the difference between two ustime_t/s_time_t data.
 ******************************************************************************/
#ifndef USTIME_H
#define USTIME_H

//******************************************************************************
// Includes
//******************************************************************************

//******************************************************************************
// Defines
//******************************************************************************

//******************************************************************************
// Typedefs
//******************************************************************************

// System time in micro-seconds
typedef uint32_t ustime_t;

// System time in seconds
typedef uint32_t s_time_t;

//******************************************************************************
// Exported Functions
//******************************************************************************

// Get System Time in us (32 bit)
ustime_t get_sys_ustime(void);

// Get the difference (always positive) between ustime_t variable and system time
ustime_t get_diff_sys_ustime(const ustime_t ustime);

// Get the difference (always positive) between two ustime_t variables
ustime_t get_diff_ustime(const ustime_t end_time, const ustime_t start_time);

// Get the difference (always positive) between two s_time_t variables
s_time_t get_diff_s_time(const s_time_t end_time, const s_time_t start_time);

// Calculate time necessary to transfer a number of bytes over a serial interface
ustime_t get_baudrate_transfer_ustime(unsigned int baudrate, unsigned int bits_pro_byte, unsigned int bytes_cnt);

//******************************************************************************
#endif /* USTIME_H */
