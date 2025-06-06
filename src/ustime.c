/*******************************************************************************
 * This file is part of the MstHora distribution.
 * Copyright (c) 2024 Igor Marinescu (igor.marinescu@gmail.com).
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

//******************************************************************************
// Includes
//******************************************************************************
#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "ustime.h"

//******************************************************************************
// Function Prototypes
//******************************************************************************

//******************************************************************************
// Global Variables
//******************************************************************************

/***************************************************************************//**
* @brief Get System Time in us (32 bit)
*        Warning! This is not safe in case of multi-core
* @return System Time in us
*******************************************************************************/
ustime_t get_sys_ustime(void)
{
    uint32_t lr = timer_hw->timelr;
    timer_hw->timehr;
    return (ustime_t) lr;
}

/*******************************************************************************
 * @brief Get the difference (always positive) between ustime_t variables
 *         and system time.
 * @param ustime [in] ustime_t variable
 * @return difference: system time - ustime
 ******************************************************************************/
ustime_t get_diff_sys_ustime(const ustime_t ustime)
{
    int32_t diff = ((int32_t) get_sys_ustime()) - ((int32_t) ustime);
    if(diff < 0)
        return (ustime_t)(-diff);
    return (ustime_t)(diff);
}

/***************************************************************************//**
* @brief Get the difference (always positive) between two ustime_t variables
* @param end_ustime [in] end ustime_t variable
* @param start_ustime [in] start ustime_t variable
* @return difference: end_ustime - start_ustime
*******************************************************************************/
ustime_t get_diff_ustime(const ustime_t end_ustime, const ustime_t start_ustime)
{
    int32_t diff = ((int32_t) end_ustime) - ((int32_t) start_ustime);
    if(diff < 0)
        return (ustime_t)(-diff);
    return (ustime_t)(diff);
}

/***************************************************************************//**
* @brief Get the difference (always positive) between two s_time_t variables
* @param end_time [in] end s_time_t variable
* @param start_time [in] start s_time_t variable
* @return difference: end_time - start_time
*******************************************************************************/
s_time_t get_diff_s_time(const s_time_t end_time, const s_time_t start_time)
{
    int32_t diff = ((int32_t) end_time) - ((int32_t) start_time);
    if(diff < 0)
        return (s_time_t)(-diff);
    return (s_time_t)(diff);
}

/*******************************************************************************
 * @brief Calculate time necessary to transfer a number of bytes over a serial interface
 * 
 *              1 [s] * bytes_cnt * bits_pro_byte [bits]
 *  time [us] = ----------------------------------------
 *                       baudrate [bits/s]
 * 
 * @param [in] baudrate
 * @param [in] bits_pro_byte - count of bits pro byte (pro frame)
 * @param [in] bytes_cnt - count of bytes (frames) to transfer
 * @return time in [us] required to transfer a number of bytes (frames)
*******************************************************************************/
ustime_t get_baudrate_transfer_ustime(unsigned int baudrate, unsigned int bits_pro_byte, unsigned int bytes_cnt)
{
    if(baudrate == 0)
        return 0;

    unsigned int val = bits_pro_byte * bytes_cnt * 1000000;

    return (ustime_t)(val / baudrate);
}
