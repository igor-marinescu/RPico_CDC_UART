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
 * utils - the implementation of a set of utility functions.
 ******************************************************************************/

//******************************************************************************
// Includes
//******************************************************************************
#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "utils.h"

//******************************************************************************
// Function Prototypes
//******************************************************************************

//******************************************************************************
// Global Variables
//******************************************************************************

/***************************************************************************//**
* @brief Extract integer value from a text-string
* @param out_val [out] pointer to integer where the extracted int is copied
* @param in_str [in] pointer to the input string
* @param in_max_len [in] maximal length of the input string
* @return End position in in_str of the extracted integer or -1 in case of error
*        Example:        01234567890123
*                in_str = "   1234   ABC "
*                *out_val = 1234
*                return = 7
*******************************************************************************/
int utils_extract_int(int * out_val, const char * in_str, int in_max_len)
{
    int val = 0, idx = 0;
    bool negative = false;
    bool valid = false;

    if((in_str == NULL) || (in_max_len <= 0))
        return -1;

    // Ignore spaces/tabs at the beginning
    while(((in_str[idx] == ' ') || (in_str[idx] == '\t')) && (idx < in_max_len))
        idx++;

    // If the string starts with '-'
    if((in_str[idx] == '-') && (idx < in_max_len))
    {
        negative = true;
        idx++;
    }

    while(idx < in_max_len)
    {
        const char ch = in_str[idx];

        if((ch >= '0') && (ch <= '9'))
        {
            val *= 10;
            val += (int) (ch - '0');
            idx++;
            valid = true;
        }
        else
            break;
    }

    if(valid)
    {
        if(out_val != NULL)
            *out_val = negative ? -val : val;
        return idx;
    }

    return -1;
}

/***************************************************************************//**
* @brief Convert a string to a decimal integer. The string must not contain
*        leading or trailing spaces/tabs. It must end with '\0'. Only the 
*        following characters are allowed '-0123456789'
* @param out_val [out] pointer to integer where converted value is stored
* @param in_str [in] string containing an integer (in decimal representation)
* @param in_max_len [in] maximal length of in_str string
* @return true if string successfully converted to integer or false in case of error
*******************************************************************************/
bool utils_get_int_dec(int * out_val, const char * in_str, int in_max_len)
{
    int val = 0, idx = 0;
    bool negative = false;

    if((in_str == NULL) || (in_max_len <= 0))
        return false;

    // If the string starts with '-'
    if(*in_str == '-')
    {
        negative = true;
        in_str++;
        idx++;
    }

    while(idx < in_max_len)
    {
        if(*in_str == '\0')
            break;

        if((*in_str < '0') || (*in_str > '9'))
            return false;

        val *= 10;
        val += (int) (*in_str - '0');
        in_str++;
        idx++;
    }

    if(negative)
    {
        val = -val;
        idx--;
    }

    if((idx > 0) && (out_val != NULL))
    {
        *out_val = val;
    }

    return (idx > 0);
}

/***************************************************************************//**
* @brief Convert a string representing a hexadecimal integer to integer. 
*        The string must not contain leading or trailing spaces/tabs. 
*        It must end with '\0'. Only the characters are allowed: '0123456789ABCDEF'
* @param out_val [out] pointer to integer where converted value is stored
* @param in_str [in] string containing an integer (in hexadecimal representation)
* @param in_max_len [in] maximal length of in_str string
* @return true if string successfully converted to integer or false in case of error
*******************************************************************************/
bool utils_get_int_hex(int * out_val, const char * in_str, int in_max_len)
{
    int val = 0, idx = 0;

    while(idx < in_max_len)
    {
        if(*in_str == '\0')
            break;

        val <<= 4;
        if((*in_str >= '0') && (*in_str <= '9'))
            val += (int) (*in_str - '0');
        else if((*in_str >= 'A') && (*in_str <= 'F'))
            val += 10 + (int) (*in_str - 'A');
        else if((*in_str >= 'a') && (*in_str <= 'f'))
            val += 10 + (int) (*in_str - 'a');
        else
            return false;

        in_str++;
        idx++;
    }

    if((idx > 0) && (out_val != NULL))
    {
        *out_val = val;
    }

    return (idx > 0);
}

/***************************************************************************//**
* @brief Convert a string representing an integer (decimal or hexadecimal form) 
*        to integer. If the string contain an integer in a hexadecimal form, 
*        it must start with '0x'
* @param out_val [out] pointer to integer where converted value is stored
* @param in_str [in] string containing an integer (in decimal or hex representation)
* @param in_max_len [in] maximal length of in_str string
* @return true if string successfully converted to integer or false in case of error
*******************************************************************************/
bool utils_get_int(int * out_val, const char * in_str, int in_max_len)
{
    // If the string is less than 3 symbols it can be decoded only as decimal
    // (hexadecimal has '0x' which makes the length of a string min 3 symbols)
    if(in_max_len < 3)
        return utils_get_int_dec(out_val, in_str, in_max_len);

    // Hexadecimal format?
    if((in_str[0] == '0') && ((in_str[1] == 'x') || (in_str[1] == 'X')))
        return utils_get_int_hex(out_val, &in_str[2], in_max_len - 2);

    return utils_get_int_dec(out_val, in_str, in_max_len);
}

/***************************************************************************//**
* @brief Convert a string to a decimal long-integer. The string must not contain
*        leading or trailing spaces/tabs. It must end with '\0'. Only the 
*        following characters are allowed '-0123456789'
* @param out_val [out] pointer to long-integer where converted value is stored
* @param in_str [in] string containing an long-integer (in decimal representation)
* @param in_max_len [in] maximal length of in_str string
* @return true if string successfully converted to long-integer or false in case of error
*******************************************************************************/
bool utils_get_long_dec(long * out_val, const char * in_str, int in_max_len)
{
    long val = 0;
    int idx = 0;
    bool negative = false;

    if((in_str == NULL) || (in_max_len <= 0))
        return false;

    // If the string starts with '-'
    if(*in_str == '-')
    {
        negative = true;
        in_str++;
        idx++;
    }

    while(idx < in_max_len)
    {
        if(*in_str == '\0')
            break;

        if((*in_str < '0') || (*in_str > '9'))
            return false;

        val *= 10L;
        val += (long) (*in_str - '0');
        in_str++;
        idx++;
    }

    if(negative)
    {
        val = -val;
        idx--;
    }

    if((idx > 0) && (out_val != NULL))
    {
        *out_val = val;
    }

    return (idx > 0);
}

/***************************************************************************//**
* @brief Convert a string representing a hexadecimal long-integer to long-integer. 
*        The string must not contain leading or trailing spaces/tabs. 
*        It must end with '\0'. Only the characters are allowed: '0123456789ABCDEF'
* @param out_val [out] pointer to long-integer where converted value is stored
* @param in_str [in] string containing an long-integer (in hexadecimal representation)
* @param in_max_len [in] maximal length of in_str string
* @return true if string successfully converted to long-integer or false in case of error
*******************************************************************************/
bool utils_get_long_hex(long * out_val, const char * in_str, int in_max_len)
{
    long val = 0L;
    int idx = 0;

    while(idx < in_max_len)
    {
        if(*in_str == '\0')
            break;

        val <<= 4;
        if((*in_str >= '0') && (*in_str <= '9'))
            val += (long) (*in_str - '0');
        else if((*in_str >= 'A') && (*in_str <= 'F'))
            val += 10L + (long) (*in_str - 'A');
        else if((*in_str >= 'a') && (*in_str <= 'f'))
            val += 10L + (long) (*in_str - 'a');
        else
            return false;

        in_str++;
        idx++;
    }

    if((idx > 0) && (out_val != NULL))
    {
        *out_val = val;
    }

    return (idx > 0);
}

/***************************************************************************//**
* @brief Convert a string representing a long-integer (decimal or hexadecimal form) 
*        to long-integer. If the string contain a long-integer in a hexadecimal form, 
*        it must start with '0x'
* @param out_val [out] pointer to long-integer where converted value is stored
* @param in_str [in] string containing a long-integer (in decimal or hex representation)
* @param in_max_len [in] maximal length of in_str string
* @return true if string successfully converted to long-integer or false in case of error
*******************************************************************************/
bool utils_get_long(long * out_val, const char * in_str, int in_max_len)
{
    // If the string is less than 3 symbols it can be decoded only as decimal
    // (hexadecimal has '0x' which makes the length of a string min 3 symbols)
    if(in_max_len < 3)
        return utils_get_long_dec(out_val, in_str, in_max_len);

    // Hexadecimal format?
    if((in_str[0] == '0') && ((in_str[1] == 'x') || (in_str[1] == 'X')))
        return utils_get_long_hex(out_val, &in_str[2], in_max_len - 2);

    return utils_get_long_dec(out_val, in_str, in_max_len);
}
