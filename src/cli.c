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
 * cli - Command Line Interface Module
 * CLI is used for testing and diagnostic purposes.
 * It uses in_out module for the input-output stream. It checks if there is
 * line of text received, splits it into words, gets the command, tries to find 
 * the coresponding function defined in cli_func, executes it and outputs 
 * back the result.
 ******************************************************************************/

//******************************************************************************
// Includes
//******************************************************************************
#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "cli.h"
#include "uart_ascii.h"

//******************************************************************************
// Typedefs
//******************************************************************************
typedef struct {
    const char * cmd;       // Command name
    const char * opt;       // Option name
    cli_func_t func;        // Function
    const char * info;      // Command info
} cli_cmd_t;

//******************************************************************************
// Global Variables
//******************************************************************************

static cli_cmd_t cli_func_arr[CLI_FUNC_CNT];

static char rx_buff[CLI_BUFF_SIZE];
static char rx_words[CLI_WORD_CNT][CLI_WORD_SIZE];
static char * rx_args[CLI_WORD_CNT];

/***************************************************************************//**
* @brief Init CLI module
*******************************************************************************/
void cli_init(void)
{
    memset(cli_func_arr, 0, sizeof(cli_func_arr));

    // Build args
    for(int i = 0; i < CLI_WORD_CNT; i++)
        rx_args[i] = rx_words[i];
}

/*******************************************************************************
 * @brief Extract single words from a buffer of text and populate rx_words.
 * @param in_buff [in] - pointer to text buffer from where to extract the words
 * @param len [in] - length of the text buffer in_buff
 * @return count of extracted words
 ******************************************************************************/
static int rx_extract_words(const char * in_buff, int len)
{
    int idx1, idx2, word;
    char ch;
    
    idx1 = 0;
    word = 0;

    memset(rx_words, 0, sizeof(rx_words));
    
    while((idx1 < len) && (word < CLI_WORD_CNT) && (in_buff[idx1] != '\0'))
    {
        // Ignore whitespaces at the beginning
        while((idx1 < len) && ((in_buff[idx1] == ' ') || (in_buff[idx1] == '\t'))) idx1++;

        // Extract word
        idx2 = 0;
        while((idx1 < len) && (in_buff[idx1] != ' ') && (in_buff[idx1] != '\t') && (in_buff[idx1] != '\0'))
        {
            if(idx2 < (CLI_WORD_SIZE - 1))
            {
                // Copy every character converting it to lowercase
                ch = in_buff[idx1];
                if((ch >= 'A') && (ch <= 'Z'))
                    ch += 0x20;
                rx_words[word][idx2] = ch;
                idx2++;
            }
            idx1++;
        }
        
        if(idx2 > 0)
        {
            rx_words[word][idx2] = '\0';
            word++;
        }
    }

    return word;
}

/*******************************************************************************
 * @brief Search the function in the list of functions that satisfies conditions:
 *         1. if command definition has an option (opt != NULL) check if both:
 *            (rx_words[0] == cmd) and (rx_words[1] == opt)
 *         2. if no command satisfies the 1 condition return the "default"
 *            command with no option: (rx_words[0] == cmd) and (opt != NULL)
 *            if there is such a command.
 * @return pointer to the command found or NULL in case no command found
 ******************************************************************************/
static const cli_cmd_t * search_func(void)
{
    const cli_cmd_t * def_cmd_ptr = NULL;

    for(int i = 0; i < CLI_FUNC_CNT; i++)
    {
        if(cli_func_arr[i].cmd == NULL)
            continue;

        if(!strncmp(rx_words[0], cli_func_arr[i].cmd, CLI_WORD_SIZE))
        {
            // We found default command (without option)
            if(cli_func_arr[i].opt == NULL)
                def_cmd_ptr = &cli_func_arr[i];

#if CLI_WORD_CNT >= 2
            // Both, command and option satisfies the condition
            else if(!strncmp(rx_words[1], cli_func_arr[i].opt, CLI_WORD_SIZE))
                return &cli_func_arr[i];
#endif
        }
    }

    return def_cmd_ptr;
}

/*******************************************************************************
 * @brief Poll CLI module, must be called every program cycle
 ******************************************************************************/
void cli_poll(void)
{
    // Something received?
    int len = uart_ascii_get_rx(rx_buff, CLI_BUFF_SIZE);
    if(len <= 0)
        return;

    // Extract single words from received text
    rx_buff[CLI_BUFF_SIZE - 1] = '\0';
    //io_printf("\r\n received: >%s< len=%i\r\n", rx_buff, len);
    int argc = rx_extract_words(rx_buff, len);
    uart_ascii_puts("\r\n");

    if(argc <= 0)
        return;

    // Based on extracted words look for matching command in lits of functions
    const cli_cmd_t * cmd_ptr = search_func();
    if(cmd_ptr != NULL)
    {
        if(cmd_ptr->func != NULL)
        {
            if(!cmd_ptr->func(argc, rx_args))
                uart_ascii_printf0("Invalid command arguments. Expected: %s\r\n", cmd_ptr->info);
        }
    }    
    else{
        uart_ascii_puts("Command not found.\r\n");
    }
}

/*******************************************************************************
 * @brief Add function
 * @param cmd [in] name of the command to add
 * @param opt [in] name of the comands option
 * @param func [in] function pointer name of the command to add
 * @param info [in] information name of the command to add
 * @return true if command successfully added or false in case there is no free space
 ******************************************************************************/
bool cli_add_func(const char * cmd, const char * opt, cli_func_t func, const char * info)
{
    // Find first empty slot and populate command
    for(int i = 0; i < CLI_FUNC_CNT; i++)
    {
        if(cli_func_arr[i].cmd == NULL)
        {
            cli_func_arr[i].cmd = cmd;
            cli_func_arr[i].opt = opt;
            cli_func_arr[i].func = func;
            cli_func_arr[i].info = info;
            return true;
        }
    }
    return false;
}

/*******************************************************************************
 * @brief Display the list of all functions/commands and their info
 ******************************************************************************/
void cli_func_list(void)
{
    for(int i = 0; i < CLI_FUNC_CNT; i++)
    {
        if(cli_func_arr[i].cmd != NULL)
        {
            if(cli_func_arr[i].info != NULL)
                uart_ascii_puts(cli_func_arr[i].info);
            else
                uart_ascii_puts(cli_func_arr[i].cmd);
            uart_ascii_puts("\r\n");
        }
    }
}
