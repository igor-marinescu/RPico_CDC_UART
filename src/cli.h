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
#ifndef CLI_H
#define CLI_H

//******************************************************************************
// Includes
//******************************************************************************

//******************************************************************************
// Defines
//******************************************************************************
#define CLI_BUFF_SIZE   128
#define CLI_WORD_SIZE   32
#define CLI_WORD_CNT    6
#define CLI_FUNC_CNT    20

//******************************************************************************
// Typedefs
//******************************************************************************
typedef bool (*cli_func_t)(int argc, char ** args);

//******************************************************************************
// Exported Variables
//******************************************************************************

//******************************************************************************
// Exported Functions
//******************************************************************************

// Init CLI module
void cli_init(void);

// Poll CLI module, must be called every program cycle
void cli_poll(void);

// Add function
bool cli_add_func(const char * cmd, const char * opt, cli_func_t func, const char * info);

// Display the list of all functions/commands and their info
void cli_func_list(void);

//******************************************************************************
#endif /* CLI_H */
