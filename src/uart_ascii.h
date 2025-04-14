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
 * uart_ascii - the UART communication driver for text information exchange 
 * (implemented in non-blocking mode, not multi-thread safe)
 ******************************************************************************/
#ifndef UART_ASCII_H
#define UART_ASCII_H

//******************************************************************************
// Includes
//******************************************************************************

//******************************************************************************
// Defines
//******************************************************************************
#define UART_ASCII_TX_PIN   8
#define UART_ASCII_RX_PIN   9

#define UART_ASCII_ID       uart1
#define UART_ASCII_IRQ_PRIO PICO_LOWEST_IRQ_PRIORITY
#define UART_ASCII_BR       1000000
#define UART_ASCII_TX_BUFF  65355
#define UART_ASCII_RX_BUFF  128

#define UART_ASCII_DBG_LVL  0

//******************************************************************************
// Exported Functions
//******************************************************************************

// UART Init function
void uart_ascii_init(void);

// Write text to UART
void uart_ascii_puts(const char * txt);

// Write hex byte to UART
void uart_ascii_puth8(unsigned char val);

// Write formatted data from variable argument list to UART
int uart_ascii_printf0(const char* format, ...);

// Write formatted data from variable argument list to UART (using UART_DBG_LVL)
int uart_ascii_printf(const char* format, ...);

// Send to UART a buffer of data in a hexadecimal 'dump' format
void uart_ascii_dump(const void * ptr_buffer, int len, unsigned long addr);

// Check if UART interrupt is active
bool uart_ascii_check_irq(void);

// Check if there are received characters and copy them to buffer
int uart_ascii_get_rx(char * buff, int buff_max_len);

//******************************************************************************
#endif /* UART_ASCII_H */
