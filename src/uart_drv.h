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
 * uart_drv - the UART communication driver (implemented in non-blocking mode,
 * not multi-thread safe)
 ******************************************************************************/
#ifndef UART_DRV_H
#define UART_DRV_H

//******************************************************************************
// Includes
//******************************************************************************

#include "hardware/uart.h"

#ifdef UART_DRV_DEBUG
#include DEBUG_INCLUDE
#endif

//******************************************************************************
// Defines
//******************************************************************************
#define UART_TX_PIN     0
#define UART_RX_PIN     1

#define UART_ID         uart0
#define UART_IRQ_PRIO   PICO_DEFAULT_IRQ_PRIORITY
#define UART_TX_BUFF    256
#define UART_RX_BUFF    256

#ifdef UART_DRV_DEBUG
    #define UART_DRV_LOG(...)   DEBUG_PRINTF(__VA_ARGS__)
#else
    #define UART_DRV_LOG(...)    
#endif  

typedef struct {
    unsigned int bit_rate;
    unsigned int stop_bits; // 1, 2
    uart_parity_t parity;   // UART_PARITY_NONE (0), UART_PARITY_EVEN (1), UART_PARITY_ODD (2)
    unsigned int data_bits; // 5, 6, 7, 8 or 16
} uart_line_coding_t;

//******************************************************************************
// Exported Functions
//******************************************************************************

// UART Init function
void uart_drv_init(const uart_line_coding_t * ptr_line_coding);

// Send buff to UART
uint32_t uart_drv_send_buff(const uint8_t * buff, uint32_t len);

// Get the free size of Tx buffer
uint32_t uart_drv_get_tx_free_cnt(void);

// Check if there are received characters and copy them to buffer
uint32_t uart_drv_get_rx(char * buff, uint32_t buff_max_len);

//******************************************************************************
#endif /* UART_DRV_H */
