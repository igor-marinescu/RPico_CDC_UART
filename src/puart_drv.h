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
 * puart_drv - the PIO-UART communication driver (implemented in non-blocking mode,
 * not multi-thread safe)
 ******************************************************************************/
#ifndef PUART_DRV_H
#define PUART_DRV_H

//******************************************************************************
// Includes
//******************************************************************************

#ifdef PUART_DRV_DEBUG
#include DEBUG_INCLUDE
#endif

//******************************************************************************
// Defines
//******************************************************************************
#define PUART_TX_PIN     0
#define PUART_RX_PIN     1

#define PUART_IRQ_PRIO   PICO_DEFAULT_IRQ_PRIORITY
#define PUART_TX_BUFF    256
#define PUART_RX_BUFF    256

#ifdef PUART_DRV_DEBUG
    #define PUART_DRV_LOG(...)   DEBUG_PRINTF(__VA_ARGS__)
#else
    #define PUART_DRV_LOG(...)    
#endif  

//******************************************************************************
// Exported Functions
//******************************************************************************

// UART Init function
void puart_drv_init(void);

// Send buff to UART
uint32_t puart_drv_send_buff(const uint8_t * buff, uint32_t len);

// Get the free size of Tx buffer
uint32_t puart_drv_get_tx_free_cnt(void);

// Check if PIO-UART has finished transmiting data and reset TX_ACTIVE signal
void puart_drv_control_tx_active(void);

// Check if there are received characters and copy them to buffer
uint32_t puart_drv_get_rx(char * buff, uint32_t buff_max_len);

//******************************************************************************
#endif /* PUART_DRV_H */
