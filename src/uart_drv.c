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

//******************************************************************************
// Includes
//******************************************************************************
#include <string.h>     // memcpy
#include <stdio.h>
//#include <stdarg.h>     //varg

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "uart_drv.h"
#include "gpio_drv.h"

#include "uart_ascii.h" //!!! TODO: delete

//******************************************************************************
// Function Prototypes
//******************************************************************************
void uart_drv_irq();

//******************************************************************************
// Global Variables
//******************************************************************************

// tx_buffer:
//
//       ... free space --->|<- data to send -->|<--- free space...
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  | | | | | | | | | | | | |X|X|X|X|X|X|X|X|X|X| | | | | | | | | | |
//  +-+-+-+-+-+-+-+-+-+-+-+-+^+-+-+-+-+-+-+-+-+-+^+-+-+-+-+-+-+-+-+-+
//   :                       |                   |                  :
//   0                   tx_rd_idx           tx_wr_idx              UART_TX_BUFF
//                     uart_drv_irq()      uart_drv_puts()
//
//
//      ...data to send --->|<--- free space --->|<-- data to send...
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |X|X|X|X|X|X|X|X|X|X|X|X| | | | | | | | | | |X|X|X|X|X|X|X|X|X|X|
//  +-+-+-+-+-+-+-+-+-+-+-+-+^+-+-+-+-+-+-+-+-+-+^+-+-+-+-+-+-+-+-+-+
//   :                       |                   |                  :
//   0                   tx_wr_idx           tx_rd_idx              UART_TX_BUFF
//                     uart_drv_puts()      uart_drv_irq()
//
static char tx_buffer[UART_TX_BUFF];
static volatile uint32_t tx_wr_idx = 0UL;
static volatile uint32_t tx_rd_idx = 0UL;
static volatile bool send_semaphore = false;

static char rx_buffer[UART_RX_BUFF];
static volatile uint32_t rx_wr_idx = 0UL;
static volatile uint32_t rx_rd_idx = 0UL;
static volatile uint32_t rx_wr_old = 0UL;
static volatile bool read_semaphore = false;

static char rx_buff0[16];
static volatile uint32_t rx_idx0 = 0UL;
static volatile bool rx_buff0_semaphore = false;

static uart_line_coding_t line_coding;

/*******************************************************************************
 * @brief UART Init function
 ******************************************************************************/
void uart_drv_init(const uart_line_coding_t * ptr_line_coding)
{
    memcpy(&line_coding, ptr_line_coding, sizeof(line_coding));

    //uart_deinit(UART_ID);
    //uart_get_hw(UART_ID)->dr = 0x00;

    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, line_coding.bit_rate);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, line_coding.data_bits, line_coding.stop_bits, line_coding.parity);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, uart_drv_irq);
    irq_set_priority(UART_IRQ, UART_IRQ_PRIO);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
    
    //!!! Send an empty character first, without this uart_drv_irq is not called on send. Why?
    uart_get_hw(UART_ID)->dr = 0x00;
}

/*******************************************************************************
 * @brief Detect jump of write pointer over read pointer (this leads to data lost)
 * and increment the read pointer ahead of write pointer.
 * 
 * Case 1 Negative  |                  |-------->|         |
 *  (no jump over)  |--------|---------|XXXXXXXXX|---------|
 *                  0     rd_idx     wr_old   wr_new    buff_len
 * 
 * Case 2 Positive  |        |------------------>|         |
 *   (jump over)    |--------|XXXXXXXXX|XXXXXXXXX|---------|
 *                  0     wr_old     rd_idx   wr_new    buff_len
 *
 * Case 3 Negative  |------>|                    |-------->|
 *  (no jump over)  |XXXXXXX|--------- |---------|XXXXXXXXX|
 *                  0    wr_new     rd_idx   wr_old    buff_len
 * 
 * Case 4 Positive  |---------------->|          |-------->|
 *   (jump over)    |XXXXXXX|XXXXXXXXX|----------|XXXXXXXXX|
 *                  0    rd_idx     wr_new    wr_old    buff_len
 *
 ******************************************************************************/
static inline void _check_wr_jumps_over_rd(void)
{
    if(((rx_wr_old < rx_rd_idx) && (rx_wr_idx >= rx_rd_idx))
    || ((rx_wr_idx < rx_wr_old) && (rx_rd_idx <= rx_wr_idx)))
    {
        // Jump of write pointer over read pointer detected, data lost
        rx_rd_idx = rx_wr_idx + 1;
        if(rx_rd_idx >= sizeof(rx_buffer))
            rx_rd_idx = 0UL;
        rx_wr_old = rx_wr_idx;
        UART_DRV_LOG("UART RX buffer overflow\r\n");
    }
}

/*******************************************************************************
 * @brief UART interrupt handler
 ******************************************************************************/
void uart_drv_irq()
{
    rx_wr_old = rx_wr_idx;

    // In case we have some data collected in rx backup buffer (rx_buff0), 
    // copy data to rx main buffer (if necessary overwrite old data)
    if((rx_idx0 > 0UL) && !rx_buff0_semaphore && !read_semaphore)
    {
        for(uint32_t i = 0UL; i < rx_idx0; i++)
        {
            if(rx_wr_idx >= sizeof(rx_buffer))
                rx_wr_idx = 0UL;
            rx_buffer[rx_wr_idx++] = rx_buff0[i];
        }
        rx_idx0 = 0UL;
        _check_wr_jumps_over_rd();
    }

    while (uart_is_readable(UART_ID)) 
    {
        char rx_ch = uart_getc(UART_ID);
        // Is the interrupt called while reading rx main buffer in uart_drv_get_rx?
        if(read_semaphore)
        {
            // Yes: Do not access rx main buffer to avoid corrupted data.
            // Instead store data in rx backup buffer (rx_buff0). Both 
            // read_semaphore and rx_buff0_semaphore cannot be set at the same 
            // time. If read_semaphore is set -> it is safe to access rx_buff0.
            if(rx_idx0 < sizeof(rx_buff0))
                rx_buff0[rx_idx0++] = rx_ch;
            else {
                // Rx backup buffer (rx_buff0) already full! Received byte lost!
                UART_DRV_LOG("UART RX buffer0 full\r\n");
            }
        }
        else{
            // No: It is safe to store data in rx main buffer.
            if(rx_wr_idx >= sizeof(rx_buffer))
                rx_wr_idx = 0UL;
            rx_buffer[rx_wr_idx++] = rx_ch;
            if(rx_wr_idx >= sizeof(rx_buffer))
                rx_wr_idx = 0UL;
            _check_wr_jumps_over_rd();
        }
        TP_TGL(TP4);
    }

    while(uart_is_writable(UART_ID))
    {
        // Is the interrupt called while in send function?
        if(send_semaphore)
        {
            // Stop sending and do not modify tx_rd_idx to avoid corrupted index.
            // Disable interrupt. The send-function which sets send_semaphore 
            // will re-enable the interrupt.
            uart_set_irq_enables(UART_ID, true, false);
            break;
        }
        // Nothing to send?
        else if(tx_wr_idx == tx_rd_idx)
        {
            // Disable tx irq
            uart_set_irq_enables(UART_ID, true, false);
            break;
        }
        // Safe to send the data
        else{
            uart_get_hw(UART_ID)->dr = (uint8_t) tx_buffer[tx_rd_idx++];
            if(tx_rd_idx >= sizeof(tx_buffer))
            {
                tx_rd_idx = 0UL;
            }
            TP_TGL(TP5);
        }
    }
}

/*******************************************************************************
 * @brief Send data to UART
 * @param buff [in] data to send to UART
 * @param len [in] count of bytes in buff to send to UART
 * @return count of copied bytes
 ******************************************************************************/
uint32_t uart_drv_send_buff(const uint8_t * buff, uint32_t len)
{
    if((buff == NULL) || (len == 0UL))
        return 0UL;

    send_semaphore = true;

    if(tx_wr_idx >= sizeof(tx_buffer))
        tx_wr_idx = 0UL;

    uint32_t available = 0UL;
    uint32_t copied = 0UL;
    uint32_t to_copy = len;

    //UART_DRV_LOG("%02x %3u | w%4i r%4i\r\n", buff[0], len, tx_wr_idx, tx_rd_idx);

    // |<------ free ----->|<--- busy for send --->|<------ free ----->|
    // |---- available ----|XXXXXXXXXXXXXXXXXXXXXXX|---- available ----|
    // 0                tx_rd_idx              tx_wr_idx       sizeof(tx_buffer)
    if(tx_wr_idx >= tx_rd_idx)
    {
        available = sizeof(tx_buffer) - tx_wr_idx;
        // tx_wr_idx should remain behind tx_rd_idx (otherwise we lose data)
        if((tx_rd_idx == 0) && (available > 0))
            available -= 1;

        if(to_copy > available)
            to_copy = available;
        if(to_copy > 0)
        {
            memcpy(&tx_buffer[tx_wr_idx], buff, to_copy);
            len -= to_copy;
            copied += to_copy;
            tx_wr_idx += to_copy;
            if(tx_wr_idx >= sizeof(tx_buffer))
                tx_wr_idx = 0UL;
        }

        //UART_DRV_LOG(">1%4i | w%4i r%4i\r\n", available, tx_wr_idx, tx_rd_idx);
    }

    to_copy = len;

    // |<-- busy for send -->|<------ free ----->|<-- busy for send -->|
    // |XXXXXXXXXXXXXXXXXXXXX|---- available ----|XXXXXXXXXXXXXXXXXXXXX|
    // 0                tx_wr_idx              tx_rd_idx       sizeof(tx_buffer)
    if((len > 0UL) && (tx_wr_idx < tx_rd_idx))
    {
        available = tx_rd_idx - tx_wr_idx;
        // tx_wr_idx should remain behind tx_rd_idx (otherwise we lose data)
        available -= 1;

        if(to_copy > available)
            to_copy = available;
        if(to_copy > 0)
        {
            memcpy(&tx_buffer[tx_wr_idx], &buff[copied], to_copy);
            copied += to_copy;
            tx_wr_idx += to_copy;
            if(tx_wr_idx >= sizeof(tx_buffer))
                tx_wr_idx = 0UL;
        }

        //UART_DRV_LOG(">2%4i | w%4i r%4i\r\n", available, tx_wr_idx, tx_rd_idx);
    }

    send_semaphore = false;

    // Reactivate IRQ (if not already active)
    uart_set_irq_enables(UART_ID, true, true);

    return copied;
}

/*******************************************************************************
 * @brief Check if UART interrupt is active
 * @return true if UART interrupt is active
 ******************************************************************************/
bool uart_drv_check_irq(void)
{
    return ((uart_get_hw(UART_ID)->ris & 0x00000020) != 0);
}

/*******************************************************************************
 * @brief Check if there are received characters and copy them to buffer
 * @param buff [out] pointer to buffer where the received characters are copied
 * @param buff_max_len [in] the length in bytes (characters) of buff
 * @return count of copied received characters
 ******************************************************************************/
uint32_t uart_drv_get_rx(char * buff, uint32_t buff_max_len)
{
    uint32_t buff_idx = 0UL;

    if((buff == NULL) || (buff_max_len == 0UL))
        return 0UL;

    // Check first if new data available in rx backup buffer (rx_buff0)?
    if(rx_idx0 > 0UL)
    {
        // Start semaphore, protect rx_buff0
        rx_buff0_semaphore = true;

        // Copy as much as possible from 
        if(rx_idx0 > buff_max_len)
            rx_idx0 = buff_max_len;
        memcpy(buff, rx_buff0, rx_idx0);
        buff_max_len -= rx_idx0;
        buff_idx += rx_idx0;
        rx_idx0 = 0UL;

        rx_buff0_semaphore = false;
    }

    // New data available?
    if(rx_rd_idx != rx_wr_idx)
    {
        // Start semaphore: protect: rx_rd_idx, rx_wr_idx, rx_buffer
        read_semaphore = true;

        if(rx_rd_idx >= sizeof(rx_buffer))
            rx_rd_idx = 0UL;

        // |............|............|//////////////|
        // |            |            |<--- copy --->|
        // 0        rx_wr_idx    rx_rd_idx     sizeof(rx_buffer)
        if(rx_rd_idx > rx_wr_idx)
        {
            uint32_t diff = sizeof(rx_buffer) - rx_rd_idx;
            if(diff > buff_max_len)
                diff = buff_max_len;

            if(diff > 0UL)
            {
                memcpy(&buff[buff_idx], &rx_buffer[rx_rd_idx], diff);
                buff_max_len -= diff;
                buff_idx += diff;
                rx_rd_idx += diff;
                if(rx_rd_idx >= sizeof(rx_buffer))
                    rx_rd_idx = 0UL;
            }
        }

        // |..........|//////////////|.............|
        // |          |<--- copy --->|             |
        // 0       rx_rd_idx      rx_wr_idx    sizeof(rx_buffer)
        if(rx_rd_idx < rx_wr_idx)
        {
            uint32_t diff = rx_wr_idx - rx_rd_idx;
            if(diff > buff_max_len)
                diff = buff_max_len;

            if(diff > 0UL)
            {
                memcpy(&buff[buff_idx], &rx_buffer[rx_rd_idx], diff);
                buff_max_len -= diff;
                buff_idx += diff;
                rx_rd_idx += diff;
                if(rx_rd_idx >= sizeof(rx_buffer))
                    rx_rd_idx = 0UL;
            }
        }

        read_semaphore = false;
    }
    return buff_idx;
}
