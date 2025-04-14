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
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "uart_ascii.h"
#include "gpio_drv.h"


//******************************************************************************
// Function Prototypes
//******************************************************************************
void uart_ascii_irq();

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
static char tx_buffer[UART_ASCII_TX_BUFF];
static volatile int tx_wr_idx = 0;
static volatile int tx_rd_idx = 0;
static volatile bool send_semaphore = false;

// rx_buffer:
static char rx_buffer[UART_ASCII_RX_BUFF];
static char rx_buff_0[UART_ASCII_RX_BUFF];
static volatile int rx_idx = 0;
static volatile int rx_len = 0;

#if UART_ASCII_DBG_LVL > 0
static uint16_t dbg_idx = 0;
#endif

/*******************************************************************************
 * @brief UART-Ascii Init function
 ******************************************************************************/
void uart_ascii_init(void)
{
    // Set up our UART with a basic baud rate.
    uart_init(UART_ASCII_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_ASCII_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_ASCII_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ASCII_ID, UART_ASCII_BR);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ASCII_ID, false, false);
    uart_set_format(UART_ASCII_ID, /*DATA_BITS*/ 8, /*STOP_BITS*/ 1, /*PARITY*/ UART_PARITY_NONE);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ASCII_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = (UART_ASCII_ID == uart0) ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, uart_ascii_irq);
    irq_set_priority(UART_IRQ, UART_ASCII_IRQ_PRIO);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ASCII_ID, true, false);
    
    //!!! Send an empty character first, without this uart_drv_irq is not called on send. Why?
    uart_puts(UART_ASCII_ID, " ");
}

/*******************************************************************************
 * @brief UART interrupt handler
 ******************************************************************************/
void uart_ascii_irq()
{
    bool send_echo = false;
    char rx_ch = '\0';

    while (uart_is_readable(UART_ASCII_ID)) 
    {
        rx_ch = uart_getc(UART_ASCII_ID);
        // Ctrl+C (0x03) or ESC (0x1B) received?
        if((rx_ch == (char) 0x03) || (rx_ch == (char) 0x1B))
        {
           rx_idx = 0; 
           rx_ch = '\r';
        }
        // Backspace?
        else if(rx_ch == (char) 0x7F)
        {
            if(rx_idx > 0)
                rx_idx--;
        }
        // If new line received, copy the entire received text to rx_buffer
        else if((rx_ch == '\r') || (rx_ch == '\n'))
        {
            if(rx_idx > 0)
            {
                if(rx_idx > UART_ASCII_RX_BUFF)
                    rx_idx = UART_ASCII_RX_BUFF;
                memcpy(rx_buffer, rx_buff_0, rx_idx);
                if(rx_idx < UART_ASCII_RX_BUFF)
                {
                    rx_buffer[rx_idx] = '\0';
                    rx_idx++;
                }
                rx_len = rx_idx;
                rx_idx = 0;
            }
        }
        else if(rx_idx < UART_ASCII_RX_BUFF)
        {
            rx_buff_0[rx_idx++] = rx_ch;
        }
        send_echo = true;
    }
    while(uart_is_writable(UART_ASCII_ID)){
        // Is the interrupt called while in send function?
        if(send_semaphore)
        {
            // In this case send a dummy character 
            // (do not modify tx_rd_idx to avoid corrupted index)
            uart_get_hw(UART_ASCII_ID)->dr = (uint8_t) '\0'; //'.';
            return;
        }
        // Nothing to send?
        if(tx_wr_idx == tx_rd_idx)
        {
            if(send_echo)
                // Send echo?
                uart_get_hw(UART_ASCII_ID)->dr = rx_ch;
            else
                // Still nothing to send? Disable tx irq
                uart_set_irq_enables(UART_ASCII_ID, true, false);
            break;
        }
        // Data to send
        else{
            uart_get_hw(UART_ASCII_ID)->dr = (uint8_t) tx_buffer[tx_rd_idx++];
            if(tx_rd_idx >= UART_ASCII_TX_BUFF){
                tx_rd_idx = 0;
            }
        }
    }
}

/*******************************************************************************
 * @brief Write text to UART
 * @param txt [in] string to write to UART
 ******************************************************************************/
void uart_ascii_puts(const char * txt)
{
    if(txt == NULL)
        return;

    int diff = 0;
    int tx_len = strlen(txt);
    if(tx_len <= 0)
        return;

    if(tx_len > UART_ASCII_TX_BUFF)
        tx_len = UART_ASCII_TX_BUFF;

    send_semaphore = true;

    int tx_wr_idx_old = tx_wr_idx;

    if((tx_wr_idx < 0) || (tx_wr_idx >= UART_ASCII_TX_BUFF))
        tx_wr_idx = 0;

    if((tx_wr_idx + tx_len) > UART_ASCII_TX_BUFF)
    {
        diff = (UART_ASCII_TX_BUFF - tx_wr_idx);
        memcpy(&tx_buffer[tx_wr_idx], txt, diff);
        tx_len -= diff;
        tx_wr_idx = 0;
    }

    if(tx_len > 0)
    {
        memcpy(&tx_buffer[tx_wr_idx], &txt[diff], tx_len);
        tx_wr_idx += tx_len;
    }

    if(tx_wr_idx >= UART_ASCII_TX_BUFF)
        tx_wr_idx = 0;

    // If while copying the new data, the wr-index "jumps over" rd-index
    // (overwriting some of not yet sent old data), move the rd-index as well 
    // (ignore the old lost data, and display a '~' symbol).
    if((tx_wr_idx_old < tx_rd_idx) && (tx_wr_idx > tx_rd_idx))
    {
        tx_rd_idx = tx_wr_idx + 1;
        if((tx_rd_idx < 0) || (tx_rd_idx >= UART_ASCII_TX_BUFF))
            tx_rd_idx = 0;
        tx_buffer[tx_rd_idx] = '~';
    }

    send_semaphore = false;

    uart_set_irq_enables(UART_ASCII_ID, true, true);
}

/*******************************************************************************
 * @brief Get the ascii-hex representation of the low part of a byte (bits 3..0)
 * @param [in] digit - byte to be represented as ascii-hex
 * @return ascii-hex representation of the lower part of the byte
 ******************************************************************************/
static inline char hex_digit(unsigned char digit)
{
    digit &= 0x0F;
    if(digit < 10)
        return ('0' + (char) digit);
    return ('A' + (char)(digit - 10));
}

/*******************************************************************************
 * @brief Write hex byte to UART
 * @param [in] val - byte to write to UART in hex
 ******************************************************************************/
void uart_ascii_puth8(unsigned char val)
{
    char txt[3];
    txt[0] = hex_digit(val >> 4);
    txt[1] = hex_digit(val);
    txt[2] = '\0';
    uart_ascii_puts(txt);
}

/*******************************************************************************
 * @brief Write formatted data from variable argument list to UART
 * @param format [in] string that contains the format string (see printf)
 * @return the number of characters sent to UART
 ******************************************************************************/
int uart_ascii_printf0(const char* format, ...)
{
    char temp_buff[128];

    va_list argp;
    va_start(argp, format);
    int res = vsnprintf(temp_buff, sizeof(temp_buff), format, argp);
    va_end(argp);

    uart_ascii_puts(temp_buff);
    return res;
}

/*******************************************************************************
 * @brief Write formatted data from variable argument list to UART
 *        The output text can be formatted using UART_DBG_LVL
 * @param format [in] string that contains the format string (see printf)
 * @return the number of characters sent to UART
 ******************************************************************************/
int uart_ascii_printf(const char* format, ...)
{
    char temp_buff[128];
    int off = 0;

#if UART_ASCII_DBG_LVL == 1
    // Format: "[<dbg_idx>]text..."
    off = sprintf(temp_buff, "[%04X]", dbg_idx++);
    if(off < 0)
        off = 0;
#elif UART_ASCII_DBG_LVL == 2
    // Format: "[<dbg_idx> <tx_wr_idx> <tx_rd_idx>]text..."
    off = sprintf(temp_buff, "[%04X %.5i %.5i]", dbg_idx++, tx_wr_idx, tx_rd_idx);
    if(off < 0)
        off = 0;
#endif

    va_list argp;
    va_start(argp, format);
    int res = vsnprintf(&temp_buff[off], sizeof(temp_buff) - off, format, argp);
    va_end(argp);

    uart_ascii_puts(temp_buff);
    return res;
}

/*******************************************************************************
 * @brief Send to UART a buffer of data in a hexadecimal 'dump' format
 * @param ptr_buffer [in] - pointer to the buffer to display as dump
 * @param len [in] - length of the buffer (in bytes)
 * @param addr [in] - address offset of data to be displayed
 ******************************************************************************/
void uart_ascii_dump(const void * ptr_buffer, int len, unsigned long addr)
{
    int i, j;
    const uint8_t * ptr = (const uint8_t *) ptr_buffer;

    for(i = 0; i < len;)
    {
        uart_ascii_printf0("%08lx  ", addr);

        for(j = 0; j < (int) (addr % 16); j++)
        {
            uart_ascii_puts(".. ");
        }

        for(; (j < 16) && (i < len); j++, i++)
        {
            uart_ascii_printf0("%02hhx ", *ptr);
            ptr++;
            addr++;
        }

        uart_ascii_puts("\r\n");
    }
}

/*******************************************************************************
 * @brief Check if UART interrupt is active
 * @return true if UART interrupt is active
 ******************************************************************************/
bool uart_ascii_check_irq(void)
{
    return ((uart_get_hw(UART_ASCII_ID)->ris & 0x00000020) != 0);
}

/*******************************************************************************
 * @brief Check if there are received characters and copy them to buffer
 * @param buff [out] pointer to buffer where the received characters are copied
 * @param buff_max_len [in] the length in bytes (characters) of buff
 * @return count of copied received characters
 ******************************************************************************/
int uart_ascii_get_rx(char * buff, int buff_max_len)
{
    if(rx_len <= 0)
        return 0;

    if(rx_len < buff_max_len)
        buff_max_len = rx_len;

    memcpy(buff, rx_buffer, buff_max_len);

    rx_len = 0;
    return buff_max_len;
}
