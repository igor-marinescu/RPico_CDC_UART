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
 *
 * Inspired/used as example from tinyusb.org: 
 * Copyright (c) 2019 Ha Thach / The MIT License (MIT)
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "usb_main.h"
#include "bsp/board_api.h"
#include "pico/stdlib.h"
#include "pico/sync.h"

#include "gpio_drv.h"

//******************************************************************************
// Defines
//******************************************************************************

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};

#define USB_MAIN_BUFF_LEN   1024

//******************************************************************************
// Function Prototypes
//******************************************************************************
static void led_blinking_task(void);
static void cdc_task(void);

//******************************************************************************
// Global Variables
//******************************************************************************
static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

// Variables used for received data
static uint8_t rx_data[USB_MAIN_BUFF_LEN];
static uint32_t rx_cnt = 0;
static struct mutex rx_mutex;

// Variables used for data transmission
static uint8_t tx_data[USB_MAIN_BUFF_LEN];
static uint32_t tx_cnt = 0;
static struct mutex tx_mutex;

// Variables used to line coding handling
static cdc_line_coding_t line_coding;
static uint32_t line_coding_changed = 0;
static struct mutex lc_mutex;

//******************************************************************************
// echo to either Serial0 or Serial1
// with Serial0 as all lower case, Serial1 as all upper case
//******************************************************************************
static void echo_serial_port(uint8_t itf, uint8_t buf[], uint32_t count)
{

    for(uint32_t i = 0; i < count; i++)
    {
        buf[i] = ~buf[i];
        tud_cdc_n_write_char(itf, buf[i]);
    }
    tud_cdc_n_write_flush(itf);
}

//******************************************************************************
// Device callbacks
//******************************************************************************

//******************************************************************************
// Invoked when device is mounted
//******************************************************************************
void tud_mount_cb(void)
{
    blink_interval_ms = BLINK_MOUNTED;
}

//******************************************************************************
// Invoked when device is unmounted
//******************************************************************************
void tud_umount_cb(void)
{
    blink_interval_ms = BLINK_NOT_MOUNTED;
}

//******************************************************************************
// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
//******************************************************************************
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

//******************************************************************************
// Invoked when usb bus is resumed
//******************************************************************************
void tud_resume_cb(void)
{
    blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//******************************************************************************
// Invoked when cdc when line state changed e.g connected/disconnected
// Invoked when line state DTR & RTS are changed via SET_CONTROL_LINE_STATE
// Use to reset to DFU when disconnect with 1200 bps
//******************************************************************************
void tud_cdc_line_state_cb(uint8_t instance, bool dtr, bool rts)
{
    (void)rts;

    // DTR = false is counted as disconnected
    if(!dtr)
    {
        // touch1200 only with first CDC instance (Serial)
        if(instance == 0)
        {
            cdc_line_coding_t coding;
            tud_cdc_get_line_coding(&coding);
            if(coding.bit_rate == 1200)
            {
                if(board_reset_to_bootloader)
                {
                    board_reset_to_bootloader();
                }
            }
        }
    }
}

/*******************************************************************************
 * @brief Invoked when line coding is change via SET_LINE_CODING
 * @param itf - interface index
 * @param p_line_coding - pointer to line code structure
 *      typedef struct TU_ATTR_PACKED
 *      {
 *          uint32_t bit_rate;
 *          uint8_t  stop_bits; ///< 0: 1 stop bit - 1: 1.5 stop bits - 2: 2 stop bits
 *          uint8_t  parity;    ///< 0: None - 1: Odd - 2: Even - 3: Mark - 4: Space
 *          uint8_t  data_bits; ///< can be 5, 6, 7, 8 or 16
 *      } cdc_line_coding_t; 
 * 
 ******************************************************************************/
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding)
{
    if(itf == 0)
    {
        mutex_enter_blocking(&lc_mutex);

        memcpy(&line_coding, p_line_coding, sizeof(line_coding));
        line_coding_changed++;

        mutex_exit(&lc_mutex);
    }
}

//******************************************************************************
uint32_t usb_get_rx(uint8_t * buff, uint32_t buff_len)
{
    // New received data available?
    // First check without locking the buffer
    if(rx_cnt == 0ul)
        return 0ul;

    uint32_t cnt = 0ul;

    // Try to lock the buffer
    if(mutex_try_enter(&rx_mutex, NULL))
    {
        // New received data available?
        if(rx_cnt > 0)
        {
            cnt = rx_cnt;
            if(cnt > buff_len)
                cnt = buff_len;

            memcpy(buff, rx_data, cnt);

            // Move the remaining data to the beginning of buffer:
            //  rx_data:
            // |XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX..............|
            // |<------ cnt ----->|<-- rx_cnt-cnt -->|              |
            // |<-------------- rx_cnt ------------->|              |
            // 0                                            USB_MAIN_BUFF_LEN
            //
            //  rx_data:
            // |XXXXXXXXXXXXXXXXXXX.................................|
            // |<-- rx_cnt-cnt -->|                                 |
            // |<---- rx_cnt ---->|                                 |
            // 0                                            USB_MAIN_BUFF_LEN
            // 
            if(cnt < rx_cnt)
            {
                memmove(&rx_data[0], &rx_data[cnt],  rx_cnt - cnt);
            }
            rx_cnt -= cnt;
        }

        // Unlock the buffer
        mutex_exit(&rx_mutex);
    }    
    return cnt;
}

//******************************************************************************
uint32_t usb_set_tx(uint8_t * buff, uint32_t buff_len)
{
    uint32_t cnt = 0ul;

    if((buff_len == 0) || (buff == NULL))
        return 0UL;

    // Block the code until buffer locked
    //mutex_enter_blocking (&tx_mutex)

    // Try to lock the buffer
    if(mutex_try_enter(&tx_mutex, NULL))
    {
        if(tx_cnt > sizeof(tx_data))
            tx_cnt = sizeof(tx_data);

        uint32_t available = sizeof(tx_data) - tx_cnt;
        if(buff_len > available)
            buff_len = available;

        if(buff_len > 0)
        {
            // Append data to send at the end of tx_buffer
            memcpy(&tx_data[tx_cnt], buff, buff_len);
            tx_cnt += buff_len;
        }

        // Unlock the buffer
        mutex_exit(&tx_mutex);
    }
    else buff_len = 0UL;

    return buff_len;
}

//******************************************************************************
static void cdc_task(void)
{

    // connected() check for DTR bit
    // Most but not all terminal client set this when making connection
    // if( tud_cdc_n_connected(itf) )
    {
        // Data received?
        if(tud_cdc_n_available(0))
        {
            // Try to lock the buffer
            if(mutex_try_enter(&rx_mutex, NULL))
            {
                if(rx_cnt < USB_MAIN_BUFF_LEN)
                {
                    TP_TGL(TP1);
                    uint32_t space = USB_MAIN_BUFF_LEN - rx_cnt;
                    uint32_t count = tud_cdc_n_read(0, &rx_data[rx_cnt], space);
                    if(count <= space)
                        rx_cnt += count;
                    else
                        rx_cnt = USB_MAIN_BUFF_LEN;
                }

                // Unlock the buffer
                mutex_exit(&rx_mutex);
            }

            // echo back
            //echo_serial_port(0, buf, count);
        }

        // Data to send? (first check without locking the tx variables)
        if(tx_cnt != 0)
        {
            // Try to lock the buffer
            if(mutex_try_enter(&tx_mutex, NULL))
            {
                if(tx_cnt > 0)
                {
                    if(tx_cnt > sizeof(tx_data))
                        tx_cnt = sizeof(tx_data);

                    // Return the number of bytes (characters) available for writing 
                    // to TX FIFO buffer in a single n_write operation.
                    uint32_t available_cnt = tud_cdc_n_write_available(0);
                    uint32_t written_cnt = 0UL;

                    if(available_cnt > tx_cnt)
                        available_cnt = tx_cnt;

                    if(available_cnt > 0)
                    {
                        // Write bytes to TX FIFO, return Number of written elements
                        written_cnt = tud_cdc_n_write(0, tx_data, available_cnt);
                        if(written_cnt > 0)
                        {
                            TP_TGL(TP2);
                            // In case we wrote less than we wanted, move the remaining 
                            // data to send to the left (to be sent next time)
                            if(written_cnt < tx_cnt)
                            {
                                memmove(&tx_data[0], &tx_data[written_cnt],  tx_cnt - written_cnt);
                                tx_cnt -= written_cnt;
                            }
                            else tx_cnt = 0UL;

                            tud_cdc_n_write_flush(0);
                        }
                    }
                }

                // Unlock the buffer
                mutex_exit(&tx_mutex);
            }
        }
    }
}

//******************************************************************************
void led_blinking_task(void)
{
    static uint32_t start_ms = 0;
    static bool led_state = false;

    // Blink every interval ms
    if(board_millis() - start_ms < blink_interval_ms)
        return; // not enough time

    start_ms += blink_interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}

/*******************************************************************************
 * @brief USB main task - running on separate kern
 ******************************************************************************/
void usb_main(void)
{
    mutex_init(&rx_mutex);
    mutex_init(&tx_mutex);
    mutex_init(&lc_mutex);

    while (1)
    {
        tud_task();         // tinyusb device task
        cdc_task();
        led_blinking_task();
        TP_TGL(TP0);
    }
}

/*******************************************************************************
 * @brief Return the count of line code changes - used to detect when a new 
 *          line coding settings are available.
 * @return the count of line code changes
 ******************************************************************************/
uint32_t usb_has_line_coding_changed(void)
{
    return line_coding_changed;
}

/*******************************************************************************
 * @brief Copy the currently used line code settings
 * @param [in] ptr_line_coding - pointer to destination line code structure
 * @return the count of line code changes
 ******************************************************************************/
uint32_t usb_get_line_coding(cdc_line_coding_t * ptr_line_coding)
{
    uint32_t changes_cnt;

    mutex_enter_blocking(&lc_mutex);

    memcpy(ptr_line_coding, &line_coding, sizeof(line_coding));
    changes_cnt = line_coding_changed;

    mutex_exit(&lc_mutex);
    return changes_cnt;
}
