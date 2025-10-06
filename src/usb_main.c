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
#include "pico/bootrom.h"

//******************************************************************************
// Defines
//******************************************************************************

typedef enum {
    usb_status_not_mounted = 0,
    usb_status_mounted,
    usb_status_suspended,
} usb_status_t;

#define USB_MAIN_BUFF_LEN   1024
#define USB_LED_RXTX_ACTIVITY_TOUT  5

//******************************************************************************
// Function Prototypes
//******************************************************************************
static void led_blinking_task(void);
static void cdc_task(void);

//******************************************************************************
// Global Variables
//******************************************************************************
static usb_status_t usb_status = usb_status_not_mounted;

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

// Variables used to line status (connected/disconnected)
static bool connected_status = false;
static struct mutex st_mutex;

static bool line_state_changed_flag = false;
static uint32_t led_rxtx_activity = 0;

/*******************************************************************************
 * @brief Echo to either Serial0 or Serial1
 * with Serial0 as all lower case, Serial1 as all upper case
 ******************************************************************************/
static void echo_serial_port(uint8_t itf, uint8_t buf[], uint32_t count)
{

    for(uint32_t i = 0; i < count; i++)
    {
        buf[i] = ~buf[i];
        tud_cdc_n_write_char(itf, buf[i]);
    }
    tud_cdc_n_write_flush(itf);
}

/*******************************************************************************
 * @brief Set USB connected status
 * @param Connected status: 1=connected, 0=disconnected/suspended
 ******************************************************************************/
static inline void set_connected_status(bool connected_flag)
{
    mutex_enter_blocking(&st_mutex);
    connected_status = connected_flag;
    mutex_exit(&st_mutex);
}

/*******************************************************************************
 * @brief Get USB connected status
 * @return USB connected status: 1=connected, 0=disconnected/suspended
 ******************************************************************************/
bool usb_get_connected_status(void)
{
    bool connected_flag;
    mutex_enter_blocking(&st_mutex);
    connected_flag = connected_status;
    mutex_exit(&st_mutex);
    return connected_flag;
}

//******************************************************************************
// Device callbacks
//******************************************************************************

/*******************************************************************************
 * @brief Callback
 * Invoked when device is mounted
 ******************************************************************************/
void tud_mount_cb(void)
{
    usb_status = usb_status_mounted;
}

/*******************************************************************************
 * @brief Callback
 * Invoked when device is unmounted
 ******************************************************************************/
void tud_umount_cb(void)
{
    usb_status = usb_status_not_mounted;
}

/*******************************************************************************
 * @brief Callback
 * Invoked when usb bus is suspended
 * remote_wakeup_en : if host allow us  to perform remote wakeup
 * Within 7ms, device must draw an average of current less than 2.5 mA from bus
 ******************************************************************************/
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    usb_status = usb_status_suspended;
}

/*******************************************************************************
 * @brief Callback
 * Invoked when usb bus is resumed
 ******************************************************************************/
void tud_resume_cb(void)
{
    usb_status = tud_mounted() ? usb_status_mounted : usb_status_not_mounted;
}

/*******************************************************************************
 * @brief Callback
 * Invoked when cdc when line state changed e.g connected/disconnected
 * Invoked when line state DTR & RTS are changed via SET_CONTROL_LINE_STATE
 * Use to reset to DFU when disconnect with 1200 bps
 ******************************************************************************/
void tud_cdc_line_state_cb(uint8_t instance, bool dtr, bool rts)
{
    (void)rts;

    // DTR = false is counted as disconnected
    if(!dtr)
    {
        TP_TGL(TP8);

        // touch1200 only with first CDC instance (Serial)
        if(instance == 0)
        {
            cdc_line_coding_t coding;
            tud_cdc_get_line_coding(&coding);
            if(coding.bit_rate == 1200)
            {
                /*
                * This function reboots the device into the BOOTSEL mode ('usb boot").
                *
                * Facilities are provided to enable an "activity light" via GPIO attached LED for the USB Mass Storage Device,
                * and to limit the USB interfaces exposed.
                *
                * \param usb_activity_gpio_pin_mask 0 No pins are used as per a cold boot. Otherwise a single bit set indicating which
                *                               GPIO pin should be set to output and raised whenever there is mass storage activity
                *                               from the host.
                * \param disable_interface_mask value to control exposed interfaces
                *  - 0 To enable both interfaces (as per a cold boot)
                *  - 1 To disable the USB Mass Storage Interface
                *  - 2 To disable the USB PICOBOOT Interface
                */
                reset_usb_boot(0, 0);
            }
        }
    }
    else {
        TP_TGL(TP9);
    }

    // Throw away data still to be sent
    mutex_enter_blocking(&tx_mutex);
    tx_cnt = 0;
    mutex_exit(&tx_mutex);

    // Throw away still available received data
    mutex_enter_blocking(&rx_mutex);
    rx_cnt = 0;
    mutex_exit(&rx_mutex);

    line_state_changed_flag = true;
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
        TP_TGL(TP7);
        
        mutex_enter_blocking(&lc_mutex);

        memcpy(&line_coding, p_line_coding, sizeof(line_coding));
        line_coding_changed++;

        mutex_exit(&lc_mutex);
    }
}

/*******************************************************************************
 * @brief Check if there is new received data, and copy it to the buffer
 * @param [out] buff - pointer to buffer where the received data is copied
 * @param [in] buff_len - the length of the buff buffer in bytes
 * @return count of copied bytes into buff buffer
 ******************************************************************************/
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

/*******************************************************************************
 * @brief Write data tu USB transmit buffer (to be sent)
 * @param [in] buff - pointer to data buffer to send
 * @param [in] buff_len - length of the buff buffer in bytes
 * @return count of bytes copied into USB transmit buffer
 ******************************************************************************/
uint32_t usb_set_tx(const uint8_t * buff, uint32_t buff_len)
{
    uint32_t cnt = 0ul;

    if((buff_len == 0) || (buff == NULL))
        return 0UL;

    // Do not accept data to send if line status has not yet changed
    //  (port has not been yet opened?)
    if(!line_state_changed_flag)
        return 0UL;

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

/*******************************************************************************
 * @brief USB CDC (Communication Device Class) main function
 ******************************************************************************/
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
                    led_rxtx_activity = USB_LED_RXTX_ACTIVITY_TOUT;
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
                            led_rxtx_activity = USB_LED_RXTX_ACTIVITY_TOUT;
                        }
                    }
                }

                // Unlock the buffer
                mutex_exit(&tx_mutex);
            }
        }
    }
}

/*******************************************************************************
 * @brief Blink board LED according to USB state
 * 
 *  Not mounted or suspended: blink 100ms every 1s
 *          100ms        100ms        100ms        100ms
 *           +-+          +-+          +-+          +-+
 *       ...-+ +----------+ +----------+ +----------+ +---...
 *                        :<--- 1s --->:
 * 
 *  Mounted (no data transfer): blink in 1s cycle
 *          
 *       ...---+          +------------+          +----...
 *             +----------+            +----------+ 
 *                        :<--- 1s --->:
 * 
 *  Mounted (data transfer): blink in 100ms cycle
 *          
 *           +-+ +-+ +-+ +-+ +-+ +-+ +-+ +-+ +-+ +-+ +-+  
 *       ...-+ +-+ +-+ +-+ +-+ +-+ +-+ +-+ +-+ +-+ +-+ +-...
 *                              -->:-:<-- 100ms
 * 
 ******************************************************************************/
void led_blinking_task(void)
{
    static uint32_t led_task_ms = 0;
    static uint32_t led_task_cnt = 0;
    static bool led_state = false;

    // Every 100ms
    if((board_millis() - led_task_ms) < 100)
        return;

    led_task_ms += 100;

    led_task_cnt++;
    if(led_task_cnt >= 10)
        led_task_cnt = 0;
        
    if(usb_status == usb_status_mounted)
    {
        // If Rx/Tx activity toggle every 100ms if not toggle every 1s
        if((led_rxtx_activity != 0) || (led_task_cnt == 0))
            led_state = 1 - led_state;
    }
    else{
        led_state = (led_task_cnt == 0);
    }

    if(led_rxtx_activity > 0)
        led_rxtx_activity--;

    board_led_write(led_state);
}

/*******************************************************************************
 * @brief USB main task - running on separate kern
 ******************************************************************************/
void usb_main(void)
{
    mutex_init(&rx_mutex);
    mutex_init(&tx_mutex);
    mutex_init(&lc_mutex);
    mutex_init(&st_mutex);

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
