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
 * main - the implementation of a main core functionality
 ******************************************************************************/ 
#include <stdlib.h>
#include <stdio.h>

#include "bsp/board_api.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "gpio_drv.h"
#include "uart_drv.h"
#include "usb_main.h"
#include "uart_ascii.h"

#include "cli.h"
#include "pico/bootrom.h"

#ifdef MAIN_DEBUG
#include DEBUG_INCLUDE
#endif

#include "utils.h"

//******************************************************************************
#ifdef MAIN_DEBUG
    #define MAIN_LOG(...)     DEBUG_PRINTF(__VA_ARGS__)
#else
    #define MAIN_LOG(...)    
#endif  

// System time in micro-seconds
typedef uint32_t ustime_t;

//******************************************************************************
// Global Variables
//******************************************************************************

// System time in us (since the board was powered-on)
ustime_t sys_ustime = 0;

// Data buffers used for USB-UART bridge functionality
static uint32_t usb_rx_cnt = 0;
static uint8_t  usb_rx_data[2048];
static uint32_t usb_tx_cnt = 0;
static uint8_t  usb_tx_data[2048];
static uint32_t uart_tx_cnt = 0;
static uint8_t  uart_tx_data[2048];
static uint32_t uart_rx_cnt = 0;
static uint8_t  uart_rx_data[2048];

// USB line coding
uint32_t usb_line_coding_changed_cnt = 0;
cdc_line_coding_t usb_line_coding;

// UART line codding
uart_line_coding_t uart_line_coding = {
    .bit_rate = 115200,
    .data_bits = 8,
    .stop_bits = 1,
    .parity = UART_PARITY_NONE
};

// Detect pause in UART reception
ustime_t uart_rx_ustime = 0;

#ifdef UART_RX_BYTES_TOUT
ustime_t uart_rx_tout_ustime;
#endif

/*******************************************************************************
 * @brief Display the list of all commands and their info
 * @param argc [in] - arguments count
 * @param args [in] - array with pointers to arguments string
 * @return true if function successfully executed, false in case of error
 ******************************************************************************/
static bool cli_func_help(int argc, char ** args)
{
    cli_func_list();
    return true;
}

/*******************************************************************************
 * @brief Jump to bootloader
 * @param argc [in] - arguments count
 * @param args [in] - array with pointers to arguments string
 * @return true if function successfully executed, false in case of error
 ******************************************************************************/
static bool cli_func_boot(int argc, char ** args)
{
    MAIN_LOG("Jump to boot now!\r\n");
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
    return true;
}

/***************************************************************************//**
* @brief Display a memdump (in hexadecimal format)
* @param argc [in] - arguments count
* @param args [in] - array with pointers to arguments string
* @return true if function successfully executed, false in case of error
*******************************************************************************/
static bool cli_func_memdump(int argc, char ** args)
{
    long addr;
    int len;

    if(argc < 3)
        return false;

    if(!utils_get_long(&addr, args[1], CLI_WORD_SIZE))
        return false;

    if(!utils_get_int(&len, args[2], CLI_WORD_SIZE))
        return false;

    uart_ascii_printf0("memdump %0X %i:\r\n", addr, len);
    uart_ascii_dump((unsigned char *) addr, len, addr);
    return true;
}

/*******************************************************************************
 * @brief Test function for inserting an artificial delay (for test purposes only)
 ******************************************************************************/
static void test_delay(void)
{
    static volatile long test1 = 0;
    static volatile long test2 = 0;

    // Test: add delay to main program cycle: ~30ms
    for(int i = 0; i < 100000; i++)
    {
        test1 = test1 + (long) i;
        if(i != 0)
            test2 = test1 / (long) i;
        else
            test2 += test1;
    }
}

/*******************************************************************************
 * @brief Convert USB line coding to UART line coding
 * @param [in] ptr_usb_lc - pointer to USB line coding
 * @param [out] ptr_uart_lc - pointer to UART line coding
 ******************************************************************************/
static void line_coding_usb_to_uart(uart_line_coding_t * ptr_uart_lc, const cdc_line_coding_t * ptr_usb_lc)
{
    ptr_uart_lc->bit_rate = (unsigned int) ptr_usb_lc->bit_rate;

    // USB stop bits: 0 = 1 stop bit, 1 = 1.5 stop bits, 2 = 2 stop bits
    // UART stop bits: 1 = 1 stop bit, 2 = 2 stop bits
    ptr_uart_lc->stop_bits = (unsigned int) ((ptr_usb_lc->stop_bits == 0) ? 1 : 2);
    
    ptr_uart_lc->data_bits = (unsigned int) ptr_usb_lc->data_bits;

    // USB parity: 0 = None, 1 = Odd, 2 = Even, 3 = Mark, 4 = Space
    // UART parity: UART_PARITY_NONE (0), UART_PARITY_EVEN (1), UART_PARITY_ODD (2)
    if(ptr_usb_lc->parity == 1)
        ptr_uart_lc->parity = UART_PARITY_ODD;
    else if(ptr_usb_lc->parity == 2)
        ptr_uart_lc->parity = UART_PARITY_EVEN;
    else
        ptr_uart_lc->parity = UART_PARITY_NONE;
}

/*******************************************************************************
 * @brief Get System Time in us
 *        Warning! This is not safe in case of multi-core
 * @return System Time in us
 ******************************************************************************/
ustime_t get_sys_ustime(void)
{
    uint32_t lr = timer_hw->timelr;
    timer_hw->timehr;
    return (ustime_t) lr;
}

/*******************************************************************************
 * @brief Get the difference (always positive) between two ustime_t variables
 * @param end_ustime [in] end ustime_t variable
 * @param start_ustime [in] start ustime_t variable
 * @return difference: end_ustime - start_ustime
 ******************************************************************************/
ustime_t get_diff_ustime(const ustime_t end_ustime, const ustime_t start_ustime)
{
    int32_t diff = ((int32_t) end_ustime) - ((int32_t) start_ustime);
    if(diff < 0)
        return (ustime_t)(-diff);
    return (ustime_t)(diff);
}

/*******************************************************************************
 * @brief Calculate time necessary to send/receive a number of bytes over UART
 *        (depending on UART line coding)
 * 
 *              1 [s] * bytes_cnt * bits_pro_byte [bits]
 *  time [us] = ----------------------------------------
 *                       baudrate [bits/s]
 * 
 *              1000000 [us] * bytes_cnt * (1 + data_bits + stop_bits) [bits]
 *  time [us] = -------------------------------------------------------------
 *                                  baudrate [bits/s]
 * 
 * @param [in] ptr_uart_lc - pointer to UART line coding
 * @param [in] bytes_cnt - count of bytes to transfer
 * @return time in [us] required to transfer a number of bytes
*******************************************************************************/
ustime_t get_uart_ustime(const uart_line_coding_t * ptr_uart_lc, const uint32_t bytes_cnt)
{
    if(!ptr_uart_lc || (ptr_uart_lc->bit_rate == 0))
        return 0;

    uint32_t val = 1;
    val += (uint32_t) ptr_uart_lc->data_bits;
    val += (uint32_t) ptr_uart_lc->stop_bits;
    val *= bytes_cnt;
    val *= 1000000;

    return (ustime_t)(val / (uint32_t) ptr_uart_lc->bit_rate);
}

/*******************************************************************************
 * @brief Program main cycle
 ******************************************************************************/
int main(void)
{
    board_init();

    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

    if (board_init_after_tusb)
    {
        board_init_after_tusb();
    }

    gpio_drv_init();
    uart_drv_init(&uart_line_coding);
    uart_ascii_init();

    cli_init();
    cli_add_func("help",    NULL,   cli_func_help,      "help");
    cli_add_func("boot",    NULL,   cli_func_boot,      "boot");
    cli_add_func("memdump", NULL,   cli_func_memdump,   "memdump <addr> <len>");

    MAIN_LOG(PROJECT_NAME " built: " __DATE__ " " __TIME__ "\r\n");

    multicore_launch_core1(usb_main);

    #ifdef UART_RX_BYTES_TOUT
    uart_rx_tout_ustime = get_uart_ustime(&uart_line_coding, UART_RX_BYTES_TOUT);
    #endif

    while(1)
    {
        sys_ustime = get_sys_ustime();
        TP_TGL(TP3);

        if(uart_tx_cnt > sizeof(uart_tx_data))
            uart_tx_cnt = sizeof(uart_tx_data);

        if(usb_tx_cnt > sizeof(usb_tx_data))
            usb_tx_cnt = sizeof(usb_tx_data);

        // Test: add delay to main program cycle: ~30ms
        //test_delay();

        //----------------------------------------------------------------------

        // Check if USB line coding has changed and apply same settings to UART
        if(usb_line_coding_changed_cnt != usb_has_line_coding_changed())
        {
            usb_line_coding_changed_cnt = usb_get_line_coding(&usb_line_coding);
            MAIN_LOG("USB line coding: %i:%i:%i:%i\r\n", 
                    usb_line_coding.bit_rate,
                    usb_line_coding.data_bits,
                    usb_line_coding.stop_bits,
                    usb_line_coding.parity);

            line_coding_usb_to_uart(&uart_line_coding, &usb_line_coding);
            uart_drv_init(&uart_line_coding);

            #ifdef UART_RX_BYTES_TOUT
            uart_rx_tout_ustime = get_uart_ustime(&uart_line_coding, UART_RX_BYTES_TOUT);
            #endif
        }

        //----------------------------------------------------------------------

        // USB data received?
        usb_rx_cnt = usb_get_rx(usb_rx_data, sizeof(usb_rx_data));
        if(usb_rx_cnt > 0UL)
        {
            // Check how much available space in uart_tx_data
            uint32_t available = sizeof(uart_tx_data) - uart_tx_cnt;
            if(usb_rx_cnt > available)
            {
                usb_rx_cnt = available;
                // Data lost, no more space in UART tx buffer
                MAIN_LOG("USB rx lost (UART tx full)\r\n");
            }

            // Append received USB data to UART tx buffer
            if(usb_rx_cnt > 0UL)
            {
                memcpy(&uart_tx_data[uart_tx_cnt], usb_rx_data,  usb_rx_cnt);
                uart_tx_cnt += usb_rx_cnt;
            }
        }

        // USB data to send?
        if(usb_tx_cnt > 0)
        {
#ifdef UART_RX_BYTES_TOUT
            // Send UART->USB only when we received >= 64 bytes
            // or there is a delay of >= 2 bytes-time from UART
            if((usb_tx_cnt >= 64) 
            || (get_diff_ustime(uart_rx_ustime, sys_ustime) > uart_rx_tout_ustime))
#endif
            {
                uint32_t sent = usb_set_tx(usb_tx_data, usb_tx_cnt);
                if(sent < usb_tx_cnt)
                {
                    // Could not send all data? Move the remaining USB tx buffer
                    // to left to send it next time
                    memmove(&usb_tx_data[0], &usb_tx_data[sent],  usb_tx_cnt - sent);
                    usb_tx_cnt -= sent;
                }
                else usb_tx_cnt = 0UL;
            }
        }

        //----------------------------------------------------------------------

        // UART data received?
        uart_rx_cnt = uart_drv_get_rx(uart_rx_data, sizeof(uart_rx_data));
        if(uart_rx_cnt > 0UL)
        {
            // Check how much available space in usb_tx_data
            uint32_t available = sizeof(usb_tx_data) - usb_tx_cnt;
            if(uart_rx_cnt > available)
            {
                uart_rx_cnt = available;
                // Data lost, no more place in USB buffer
                MAIN_LOG("UART rx lost (USB tx full)\r\n");
            }

            // Append received UART data to USB tx buffer
            if(uart_rx_cnt > 0UL)
            {
                memcpy(&usb_tx_data[usb_tx_cnt], uart_rx_data,  uart_rx_cnt);
                usb_tx_cnt += uart_rx_cnt;
            }

            uart_rx_cnt = 0;
            uart_rx_ustime = sys_ustime;
        }

        // UART data to send?
        if(uart_tx_cnt > 0UL)
        {
            uint32_t sent = uart_drv_send_buff(uart_tx_data, uart_tx_cnt);
            if(sent < uart_tx_cnt)
            {
                // Could not sent all data? Move the remaining UART tx buffer 
                // to left to it send next time
                memmove(&uart_tx_data[0], &uart_tx_data[sent],  uart_tx_cnt - sent);
                uart_tx_cnt -= sent;
            }
            else uart_tx_cnt = 0UL;
        }

        cli_poll();
    }
}
