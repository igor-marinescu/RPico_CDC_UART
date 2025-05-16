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

//******************************************************************************
// Includes
//******************************************************************************
#include <string.h>     // memcpy
#include <stdio.h>
//#include <stdarg.h>     //varg

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "puart_drv.h"
#include "gpio_drv.h"

#include "puart_tx.pio.h"

#include "uart_ascii.h" //!!! TODO: delete

//******************************************************************************
// Function Prototypes
//******************************************************************************
static void puart_drv_irq(void);

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
//   0                   tx_rd_idx           tx_wr_idx            PUART_TX_BUFF
//                    puart_drv_irq()     puart_drv_puts()
//
//
//      ...data to send --->|<--- free space --->|<-- data to send...
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |X|X|X|X|X|X|X|X|X|X|X|X| | | | | | | | | | |X|X|X|X|X|X|X|X|X|X|
//  +-+-+-+-+-+-+-+-+-+-+-+-+^+-+-+-+-+-+-+-+-+-+^+-+-+-+-+-+-+-+-+-+
//   :                       |                   |                  :
//   0                   tx_wr_idx           tx_rd_idx            PUART_TX_BUFF
//                    puart_drv_puts()     puart_drv_irq()
//
static char tx_buffer[PUART_TX_BUFF];
static volatile uint32_t tx_wr_idx = 0UL;
static volatile uint32_t tx_rd_idx = 0UL;
static volatile uint32_t tx_free_cnt = sizeof(tx_buffer);
static volatile bool send_semaphore = false;

static char rx_buffer[PUART_RX_BUFF];
static volatile uint32_t rx_wr_idx = 0UL;
static volatile uint32_t rx_rd_idx = 0UL;
static volatile uint32_t rx_wr_old = 0UL;
static volatile bool read_semaphore = false;

static char rx_buff0[16];
static volatile uint32_t rx_idx0 = 0UL;
static volatile bool rx_buff0_semaphore = false;

static PIO pio = pio0;
static uint sm = 0;

static pio_interrupt_source_t pio_irq_src_txnfull;
static uint irq_idx_txnfull;
static int8_t pio_irq_txnfull;

static int tx_active_status = 0;

// Tx Not-Full Interrupt Disable/Enable Macros
#define PIO_IRQ_TXNFULL_DISABLE()   pio_set_irqn_source_enabled(pio, irq_idx_txnfull, pio_irq_src_txnfull, false)
#define PIO_IRQ_TXNFULL_ENABLE()    pio_set_irqn_source_enabled(pio, irq_idx_txnfull, pio_irq_src_txnfull, true)

/*******************************************************************************
 * @brief PIO-UART Init function
 ******************************************************************************/
void puart_drv_init(void)
{
    const uint SERIAL_BAUD = 115200;

    pio = pio0;
    sm = 0;

    uint offset = pio_add_program(pio, &puart_tx_program);
    puart_tx_program_init(pio, sm, offset, PUART_TX_PIN, SERIAL_BAUD);
    PUART_DRV_LOG("PUART drv init\r\n");

    // Configure TX Fifo not full interrupt

    // Find a free irq
    pio_irq_txnfull = pio_get_irq_num(pio, 0);
    if(irq_get_exclusive_handler(pio_irq_txnfull))
    {
        pio_irq_txnfull++;
        if(irq_get_exclusive_handler(pio_irq_txnfull))
        {
            PUART_DRV_LOG("PUART error all irq in use\r\n");
        }
    }

    // Enable interrupt
    irq_add_shared_handler(pio_irq_txnfull, puart_drv_irq, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY); // Add a shared IRQ handler
    irq_set_enabled(pio_irq_txnfull, true);

    // Get index of the IRQ
    irq_idx_txnfull = pio_irq_txnfull - pio_get_irq_num(pio, 0);
    pio_irq_src_txnfull = pio_get_tx_fifo_not_full_interrupt_source(sm);
    
    // Disable interrupt
    // PIO_IRQ_TXNFULL_DISABLE();
    // irq_set_enabled(pio_irq_txnfull, false);
    // irq_remove_handler(pio_irq_txnfull, puart_drv_irq);
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
        PUART_DRV_LOG("PUART RX buffer overflow\r\n");
    }
}

/*******************************************************************************
 * @brief Get the count of available (free) bytes in Tx buffer
 * @return count of free bytes in Tx buffer
 ******************************************************************************/
static inline uint32_t get_tx_buff_free_cnt(void)
{
    // |<------ free ----->|<--- busy for send --->|<------ free ----->|
    // |---- available ----|XXXXXXXXXXXXXXXXXXXXXXX|---- available ----|
    // 0                tx_rd_idx              tx_wr_idx       sizeof(tx_buffer)
    if(tx_wr_idx > tx_rd_idx)
        return (sizeof(tx_buffer) - tx_wr_idx + tx_rd_idx);

        // |<-- busy for send -->|<------ free ----->|<-- busy for send -->|
    // |XXXXXXXXXXXXXXXXXXXXX|---- available ----|XXXXXXXXXXXXXXXXXXXXX|
    // 0                tx_wr_idx              tx_rd_idx       sizeof(tx_buffer)
    else if(tx_wr_idx < tx_rd_idx)
        return (tx_rd_idx - tx_wr_idx);    

        return sizeof(tx_buffer);
}

/*******************************************************************************
 * @brief PIO-UART interrupt handler
 ******************************************************************************/
static void puart_drv_irq(void)
{
    TP_TGL(TP7);

/*
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
*/

    while(!pio_sm_is_tx_fifo_full(pio, sm))
    {
        // Is the interrupt called while in send function?
        if(send_semaphore)
        {
            // Stop sending and do not modify tx_rd_idx to avoid corrupted index.
            // Disable interrupt. The send-function which sets send_semaphore 
            // will re-enable the interrupt.
            PIO_IRQ_TXNFULL_DISABLE();
            break;
        }
        // Nothing to send?
        else if(tx_wr_idx == tx_rd_idx)
        {
            TP_TGL(TP8);
            // Disable tx irq
            PIO_IRQ_TXNFULL_DISABLE();
            break;
        }
        // Safe to send the data
        else{
            pio->txf[sm] = (uint32_t) tx_buffer[tx_rd_idx++];
            if(tx_rd_idx >= sizeof(tx_buffer))
                tx_rd_idx = 0UL;
            // Update Tx Buffer free count
            tx_free_cnt = get_tx_buff_free_cnt();
            TP_TGL(TP5);
        }
    }
}

/*******************************************************************************
 * @brief Send data to PIO-UART
 * @param buff [in] data to send to PIO-UART
 * @param len [in] count of bytes in buff to send to PIO-UART
 * @return count of copied bytes
 ******************************************************************************/
uint32_t puart_drv_send_buff(const uint8_t * buff, uint32_t len)
{
    if((buff == NULL) || (len == 0UL))
        return 0UL;

    // TX-ACTIVE Signal Control
    #ifdef TX_ACTIVE_ENABLED
        tx_active_status = 1;
        TP_SET(TP6);
    #endif

    send_semaphore = true;

    if(tx_wr_idx >= sizeof(tx_buffer))
        tx_wr_idx = 0UL;

    uint32_t available = 0UL;
    uint32_t copied = 0UL;
    uint32_t to_copy = len;

    //PUART_DRV_LOG("%02x %3u | w%4i r%4i\r\n", buff[0], len, tx_wr_idx, tx_rd_idx);

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

        //PUART_DRV_LOG(">1%4i | w%4i r%4i\r\n", available, tx_wr_idx, tx_rd_idx);
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

        //PUART_DRV_LOG(">2%4i | w%4i r%4i\r\n", available, tx_wr_idx, tx_rd_idx);
    }

    // Update Tx Buffer free count
    tx_free_cnt = get_tx_buff_free_cnt();

    send_semaphore = false;

    // Reactivate IRQ (if not already active)
    PIO_IRQ_TXNFULL_ENABLE();

    return copied;
}

/*******************************************************************************
 * @brief Get the free size of Tx buffer
 * @return count of bytes still free in Tx buffer
 ******************************************************************************/
uint32_t puart_drv_get_tx_free_cnt(void)
{
    return tx_free_cnt;
}

/*******************************************************************************
 * @brief Check if PIO-UART has finished transmiting data and reset TX_ACTIVE signal
 ******************************************************************************/
void puart_drv_control_tx_active(void)
{
    if(tx_active_status)
    {

        // If nothing more to send and PIO-UART SM is not busy
        // (Check EXEC_STALLED (RO) - If 1, an instruction written to SMx_INSTR is stalled)
        if(tx_wr_idx == tx_rd_idx) 
        {
            if(pio_sm_is_exec_stalled(pio, sm))
            {
                PUART_DRV_LOG("3\r\n");
                tx_active_status = 0;
                TP_CLR(TP6);
            }
        }
    }
}

/*******************************************************************************
 * @brief Check if there are received characters and copy them to buffer
 * @param buff [out] pointer to buffer where the received characters are copied
 * @param buff_max_len [in] the length in bytes (characters) of buff
 * @return count of copied received characters
 ******************************************************************************/
uint32_t puart_drv_get_rx(char * buff, uint32_t buff_max_len)
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
