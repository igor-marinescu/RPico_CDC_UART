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
#include "ustime.h"

#include "puart_tx.pio.h"
#include "puart_rx.pio.h"

#include "uart_ascii.h" //!!! TODO: delete

//******************************************************************************
// Function Prototypes
//******************************************************************************
static void irq_txnfull_func(void);
static void irq_rx_func(void);

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
static piodata_t tx_buffer[PUART_TX_BUFF];
static volatile uint32_t tx_wr_idx = 0UL;
static volatile uint32_t tx_rd_idx = 0UL;
static volatile uint32_t tx_free_cnt = PUART_TX_BUFF;
static volatile bool send_semaphore = false;

static piodata_t rx_buffer[PUART_RX_BUFF];
static volatile uint32_t rx_wr_idx = 0UL;
static volatile uint32_t rx_rd_idx = 0UL;
static volatile uint32_t rx_wr_old = 0UL;
static volatile bool read_semaphore = false;

static piodata_t rx_buff0[PUART_RX_BUFF0];
static volatile uint32_t rx_idx0 = 0UL;
static volatile bool rx_buff0_semaphore = false;

static PIO pio_tx = pio0;
static uint sm_tx = 0;
static pio_interrupt_source_t pio_irq_src_txnfull;
static uint irq_idx_txnfull;
static int8_t pio_irq_txnfull;

static PIO pio_rx = pio0;
static uint sm_rx = 1;
static uint irq_idx_rx;
static int8_t pio_irq_rx;

static int tx_active_status = 0;
static ustime_t tx_last_element_ustime;

static ustime_t element_req_ustime;

// Tx Not-Full Interrupt Disable/Enable Macros
#define PIO_IRQ_TXNFULL_DISABLE()   pio_set_irqn_source_enabled(pio_tx, irq_idx_txnfull, pio_irq_src_txnfull, false)
#define PIO_IRQ_TXNFULL_ENABLE()    pio_set_irqn_source_enabled(pio_tx, irq_idx_txnfull, pio_irq_src_txnfull, true)

/*******************************************************************************
 * @brief PIO-UART Init function
 ******************************************************************************/
void puart_drv_init(void)
{
    int irq_found_flag;

    // Calculate time necessary to transfer a piodata_t element (1 start + PIO_DATA_BIT + 1 stop + 1 reserve)
    element_req_ustime = get_baudrate_transfer_ustime(PIO_BAUDRATE, (1 + PIO_DATA_BIT + 1 + 1), 1);

    //--------------------------------------------------------------------------
    // Configure TX-PIO
    pio_tx = pio0;
    sm_tx = 0;

    uint offset = pio_add_program(pio_tx, &puart_tx_program);
    puart_tx_program_init(pio_tx, sm_tx, offset, PUART_TX_PIN, PIO_BAUDRATE);
    // Send bits_count as first data to SM
    pio_tx->txf[sm_tx] = (PIO_DATA_BIT - 1);

    // Find a free irq
    irq_found_flag = 1;
    pio_irq_txnfull = pio_get_irq_num(pio_tx, 0);
    if(irq_get_exclusive_handler(pio_irq_txnfull))
    {
        pio_irq_txnfull++;
        if(irq_get_exclusive_handler(pio_irq_txnfull))
        {
            PUART_DRV_LOG("PUART-TX error all irq in use\r\n");
            irq_found_flag = 0;
        }
    }

    // Configure TX Fifo not full interrupt
    if(irq_found_flag != 0)
    {
        irq_add_shared_handler(pio_irq_txnfull, irq_txnfull_func, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(pio_irq_txnfull, true);

        // Get index of the IRQ
        irq_idx_txnfull = pio_irq_txnfull - pio_get_irq_num(pio_tx, 0);
        pio_irq_src_txnfull = pio_get_tx_fifo_not_full_interrupt_source(sm_tx);
        
        // Disable interrupt
        // PIO_IRQ_TXNFULL_DISABLE();
        // irq_set_enabled(pio_irq_txnfull, false);
        // irq_remove_handler(pio_irq_txnfull, irq_txnfull_func);
    }

    //PUART_DRV_LOG("PUART-TX drv init: @%i, %i, %i\r\n", offset, pio_irq_txnfull, irq_idx_txnfull);

    //--------------------------------------------------------------------------
    // Configure RX-PIO
    pio_rx = pio0;
    sm_rx = 1;

    offset = pio_add_program(pio_rx, &puart_rx_program);
    puart_rx_program_init(pio_rx, sm_rx, offset, PUART_RX_PIN, PIO_BAUDRATE);
    // Send bits_count as first data to SM
    pio_rx->txf[sm_rx] = (PIO_DATA_BIT - 1);

    // Find a free irq
    irq_found_flag = 1;
    pio_irq_rx = pio_get_irq_num(pio_rx, 1);
    if (irq_get_exclusive_handler(pio_irq_rx))
    {
        pio_irq_rx++;
        if (irq_get_exclusive_handler(pio_irq_rx))
        {
            PUART_DRV_LOG("PUART-RX error all irq in use\r\n");
            irq_found_flag = 0;
        }
    }

    // Configure RX interrupt
    if(irq_found_flag != 0)
    {
        irq_add_shared_handler(pio_irq_rx, irq_rx_func, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(pio_irq_rx, true);

        // Get index of the IRQ
        irq_idx_rx = pio_irq_rx - pio_get_irq_num(pio_rx, 0);
        // Set pio to tell us when the FIFO is NOT empty
        pio_set_irqn_source_enabled(pio_rx, irq_idx_rx, pio_get_rx_fifo_not_empty_interrupt_source(sm_rx), true);

        // Disable interrupt
        // pio_set_irqn_source_enabled(pio_rx, irq_idx_rx, pio_get_rx_fifo_not_empty_interrupt_source(sm_rx), false);
        //irq_set_enabled(pio_irq_rx, false);
        //irq_remove_handler(pio_irq_rx, irq_rx_func);
    }

    //PUART_DRV_LOG("PUART-RX drv init: @%i, %i, %i\r\n", offset, pio_irq_rx, irq_idx_rx);    
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
        if(rx_rd_idx >= PUART_RX_BUFF)
            rx_rd_idx = 0UL;
        rx_wr_old = rx_wr_idx;
        PUART_DRV_LOG("PUART RX buffer overflow\r\n");
    }
}

/*******************************************************************************
 * @brief Get the count of available (free) elements in Tx buffer
 * @return count of free elements in Tx buffer
 ******************************************************************************/
static inline uint32_t get_tx_buff_free_cnt(void)
{
    // |<------ free ----->|<--- busy for send --->|<------ free ----->|
    // |---- available ----|XXXXXXXXXXXXXXXXXXXXXXX|---- available ----|
    // 0                tx_rd_idx              tx_wr_idx       PUART_TX_BUFF
    if(tx_wr_idx > tx_rd_idx)
        return (PUART_TX_BUFF - tx_wr_idx + tx_rd_idx);

        // |<-- busy for send -->|<------ free ----->|<-- busy for send -->|
    // |XXXXXXXXXXXXXXXXXXXXX|---- available ----|XXXXXXXXXXXXXXXXXXXXX|
    // 0                tx_wr_idx              tx_rd_idx       PUART_TX_BUFF
    else if(tx_wr_idx < tx_rd_idx)
        return (tx_rd_idx - tx_wr_idx);    

    return PUART_TX_BUFF;
}

/*******************************************************************************
 * @brief PIO-UART RX interrupt handler
 ******************************************************************************/
static void irq_rx_func(void)
{
    rx_wr_old = rx_wr_idx;

    // In case we have some data collected in rx backup buffer (rx_buff0), 
    // copy data to rx main buffer (if necessary overwrite old data)
    if((rx_idx0 > 0UL) && !rx_buff0_semaphore && !read_semaphore)
    {
        for(uint32_t i = 0UL; i < rx_idx0; i++)
        {
            if(rx_wr_idx >= PUART_RX_BUFF)
                rx_wr_idx = 0UL;
            rx_buffer[rx_wr_idx++] = rx_buff0[i];
        }
        rx_idx0 = 0UL;
        _check_wr_jumps_over_rd();
    }

    while(!pio_sm_is_rx_fifo_empty(pio_rx, sm_rx))
    {
        // For 9-bit, shift data to right (23 = 32 - 9)
        uint32_t rx_v32 = pio_rx->rxf[sm_rx];
        piodata_t rx_ch = (piodata_t) (rx_v32 >> 23);

        // Is the interrupt called while reading rx main buffer in puart_drv_get_rx?
        if(read_semaphore)
        {
            // Yes: Do not access rx main buffer to avoid corrupted data.
            // Instead store data in rx backup buffer (rx_buff0). Both 
            // read_semaphore and rx_buff0_semaphore cannot be set at the same 
            // time. If read_semaphore is set -> it is safe to access rx_buff0.
            if(rx_idx0 < PUART_RX_BUFF0)
                rx_buff0[rx_idx0++] = rx_ch;
            else {
                // Rx backup buffer (rx_buff0) already full! Received data lost!
                PUART_DRV_LOG("PUART RX buffer0 full\r\n");
            }
        }
        else{
            // No: It is safe to store data in rx main buffer.
            if(rx_wr_idx >= PUART_RX_BUFF)
                rx_wr_idx = 0UL;
            rx_buffer[rx_wr_idx++] = rx_ch;
            if(rx_wr_idx >= PUART_RX_BUFF)
                rx_wr_idx = 0UL;
            _check_wr_jumps_over_rd();
        }
        TP_TGL(TP4);
    }
}

/*******************************************************************************
 * @brief PIO-UART TX interrupt handler
 ******************************************************************************/
static void irq_txnfull_func(void)
{
    TP_TGL(TP7);

    while(!pio_sm_is_tx_fifo_full(pio_tx, sm_tx))
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
            // Tx Buffer is empty
            tx_free_cnt = PUART_TX_BUFF;
            //TP_TGL(TP8);
            // Disable tx irq
            PIO_IRQ_TXNFULL_DISABLE();
            break;
        }
        // Safe to send the data
        else{
            pio_tx->txf[sm_tx] = (uint32_t) tx_buffer[tx_rd_idx++];
            if(tx_rd_idx >= PUART_TX_BUFF)
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
 * @param len [in] count of elements in buff to send to PIO-UART
 * @return count of copied elements
 ******************************************************************************/
uint32_t puart_drv_send_buff(const piodata_t * buff, uint32_t len)
{
    if((buff == NULL) || (len == 0UL))
        return 0UL;

    // TX-ACTIVE Signal Control
    #ifdef TX_ACTIVE_ENABLED
        tx_active_status = 1;
        PUART_TX_ACTIVE_SIGNAL_SET();
    #endif

    send_semaphore = true;

    if(tx_wr_idx >= PUART_TX_BUFF)
        tx_wr_idx = 0UL;

    uint32_t available = 0UL;
    uint32_t copied = 0UL;
    uint32_t to_copy = len;

    //PUART_DRV_LOG("%02x %3u | w%4i r%4i\r\n", buff[0], len, tx_wr_idx, tx_rd_idx);

    // |<------ free ----->|<--- busy for send --->|<------ free ----->|
    // |---- available ----|XXXXXXXXXXXXXXXXXXXXXXX|---- available ----|
    // 0                tx_rd_idx              tx_wr_idx       PUART_TX_BUFF
    if(tx_wr_idx >= tx_rd_idx)
    {
        available = PUART_TX_BUFF - tx_wr_idx;
        // tx_wr_idx should remain behind tx_rd_idx (otherwise we lose data)
        if((tx_rd_idx == 0) && (available > 0))
            available -= 1;

        if(to_copy > available)
            to_copy = available;
        if(to_copy > 0)
        {
            memcpy(&tx_buffer[tx_wr_idx], buff, to_copy * sizeof(piodata_t));
            len -= to_copy;
            copied += to_copy;
            tx_wr_idx += to_copy;
            if(tx_wr_idx >= PUART_TX_BUFF)
                tx_wr_idx = 0UL;
        }

        //PUART_DRV_LOG(">1%4i | w%4i r%4i\r\n", available, tx_wr_idx, tx_rd_idx);
    }

    to_copy = len;

    // |<-- busy for send -->|<------ free ----->|<-- busy for send -->|
    // |XXXXXXXXXXXXXXXXXXXXX|---- available ----|XXXXXXXXXXXXXXXXXXXXX|
    // 0                tx_wr_idx              tx_rd_idx       PUART_TX_BUFF
    if((len > 0UL) && (tx_wr_idx < tx_rd_idx))
    {
        available = tx_rd_idx - tx_wr_idx;
        // tx_wr_idx should remain behind tx_rd_idx (otherwise we lose data)
        available -= 1;

        if(to_copy > available)
            to_copy = available;
        if(to_copy > 0)
        {
            memcpy(&tx_buffer[tx_wr_idx], &buff[copied], to_copy * sizeof(piodata_t));
            copied += to_copy;
            tx_wr_idx += to_copy;
            if(tx_wr_idx >= PUART_TX_BUFF)
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
 * @brief Get the free size (in elements count) of Tx buffer
 * @return count of elements still free in Tx buffer
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
        // If nothing more to send
        if(tx_wr_idx == tx_rd_idx) 
        {
            if(tx_active_status == 1)
            {
                // Check if TX-FIFO is empty
                if((pio_tx->flevel & PIO_FLEVEL_TX0_BITS) == 0)
                {
                    // Goto status 2 where we wait for the last element to be sent
                    tx_active_status = 2;
                    tx_last_element_ustime = get_sys_ustime();
                }
            }
            else {
                // Check if last element was sent (time elapsed requited to send one element)
                if(get_diff_sys_ustime(tx_last_element_ustime) >= element_req_ustime)
                {
                    tx_active_status = 0;
                    PUART_TX_ACTIVE_SIGNAL_CLR();
                }
            }
        }
    }
}

/*******************************************************************************
 * @brief Check if there are received elements and copy them to buffer
 * @param buff [out] pointer to buffer where the received elements are copied
 * @param buff_max_len [in] the maximal size of buff (in elements)
 * @return count of copied received elements
 ******************************************************************************/
uint32_t puart_drv_get_rx(piodata_t * buff, uint32_t buff_max_len)
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
        memcpy(buff, rx_buff0, rx_idx0 * sizeof(piodata_t));
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

        if(rx_rd_idx >= PUART_RX_BUFF)
            rx_rd_idx = 0UL;

        // |............|............|//////////////|
        // |            |            |<--- copy --->|
        // 0        rx_wr_idx    rx_rd_idx     PUART_RX_BUFF
        if(rx_rd_idx > rx_wr_idx)
        {
            uint32_t diff = PUART_RX_BUFF - rx_rd_idx;
            if(diff > buff_max_len)
                diff = buff_max_len;

            if(diff > 0UL)
            {
                memcpy(&buff[buff_idx], &rx_buffer[rx_rd_idx], diff * sizeof(piodata_t));
                buff_max_len -= diff;
                buff_idx += diff;
                rx_rd_idx += diff;
                if(rx_rd_idx >= PUART_RX_BUFF)
                    rx_rd_idx = 0UL;
            }
        }

        // |..........|//////////////|.............|
        // |          |<--- copy --->|             |
        // 0       rx_rd_idx      rx_wr_idx    PUART_RX_BUFF
        if(rx_rd_idx < rx_wr_idx)
        {
            uint32_t diff = rx_wr_idx - rx_rd_idx;
            if(diff > buff_max_len)
                diff = buff_max_len;

            if(diff > 0UL)
            {
                memcpy(&buff[buff_idx], &rx_buffer[rx_rd_idx], diff * sizeof(piodata_t));
                buff_max_len -= diff;
                buff_idx += diff;
                rx_rd_idx += diff;
                if(rx_rd_idx >= PUART_RX_BUFF)
                    rx_rd_idx = 0UL;
            }
        }

        read_semaphore = false;
    }
    return buff_idx;
}
