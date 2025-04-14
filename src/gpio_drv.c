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
 * gpio_drv - the GPIO module functions and definitions.
 ******************************************************************************/

//******************************************************************************
// Includes
//******************************************************************************
#include <string.h>
#include "pico/stdlib.h"
#include "gpio_drv.h"

//******************************************************************************
// Function Prototypes
//******************************************************************************

//******************************************************************************
// Global Variables
//******************************************************************************

// Output pins
static const uint32_t out_pin_maps[] = {
    #ifdef GPIO_OUT_0
    GPIO_OUT_0,
    #endif
    #ifdef GPIO_OUT_1
    GPIO_OUT_1,
    #endif
    #ifdef GPIO_OUT_2
    GPIO_OUT_2,
    #endif
    #ifdef GPIO_OUT_3
    GPIO_OUT_3,
    #endif
    #ifdef GPIO_OUT_4
    GPIO_OUT_4,
    #endif
    #ifdef GPIO_OUT_5
    GPIO_OUT_5,
    #endif
    #ifdef GPIO_OUT_6
    GPIO_OUT_6,
    #endif
    #ifdef GPIO_OUT_7
    GPIO_OUT_7,
    #endif
    #ifdef GPIO_OUT_8
    GPIO_OUT_8,
    #endif
    #ifdef GPIO_OUT_9
    GPIO_OUT_9,
    #endif
    #ifdef GPIO_OUT_10
    GPIO_OUT_10,
    #endif
    #ifdef GPIO_OUT_11
    GPIO_OUT_11,
    #endif
    #ifdef GPIO_OUT_12
    GPIO_OUT_12,
    #endif
    #ifdef GPIO_OUT_13
    GPIO_OUT_13,
    #endif
    #ifdef GPIO_OUT_14
    GPIO_OUT_14,
    #endif
    #ifdef GPIO_OUT_15
    GPIO_OUT_15,
    #endif
};

static const uint32_t out_pin_cnt = sizeof(out_pin_maps)/sizeof(uint32_t);

/*******************************************************************************
 * @brief Init GPIO pins
 ******************************************************************************/
void gpio_drv_init(void)
{
    uint32_t pin_mask_all = 0ul;
    
    // Configure output pins
    for(int i = 0; i < out_pin_cnt; i++)
        pin_mask_all |= out_pin_maps[i];

    gpio_init_mask(pin_mask_all);
    gpio_set_dir_out_masked(pin_mask_all);
}
