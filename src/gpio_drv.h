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
#ifndef GPIO_DRV_H
#define GPIO_DRV_H

//******************************************************************************
// Includes
//******************************************************************************

//******************************************************************************
// Defines
//******************************************************************************

// Pi Pico Pinout
// 
// UART0 TX, I2C0 SDA, SPI0 RX,  GP0 ->|1        40|<- VBUS
// UART0 RX, I2C0 SCL, SPI0 CSn, GP1 ->|2        39|<- VSYS
//                               GND ->|3        38|<- GND
//           I2C1 SDA, SPI0 SCK, GP2 ->|4        37|<- 3V3_EN
//           I2C1 SCL, SPI0 TX,  GP3 ->|5        36|<- 3V3(OUT)
// UART1 TX, I2C0 SDA, SPI0 RX,  GP4 ->|6        35|<- ADC_VREF
// UART1 RX, I2C0 SCL, SPI0 CSn, GP5 ->|7        34|<- GP28
//                               GND ->|8        33|<- GND
//           I2C1 SDA, SPI0 SCK, GP6 ->|9        32|<- GP27
//           I2C1 SCL, SPI0 TX,  GP7 ->|10       31|<- GP26
// UART1 TX, I2C0 SDA, SPI1 RX,  GP8 ->|11       30|<- RUN
// UART1 RX, I2C0 SCL, SPI1 CSn, GP9 ->|12       29|<- GP22
//                               GND ->|13       28|<- GND
//           I2C1 SDA, SPI1 SCK, GP10->|14       27|<- GP21
//           I2C1 SCL, SPI1 TX,  GP11->|15       26|<- GP20
// UART0 TX, I2C0 SDA, SPI1 RX,  GP12->|16       25|<- GP19
// UART0 RX, I2C0 SCL, SPI1 CSn, GP13->|17       24|<- GP18
//                               GND ->|18       23|<- GND
//           I2C1 SDA, SPI1 SCK, GP14->|19       22|<- GP17
//           I2C1 SCL, SPI1 TX,  GP15->|20       21|<- GP16

#define GPIO_OUT_SET(x)     gpio_set_mask((x))
#define GPIO_OUT_CLR(x)     gpio_clr_mask((x))
#define GPIO_OUT_TGL(x)     gpio_xor_mask((x))

// Pins definitions
#define GPIO_GP0    0x00000001ul
#define GPIO_GP1    0x00000002ul
#define GPIO_GP2    0x00000004ul
#define GPIO_GP3    0x00000008ul
#define GPIO_GP4    0x00000010ul
#define GPIO_GP5    0x00000020ul
#define GPIO_GP6    0x00000040ul
#define GPIO_GP7    0x00000080ul
#define GPIO_GP8    0x00000100ul
#define GPIO_GP9    0x00000200ul
#define GPIO_GP10   0x00000400ul
#define GPIO_GP11   0x00000800ul
#define GPIO_GP12   0x00001000ul
#define GPIO_GP13   0x00002000ul
#define GPIO_GP14   0x00004000ul
#define GPIO_GP15   0x00008000ul

// Pins defined as outputs
//#define GPIO_OUT_0  GPIO_GP0
//#define GPIO_OUT_1  GPIO_GP1
#define GPIO_OUT_2  GPIO_GP2
#define GPIO_OUT_3  GPIO_GP3
#define GPIO_OUT_4  GPIO_GP4
#define GPIO_OUT_5  GPIO_GP5
#define GPIO_OUT_6  GPIO_GP6
#define GPIO_OUT_7  GPIO_GP7
//#define GPIO_OUT_8  GPIO_GP8
//#define GPIO_OUT_9  GPIO_GP9
#define GPIO_OUT_10 GPIO_GP10
#define GPIO_OUT_11 GPIO_GP11
#define GPIO_OUT_12 GPIO_GP12
#define GPIO_OUT_13 GPIO_GP13
//#define GPIO_OUT_14 GPIO_GP14
//#define GPIO_OUT_15 GPIO_GP15

// TP
#define TP_SET      GPIO_OUT_SET
#define TP_CLR      GPIO_OUT_CLR
#define TP_TGL      GPIO_OUT_TGL

#define TP0         GPIO_OUT_2
#define TP1         GPIO_OUT_3
#define TP2         GPIO_OUT_4
#define TP3         GPIO_OUT_5
#define TP4         GPIO_OUT_6
#define TP5         GPIO_OUT_7
#define TP6         GPIO_OUT_10
#define TP7         GPIO_OUT_11
#define TP8         GPIO_OUT_12
#define TP9         GPIO_OUT_13

//******************************************************************************
// Exported Functions
//******************************************************************************

// Init GPIO pins
void gpio_drv_init();

//******************************************************************************
#endif /* GPIO_DRV_H */
