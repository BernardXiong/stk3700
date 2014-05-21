/*
 * File      : drv_led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author        Notes
 * 2012-10-07     yiyue.fang    modified from LED driver code for Energy Micro 
 *                              EFM32_G8xx_STK starter kit
 */

#include "board.h"
#include "drv_led.h"

#define LEDPORT gpioPortE
#define LEDPIN  2

/**
 * This function light up LED
 *
 * @param led LED number (6)
 */
void led_set(int led)
{
    if (led == 0)
    {
        GPIO_PinOutSet(LEDPORT, (led + LEDPIN));
    }
}

/**
 * This function return LED status, on or off
 *
 * @param led LED number (0-1)
 */
int led_get(int led)
{
    int ret = 0;

    if (led == 0)
    {
        ret = GPIO_PinOutGet(LEDPORT, (led + LEDPIN));
    }

    return ret;
}

/**
 * This function turn off LED
 *
 * @param led LED number (0-1)
 */
void led_clear(int led)
{
    if (led == 0)
    {
        GPIO_PinOutClear(LEDPORT, (led + LEDPIN));
    }
}

/**
 * This function toggle LED, switch from on to off or vice versa
 *
 * @param led LED number (0-1)
 */
void led_toggle(int led)
{
    if (led == 0)
    {
        GPIO_PinOutToggle(LEDPORT, (led + LEDPIN));
    }
}

/**
 * This function light up LEDs according value of 2 least significat bits
 *
 * @param value Bit pattern
 */
void led_value(int value)
{
    /* Set the value directly using 0xc as a mask. */
    GPIO_PortOutSetVal(LEDPORT, (value << LEDPIN), 0x20);
}

/**
 * This function initialize LED interface
 */
void led_init(void)
{
    /* Enable the LED by default */
    GPIO_PinModeSet(LEDPORT, LEDPIN, gpioModePushPull, 0);
}
