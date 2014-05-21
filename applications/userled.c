/*
 * File      : userled.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author        Notes
 * 2012-09-20     heyuanjie     first version
 * 2012-09-29     lgnq          re-fromat the coding style
 */

#include <rtthread.h>
#include "drv_led.h"

/**
 * This function is the entry of the led thread
 *
 * @param param parameter of the led thread
 */
static void rt_userled_thread_init(void *param)
{
    led_init();

    /* leds blink */
    while (1)
    {
		led_set(0);
		rt_thread_delay(RT_TICK_PER_SECOND/4);

		led_clear(0);
		rt_thread_delay(RT_TICK_PER_SECOND);
    }
}

/**
 * This function will create and start a led thread
 */
void rt_hw_userled_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("led",
                           rt_userled_thread_init,
                           RT_NULL,
                           512,
                           8,
                           10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
}
