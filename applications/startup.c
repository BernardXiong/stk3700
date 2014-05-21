/*
 * File      : startup.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author        Notes
 * 2010-11-20     onelife       first version
 * 2012-09-29     lgnq          re-fromat the coding style
 * 2012-10-09     bernard       remove the deprecated API invoking.
 */

/**
 * @addtogroup EFM32
 */

/*@{*/

#include "board.h"
#include <rtthread.h>
#include "drv_uart.h"

#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN	((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define HEAP_BEGIN	(__segment_end("HEAP"))
#else
extern int __bss_end;
#define HEAP_BEGIN	((void *)&__bss_end)
#endif

extern int rt_application_init(void);

/**
 * This function will startup RT-Thread RTOS.
 */
void rtthread_startup(void)
{
    /* initialize board */
    rt_hw_board_init();

    /* we need to init the heap in the very first because some device
     * initialization will use malloc */
#ifdef RT_USING_HEAP
    /* initialize memory system */
    rt_system_heap_init(HEAP_BEGIN, (void*)EFM32_SRAM_END);
#endif

#ifdef RT_USING_SERIAL
    /* Initialize USART */
#if (defined(EFM32GG_USING_USART0) || defined(EFM32GG_USING_USART1) || \
     defined(EFM32GG_USING_UART0) || defined(EFM32GG_USING_UART1)  || \
     defined(EFM32GG_USING_LEUART0) || defined(EFM32GG_USING_LEUART1))
    rt_hw_serial_init();
#endif
#endif

    /* Setup Console */
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);

    /* show version */
    rt_show_version();

    /* init timer system */
    rt_system_timer_init();

    /* init scheduler system */
    rt_system_scheduler_init();

    /* init idle thread */
    rt_thread_idle_init();

    /* init application */
    rt_application_init();

    /* start scheduler */
    rt_system_scheduler_start();

    /* never reach here */
    return ;
}

int main(void)
{
    /* disable interrupt first */
    rt_hw_interrupt_disable();

    /* init system setting */
    SystemInit();

    /* startup RT-Thread RTOS */
    rtthread_startup();

    return 0;
}

/*@}*/
