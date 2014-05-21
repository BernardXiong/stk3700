/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author        Notes
 * 2010-11-20     onelife       first version
 * 2013-02-20     Bernard       Add log trace initialization.
 */

#include <board.h>

#ifdef RT_USING_FINSH
#include <shell.h>
#endif

extern void rt_hw_userled_init(void);

void rt_init_thread_entry(void* parameter)
{
    rt_hw_userled_init();

	/* init finsh */
#ifdef RT_USING_FINSH
    finsh_system_init();
#endif /* RT_USING_FINSH */
}

/**
 * This function create and start the init thread
 */
int rt_application_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("init",
                           rt_init_thread_entry,
                           RT_NULL,
                           2048,
                           8,
                           20);
    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}
