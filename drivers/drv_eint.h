/*
 * File      : drv_eint.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author         Notes
 * 2012-11-14     heyuanjie87    the first version
 */

#ifndef __DRV_EINT_H__
#define __DRV_EINT_H__

#include <rtthread.h>
#include "board.h"

#define PA    gpioPortA
#define PB    gpioPortB
#define PC    gpioPortC
#define PD    gpioPortD
#define PE    gpioPortE
#define PF    gpioPortF

typedef enum
{
    eint_trigger_none    = 0,
    eint_trigger_rising  = 1,
    eint_trigger_falling = 2,
    eint_trigger_double  = 3
} eint_trigger_typedef;

void eint_init(void);
void eint_handler_install(rt_uint32_t       pin,
                          rt_isr_handler_t  new_handler,
                          rt_isr_handler_t *old_handler);
void eint_trigger_config(GPIO_Port_TypeDef    port,
                         rt_uint32_t          pin,
                         eint_trigger_typedef eint_trigger);
void eint_enable(rt_uint32_t pin);
void eint_disable(rt_uint32_t pin);
eint_trigger_typedef eint_trigger_get(rt_uint32_t pin);

#endif /* __DRV_EINT_H__ */
