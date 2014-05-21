/*
 * File      : drv_eint.c
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

#include "drv_eint.h"

/* EFM32GG external interrupt 0~15 */
#define MAX_EINT_HANDLER    16

#ifdef EFM32GG_USING_EINT
/* external interrupt handler table */
static rt_isr_handler_t _eint_isr_table[MAX_EINT_HANDLER];

static void default_handler(int vector, void* param)
{
    rt_kprintf("default external handler! vector:%d\r\n", vector);
}

static void EFM32GG_INT_Handler(void)
{
    rt_uint32_t pending;
    rt_isr_handler_t func;
    rt_uint32_t i;

    pending = GPIO->IF & GPIO->IEN;
    /* clear ext interrupt flag */
    GPIO->IFC = GPIO->IF & 0xFFFF;

    for (i=0; i<MAX_EINT_HANDLER; i++)
    {
        if (pending & (1UL<<i))
        {
            func = _eint_isr_table[i];
            if (func != RT_NULL)
            {
                func(i, RT_NULL);
            }
        }
    }
}

void GPIO_EVEN_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    EFM32GG_INT_Handler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void GPIO_ODD_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    EFM32GG_INT_Handler();

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * This function install the external interrupt handler.
 *
 * param interrupt pin.(0~15)
 * param new_handler the new handler.
 * param old_handler the pointer to save old handler.
 */
void eint_handler_install(rt_uint32_t       pin,
                          rt_isr_handler_t  new_handler,
                          rt_isr_handler_t *old_handler)
{
    if (pin < MAX_EINT_HANDLER)
    {
        if (old_handler != RT_NULL)
            *old_handler = _eint_isr_table[pin];
        _eint_isr_table[pin] = new_handler;
    }
}

/**
 * This function enable the external interrupt.
 *
 * param pin interrupt pin.(0~15)
 */
void eint_enable(rt_uint32_t pin)
{
    /* clear pending interrupt */
    GPIO->IFC = 1 << pin;

    /* enable interrupt */
    BITBAND_Peripheral(&(GPIO->IEN), pin, 1);
}

/**
 * This function disable the external interrupt.
 *
 * param pin interrupt pin.(0~15)
 */
void eint_disable(rt_uint32_t pin)
{
    /* disable interrupt */
    BITBAND_Peripheral(&(GPIO->IEN), pin, 0);
}

/**
 * This function config external interrupt trigger mode.
 *
 * param port GPIO port,PA PB PF etc.
 * param pin Interrupt pin.(0~15)
 * param eint_trigger eint_trigger_typedef
 */
void eint_trigger_config(GPIO_Port_TypeDef    port,
                         rt_uint32_t          pin,
                         eint_trigger_typedef eint_trigger)
{
    rt_bool_t rising = RT_FALSE, falling = RT_FALSE;

    if (eint_trigger == eint_trigger_rising)
    {
        rising = RT_TRUE;
    }
    else if (eint_trigger == eint_trigger_falling)
    {
        falling = RT_TRUE;
    }
    else if (eint_trigger == eint_trigger_double)
    {
        falling = RT_TRUE;
        rising  = RT_TRUE;
    }

    GPIO_PinModeSet(port, pin, gpioModeInput, 1);
    GPIO_IntConfig(port, pin, rising, falling, RT_FALSE);
}

/**
 * This function get the external interrupt trigger mode.
 *
 * param pin Interrupt pin.(0~15)
 *
 * return eint_trigger_typedef
 */
eint_trigger_typedef eint_trigger_get(rt_uint32_t pin)
{
    eint_trigger_typedef ret;
    rt_uint32_t rising, falling;

    rising  = GPIO->EXTIRISE;
    falling = GPIO->EXTIFALL;
    ret     = eint_trigger_none;
    pin     = 1 << pin;

    if (rising & falling & pin)
    {
        ret = eint_trigger_double;
    }
    else if (rising & pin)
    {
        ret = eint_trigger_rising;
    }
    else if (falling & pin)
    {
        ret = eint_trigger_falling;
    }

    return ret;
}

/**
 * This function initialize external interrupt.
 */
void eint_init(void)
{
    rt_uint32_t i;

    /* disable all external interrupt. */
    GPIO->IEN = 0;
    /* clear all interrupt flag */
    GPIO->IFC = 0xFFFF;
    /* set defualt external handler */
    for (i=0; i<MAX_EINT_HANDLER; i++)
    {
        _eint_isr_table[i] = default_handler;
    }
    /* enable NVIC */
    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

#endif /* EFM32GG_USING_EINT */
