/*
 * File      : drv_uart.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2008 - 2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author          Notes
 * 2012-10-04     heyuanjie87     the first version
 * 2012-10-20     heyuanjie87     add dma transmit
 */

#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#include "board.h"
#include <rtdevice.h>

struct efm32_uart
{
    void                   *uart_regs;
    /* irq number */
    IRQn_Type               rx_irq;
    IRQn_Type               tx_irq;
    rt_uint32_t             location;
#ifdef EFM32GG_UART_USING_DMA
    /* used for DMA transfer */
    rt_uint16_t             channel;
    rt_uint16_t             remain;
    DMA_CfgChannel_TypeDef *txchn_cfg;
    const char             *buffer;
#endif
};

void rt_hw_serial_init(void);
void rt_hw_serial_pm_init(void);

#endif /* __DRV_UART_H__ */
