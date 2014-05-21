/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author        Notes
 * 2010-12-21     onelife       first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <efm32.h>
#include <em_chip.h>
#include <em_cmu.h>
#include <em_rmu.h>
#include <em_dma.h>
#include <em_ebi.h>
#include <em_rtc.h>
#include <em_timer.h>
#include <em_gpio.h>
#include <em_acmp.h>
#include <em_adc.h>
#include <em_usart.h>
#include <em_leuart.h>
#include <em_i2c.h>

#include <rtthread.h>
#include <rthw.h>

/* SECTION: SWO */
//#define EFM32GG_USING_SWO                      /* enable Serial Wire Viewer Output */

/* SECTION: SYSTEM */
#define EFM32_SRAM_END                (SRAM_BASE + SRAM_SIZE)
#define EFM32_BASE_PRI_DEFAULT        (0x0UL << 5)
#define EFM32_IRQ_PRI_DEFAULT         (0x4UL << 5)


/* SECTION: CLOCK */
#define EFM32_HFXO_FREQUENCY          (48000000)

/* SECTION: SERIAL */
#define EFM32GG_USING_LEUART0       /* enable leuart0 */
#define EFM32GG_USING_LEUART1       /* enable leuart1 */

#define LEUART0_LOCATION            0          /* 0:PD4 PD5; */
#define LEUART1_LOCATION            0          /* 0:PC6 PC7; */
#define UART0_LOCATION              0          /* 0:PF6 PF7; */
#define UART1_LOCATION              3          /* 3:PE2 PE3; */
#define USART0_LOCATION             5          /* PC0:TX,PC1:RX */
#define USART1_LOCATION             2          /* PD7:TX,PD6:RX */


/* SECTION: EXT INT */
#define EFM32GG_USING_EINT          /* enable external interrupt */

void rt_hw_board_init(void);

#endif /*__BOARD_H__ */
