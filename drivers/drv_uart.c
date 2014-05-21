/*
 * File      : drv_uart.c
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
 * 2012-12-03     heyuanjie87     block transmission when use DMA
 * 2013-02-25     heyuanjie87     not enable TX RX pin when init
 */

#include "drv_uart.h"

static rt_err_t uart_disable(struct efm32_uart *uart);
static rt_err_t uart_enable(struct efm32_uart *uart);

#ifdef RT_USING_SERIAL
#include <drivers/serial.h>

/* Using LEUART0 */
#ifdef EFM32GG_USING_LEUART0
static struct rt_serial_device serial5;
static struct serial_ringbuffer leuart0_int_rx;
static struct efm32_uart leuart0 =
{
    LEUART0,
    LEUART0_IRQn,
    LEUART0_IRQn,
    LEUART0_LOCATION,
};

void LEUART0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_hw_serial_isr(&serial5);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

/* Using LEUART1 */
#ifdef EFM32GG_USING_LEUART1
static struct rt_serial_device serial6;
static struct serial_ringbuffer leuart1_int_rx;
static struct efm32_uart leuart1 =
{
    LEUART1,
    LEUART1_IRQn,
    LEUART1_IRQn,
    LEUART1_LOCATION
};

void LEUART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_hw_serial_isr(&serial6);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

/* Using UART0 */
#ifdef EFM32GG_USING_UART0
static struct rt_serial_device serial0;
static struct serial_ringbuffer uart0_int_rx;
static struct efm32_uart uart0 =
{
    UART0,
    UART0_RX_IRQn,
    UART0_TX_IRQn,
    UART0_LOCATION
};

void UART0_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_hw_serial_isr(&serial0);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

/* Using UART1 */
#ifdef EFM32GG_USING_UART1
static struct rt_serial_device serial1;
static struct serial_ringbuffer uart1_int_rx;
static struct efm32_uart uart1 =
{
    UART1,
    UART1_RX_IRQn,
    UART1_TX_IRQn,
    UART1_LOCATION
};

void UART1_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_hw_serial_isr(&serial1);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

/* Using USART0 */
#ifdef EFM32GG_USING_USART0
static struct rt_serial_device serial2;
static struct serial_ringbuffer usart0_int_rx;
static struct efm32_uart usart0 =
{
    USART0,
    USART0_RX_IRQn,
    USART0_TX_IRQn,
    USART0_LOCATION
};

void USART0_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_hw_serial_isr(&serial2);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

/* Using USART1 */
#ifdef EFM32GG_USING_USART1
static struct rt_serial_device serial3;
static struct serial_ringbuffer usart1_int_rx;
static struct efm32_uart usart1 =
{
    USART1,
    USART1_RX_IRQn,
    USART1_TX_IRQn,
    USART1_LOCATION
};

void USART1_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    rt_hw_serial_isr(&serial3);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

/**
 * UART DMA transmit config
 */
#ifdef EFM32GG_UART_USING_DMA
static rt_size_t dma_transmit(struct rt_serial_device *serial,
                              const char              *buf,
                              rt_size_t                size);

static void completion_callback(unsigned int channel, bool primary, void *user)
{
    struct efm32_uart *uart;
    struct rt_serial_device *serial;

    serial = (struct rt_serial_device *)user;
    uart = (struct efm32_uart *)serial->parent.user_data;

    if (uart->remain == 0)
    {
        rt_hw_serial_dma_tx_isr(user);
    }
    else
    {
        dma_transmit(serial, uart->buffer, uart->remain);
    }
}

static DMA_CfgDescr_TypeDef uart_descCfgWr =
{
    dmaDataIncNone,    /* Do not increment destination address */
    dmaDataInc1,       /* Increment source address by one byte */
    dmaDataSize1,      /* Data size is one byte */
    dmaArbitrate1,     /* Rearbitrate for each byte recieved */
    0,                 /* No read/write source protection */
};

#ifdef EFM32GG_USING_LEUART0
static DMA_CB_TypeDef le0_dmaCb =
{
    completion_callback,
    &serial5,
    0
};

static DMA_CfgChannel_TypeDef le0tx_chnCfg =
{
    RT_FALSE,            /* Default priority */
    RT_TRUE,             /* Interrupt on transfer completion */
    DMAREQ_LEUART0_TXBL, /* Set LEUART0 TX buffer empty, as source of DMA signals */
    &le0_dmaCb           /* Transfer completion callback */
};
#endif

#ifdef EFM32GG_USING_LEUART1
static DMA_CB_TypeDef le1_dmaCb =
{
    completion_callback,
    &serial6,
    0
};

static DMA_CfgChannel_TypeDef le1tx_chnCfg =
{
    RT_FALSE,            /* Default priority */
    RT_TRUE,             /* Interrupt on transfer completion */
    DMAREQ_LEUART1_TXBL, /* Set LEUART0 TX buffer empty, as source of DMA signals */
    &le1_dmaCb           /* Transfer completion callback */
};
#endif

#ifdef EFM32GG_USING_UART0
static DMA_CB_TypeDef ut0_dmaCb =
{
    completion_callback,
    &serial0,
    0
};

static DMA_CfgChannel_TypeDef ut0tx_chnCfg =
{
    RT_FALSE,            /* Default priority */
    RT_TRUE,             /* Interrupt on transfer completion */
    DMAREQ_UART0_TXBL,   /* Set UART0 TX buffer empty, as source of DMA signals */
    &ut0_dmaCb           /* Transfer completion callback */
};
#endif

#ifdef EFM32GG_USING_UART1
static DMA_CB_TypeDef ut1_dmaCb =
{
    completion_callback,
    &serial1,
    0
};

static DMA_CfgChannel_TypeDef ut1tx_chnCfg =
{
    RT_FALSE,            /* Default priority */
    RT_TRUE,             /* Interrupt on transfer completion */
    DMAREQ_UART1_TXBL,   /* Set UART1 TX buffer empty, as source of DMA signals */
    &ut1_dmaCb           /* Transfer completion callback */
};
#endif

#ifdef EFM32GG_USING_USART0
static DMA_CB_TypeDef ust0_dmaCb =
{
    completion_callback,
    &serial2,
    0
};

static DMA_CfgChannel_TypeDef ust0tx_chnCfg =
{
    RT_FALSE,            /* Default priority */
    RT_TRUE,             /* Interrupt on transfer completion */
    DMAREQ_USART0_TXBL,  /* Set USART0 TX buffer empty, as source of DMA signals */
    &ust0_dmaCb          /* Transfer completion callback */
};
#endif

#ifdef EFM32GG_USING_USART1
static DMA_CB_TypeDef ust1_dmaCb =
{
    completion_callback,
    &serial3,
    0
};

static DMA_CfgChannel_TypeDef ust1tx_chnCfg =
{
    RT_FALSE,            /* Default priority */
    RT_TRUE,             /* Interrupt on transfer completion */
    DMAREQ_USART1_TXBL,  /* Set USART1 TX buffer empty, as source of DMA signals */
    &ust1_dmaCb          /* Transfer completion callback */
};
#endif

#endif

/**
 * The following is UART operations
 */
static rt_err_t usart_configure(struct rt_serial_device *serial,
                                struct serial_configure *cfg)
{
    struct efm32_uart *uart;
    USART_TypeDef *regs;
    RT_ASSERT(cfg != RT_NULL);

    uart = serial->parent.user_data;
    regs = uart->uart_regs;
    /* stop usart first */
    USART_IntDisable(regs, USART_IEN_RXDATAV);
    USART_Enable(regs, usartDisable);

    serial->config = *cfg;
    /* set baudrate */
    USART_BaudrateAsyncSet(regs, 0, cfg->baud_rate, usartOVS16);
    /* enable usart */
    USART_IntEnable(regs, USART_IEN_RXDATAV);
    USART_Enable(regs, usartEnable);

    return (RT_EOK);
}

static rt_err_t usart_control(struct rt_serial_device *serial,
                              int                      cmd,
                              void                    *arg)
{
    struct efm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        NVIC_DisableIRQ(uart->rx_irq);
        uart_disable(uart);
        break;

    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        NVIC_ClearPendingIRQ(uart->rx_irq);
        NVIC_SetPriority(uart->rx_irq, EFM32_IRQ_PRI_DEFAULT);
        NVIC_EnableIRQ(uart->rx_irq);
        uart_enable(uart);
        break;
    }

    return (RT_EOK);
}

static int usart_putc(struct rt_serial_device *serial, char c)
{
    struct efm32_uart *uart;

    RT_ASSERT(serial !=RT_NULL);
    uart = serial->parent.user_data;

    USART_Tx(uart->uart_regs, c);

    return (1);
}

static int usart_getc(struct rt_serial_device *serial)
{
    struct efm32_uart *uart;
    USART_TypeDef *regs;

    RT_ASSERT(serial != RT_NULL);
    uart = serial->parent.user_data;
    regs = uart->uart_regs;

    /* Some error occured */
    if (regs->IF & (USART_IF_PERR | USART_IF_RXOF))
    {
        regs->IFC = (regs->IF & (USART_IF_PERR | USART_IF_RXOF));

        return (-1);
    }

    if (regs->IF & USART_IF_RXDATAV)
    {
        return (regs->RXDATA);
    }
    else
    {
        return (-1);
    }
}

#ifdef EFM32GG_UART_USING_DMA
static rt_size_t dma_transmit(struct rt_serial_device *serial,
                              const char              *buf,
                              rt_size_t                size)
{
    volatile uint32_t *dst;
    struct efm32_uart *uart;

    uart = (struct efm32_uart *)serial->parent.user_data;
    if (uart == &leuart0 || uart == &leuart1)
    {
        dst = &((LEUART_TypeDef *)uart->uart_regs)->TXDATA;
    }
    else
    {
        dst = &((USART_TypeDef *)uart->uart_regs)->TXDATA;
    }

    /* the efm DMA can only transmit 1024 onece in basic mode */
    if ((size / 1024) > 0)
    {
        uart->remain = size - 1024;
        size = 1024;
    }
    else
    {
        uart->remain = 0;
    }
    uart->buffer = buf + 1024;

    DMA_CfgChannel(uart->channel, (void *)uart->txchn_cfg);
    DMA_CfgDescr(uart->channel, RT_TRUE, (void *)&uart_descCfgWr);
    DMA_ActivateBasic(uart->channel,
                      RT_TRUE,
                      RT_TRUE,
                      (void *)dst,
                      (void *)buf,
                      size - 1);

    return (size);
}
#endif

static struct rt_uart_ops usart_ops =
{
    usart_configure,
    usart_control,
    usart_putc,
    usart_getc,
#ifdef EFM32GG_UART_USING_DMA
    dma_transmit
#endif
};

static rt_err_t leuart_configure(struct rt_serial_device *serial,
                                 struct serial_configure *cfg)
{
    struct efm32_uart *uart;
    rt_uint32_t ctrl = 0;
    LEUART_TypeDef *regs;

    RT_ASSERT(cfg != RT_NULL);

    uart = serial->parent.user_data;
    regs = uart->uart_regs;
    /* stop usart first */
    LEUART_IntDisable(regs, LEUART_IEN_RXDATAV);
    LEUART_Enable(regs, leuartDisable);

    serial->config = *cfg;
    LEUART_BaudrateSet(regs, 0, cfg->baud_rate);

    /* set parity */
    if (cfg->parity == PARITY_ODD)
    {
        ctrl |= leuartOddParity;
    }
    else if (cfg->parity == PARITY_EVEN)
    {
        ctrl |= leuartEvenParity;
    }
    /* set Tx stop-bits,Rx stop-bits are fixed at 1 */
    if (cfg->stop_bits == STOP_BITS_2)
    {
        ctrl |= leuartStopbits2;
    }
    /* change setting */
    regs->CTRL &= ~0x1E;
    while(regs->SYNCBUSY & 1);
    regs->CTRL |= ctrl;
    while(regs->SYNCBUSY & 1);
    /* enable usart */
    LEUART_IntEnable(regs, LEUART_IEN_RXDATAV);
    LEUART_Enable(regs, leuartEnable);

    return (RT_EOK);
}

static rt_err_t leuart_control(struct rt_serial_device *serial,
                               int                      cmd,
                               void                    *arg)
{
    struct efm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        NVIC_DisableIRQ(uart->rx_irq);
        uart_disable(uart);
        break;

    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        NVIC_ClearPendingIRQ(uart->rx_irq);
        NVIC_SetPriority(uart->rx_irq, EFM32_IRQ_PRI_DEFAULT);
        NVIC_EnableIRQ(uart->rx_irq);
        uart_enable(uart);
        break;
    }

    return (RT_EOK);
}

static int leuart_putc(struct rt_serial_device *serial, char c)
{
    LEUART_TypeDef *leuart;
    struct efm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct efm32_uart *)serial->parent.user_data;
    leuart = (LEUART_TypeDef *)uart->uart_regs;
    LEUART_Tx(leuart, c);

    return (1);
}

static int leuart_getc(struct rt_serial_device *serial)
{
    struct efm32_uart *uart;
    LEUART_TypeDef *regs;

    RT_ASSERT(serial != RT_NULL);
    uart = serial->parent.user_data;
    regs = uart->uart_regs;

    /* Some error occured */
    if (regs->IF & (LEUART_IF_PERR | LEUART_IF_RXOF))
    {
        regs->IFC = (regs->IF & (LEUART_IF_PERR | LEUART_IF_RXOF));

        return (-1);
    }

    if (regs->IF & LEUART_IF_RXDATAV)
    {
        return (regs->RXDATA);
    }
    else
    {
        return (-1);
    }
}

static struct rt_uart_ops leuart_ops =
{
    leuart_configure,
    leuart_control,
    leuart_putc,
    leuart_getc,
#ifdef EFM32GG_UART_USING_DMA
    dma_transmit
#endif
};

static void efm32gg_usart_init(struct efm32_uart *uart)
{
    USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;
    USART_TypeDef *regs;
    CMU_Clock_TypeDef clock;

    switch ((rt_uint32_t)uart->uart_regs)
    {
    case (rt_uint32_t)UART0:
        clock = cmuClock_UART0;
        break;
    case (rt_uint32_t)UART1:
        clock = cmuClock_UART1;
        break;
    case (rt_uint32_t)USART0:
        clock = cmuClock_USART0;
        break;
    case (rt_uint32_t)USART1:
        clock = cmuClock_USART1;
        break;
    default:
        return;
    }

    regs = (USART_TypeDef *)uart->uart_regs;
    CMU_ClockEnable(clock, true);
    USART_InitAsync(regs, &uartInit);
    USART_IntEnable(regs, USART_IEN_RXDATAV);
}

static void efm32gg_leuart_init(struct efm32_uart *uart)
{
    LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;
    LEUART_TypeDef *regs;
    CMU_Clock_TypeDef clock;

    switch ((rt_uint32_t) uart->uart_regs)
    {
    case (rt_uint32_t) LEUART0:
        clock = cmuClock_LEUART0;
        break;
    case (rt_uint32_t) LEUART1:
        clock = cmuClock_LEUART1;
        break;
    default:
        return;
    }

    regs = uart->uart_regs;
    /* Enable LEUART clock */
    CMU_ClockEnable(clock, true);
    /* Do not prescale clock */
    CMU_ClockDivSet(clock, cmuClkDiv_1);
    LEUART_Init(regs, &init);
    LEUART_IntEnable(regs, LEUART_IEN_RXDATAV);
}

/*
 * uart enable/disable used to reduce the energy consumption of the UART0
 */
static rt_err_t uart_enable(struct efm32_uart *uart)
{
    rt_err_t ret = RT_EOK;
    rt_uint8_t port;
    rt_uint8_t pin_tx, pin_rx;

    switch((rt_uint32_t) uart->uart_regs)
    {
#ifdef EFM32GG_USING_UART0
    case (rt_uint32_t) UART0:
        port = AF_UART0_TX_PORT(uart->location);
        pin_tx = AF_UART0_TX_PIN(uart->location);
        pin_rx = AF_UART0_RX_PIN(uart->location);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_tx, gpioModePushPull, 1);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_rx, gpioModeInput, 1);
        CMU_ClockEnable(cmuClock_UART0, RT_TRUE);

        /* Enable RX and TX pins and set location */
        UART0->ROUTE = (1 << 1) | (1 << 0) | (uart->location << 8);
        break;
#endif

#ifdef EFM32GG_USING_UART1
    case (rt_uint32_t) UART1:
        port = AF_UART1_TX_PORT(uart->location);
        pin_tx = AF_UART1_TX_PIN(uart->location);
        pin_rx = AF_UART1_RX_PIN(uart->location);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_tx, gpioModePushPull, 1);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_rx, gpioModeInput, 1);
        CMU_ClockEnable(cmuClock_UART1, RT_TRUE);

        /* Enable RX and TX pins and set location */
        UART1->ROUTE = (1 << 1) | (1 << 0) | (uart->location << 8);
        break;
#endif

#ifdef EFM32GG_USING_USART0
    case (rt_uint32_t) USART0:
        port = AF_USART0_TX_PORT(uart->location);
        pin_tx = AF_USART0_TX_PIN(uart->location);
        pin_rx = AF_USART0_RX_PIN(uart->location);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_tx, gpioModePushPull, 1);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_rx, gpioModeInput, 1);
        CMU_ClockEnable(cmuClock_USART0, RT_TRUE);

        /* Enable RX and TX pins and set location */
        USART0->ROUTE = (1 << 1) | (1 << 0) | (uart->location << 8);
        break;
#endif

#ifdef EFM32GG_USING_USART1
    case (rt_uint32_t) USART1:
        port = AF_USART1_TX_PORT(uart->location);
        pin_tx = AF_USART1_TX_PIN(uart->location);
        pin_rx = AF_USART1_RX_PIN(uart->location);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_tx, gpioModePushPull, 1);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_rx, gpioModeInput, 1);
        CMU_ClockEnable(cmuClock_USART1, RT_TRUE);

        /* Enable RX and TX pins and set location */
        USART1->ROUTE = (1 << 1) | (1 << 0) | (uart->location << 8);
        break;
#endif

#ifdef EFM32GG_USING_LEUART0
    case (rt_uint32_t) LEUART0:
        port = AF_LEUART0_TX_PORT(uart->location);
        pin_tx = AF_LEUART0_TX_PIN(uart->location);
        pin_rx = AF_LEUART0_RX_PIN(uart->location);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_tx, gpioModePushPull, 1);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_rx, gpioModeInput, 1);
        CMU_ClockEnable(cmuClock_LEUART0, RT_TRUE);

        /* Enable RX and TX pins and set location */
        LEUART0->ROUTE = (1 << 1) | (1 << 0) | (uart->location << 8);
        break;
#endif

#ifdef EFM32GG_USING_LEUART1
    case (rt_uint32_t) LEUART1:
        port = AF_LEUART1_TX_PORT(uart->location);
        pin_tx = AF_LEUART1_TX_PIN(uart->location);
        pin_rx = AF_LEUART1_RX_PIN(uart->location);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_tx, gpioModePushPull, 1);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_rx, gpioModeInput, 1);
        CMU_ClockEnable(cmuClock_LEUART1, RT_TRUE);

        /* Enable RX and TX pins and set location */
        LEUART1->ROUTE = (1 << 1) | (1 << 0) | (uart->location << 8);
        break;
#endif

    default:
        ret = -RT_ERROR;
        break;
    }

    return (ret);
}

static rt_err_t uart_disable(struct efm32_uart *uart)
{
    rt_err_t ret = RT_EOK;
    CMU_Clock_TypeDef clock;
    rt_uint8_t port;
    rt_uint8_t pin_tx, pin_rx;

    switch((rt_uint32_t) uart->uart_regs)
    {
#ifdef EFM32GG_USING_UART0
    case (rt_uint32_t) UART0:
        port = AF_UART0_TX_PORT(uart->location);
        pin_tx = AF_UART0_TX_PIN(uart->location);
        pin_rx = AF_UART0_RX_PIN(uart->location);
        clock = cmuClock_UART0;

        /* disable TX RX pin */
        UART0->ROUTE &= ~3;
        break;
#endif

#ifdef EFM32GG_USING_UART1
    case (rt_uint32_t) UART1:
        port = AF_UART1_TX_PORT(uart->location);
        pin_tx = AF_UART1_TX_PIN(uart->location);
        pin_rx = AF_UART1_RX_PIN(uart->location);
        clock = cmuClock_UART1;

        /* disable TX RX pin */
        UART1->ROUTE &= ~3;
        break;
#endif

#ifdef EFM32GG_USING_USART0
    case (rt_uint32_t) USART0:
        port = AF_USART0_TX_PORT(uart->location);
        pin_tx = AF_USART0_TX_PIN(uart->location);
        pin_rx = AF_USART0_RX_PIN(uart->location);
        clock = cmuClock_USART0;

        /* disable TX RX pin */
        USART0->ROUTE &= ~3;
        break;
#endif

#ifdef EFM32GG_USING_USART1
    case (rt_uint32_t) USART1:
        port = AF_USART1_TX_PORT(uart->location);
        pin_tx = AF_USART1_TX_PIN(uart->location);
        pin_rx = AF_USART1_RX_PIN(uart->location);
        clock = cmuClock_USART1;

        /* disable TX RX pin */
        USART1->ROUTE &= ~3;
        break;
#endif

#ifdef EFM32GG_USING_LEUART0
    case (rt_uint32_t) LEUART0:
        port = AF_LEUART0_TX_PORT(uart->location);
        pin_tx = AF_LEUART0_TX_PIN(uart->location);
        pin_rx = AF_LEUART0_RX_PIN(uart->location);
        clock = cmuClock_LEUART0;

        /* disable TX RX pin */
        LEUART0->ROUTE &= ~3;
        break;
#endif

#ifdef EFM32GG_USING_LEUART1
    case (rt_uint32_t) LEUART1:
        port = AF_LEUART1_TX_PORT(uart->location);
        pin_tx = AF_LEUART1_TX_PIN(uart->location);
        pin_rx = AF_LEUART1_RX_PIN(uart->location);
        clock = cmuClock_LEUART1;
        /* disable TX RX pin */
        LEUART1->ROUTE &= ~3;
        break;
#endif

    default:
        ret = -RT_ERROR;
        break;
    }

    if (ret == RT_EOK)
    {
        /* Disnable UART clock */
        CMU_ClockEnable(clock, RT_FALSE);
        /* Disable GPIO of UART */
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_tx, gpioModeDisabled, 0);
        GPIO_PinModeSet((GPIO_Port_TypeDef) port, pin_rx, gpioModeDisabled, 0);
    }

    return (ret);
}

void rt_hw_serial_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef EFM32GG_USING_UART0
    /* Initialize uart0 controller */
    efm32gg_usart_init(&uart0);

    serial0.ops    = &usart_ops;
    serial0.int_rx = &uart0_int_rx;
    serial0.config = config;
#ifdef EFM32GG_UART_USING_DMA
    uart0.channel = EFM32GG_UART0_DMACH;
    uart0.txchn_cfg = &ut0tx_chnCfg;
#endif /* EFM32GG_UART_USING_DMA */

    /* Register UART0 device */
    rt_hw_serial_register(&serial0,
                          "uart0",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX,
                          &uart0);
#endif /* EFM32GG_USING_UART0 */

#ifdef EFM32GG_USING_UART1
    /* Initialize uart1 controller */
    efm32gg_usart_init(&uart1);

    serial1.ops    = &usart_ops;
    serial1.int_rx = &uart1_int_rx;
    serial1.config = config;
#ifdef EFM32GG_UART_USING_DMA
    uart1.channel = EFM32GG_UART1_DMACH;
    uart1.txchn_cfg = &ut1tx_chnCfg;
#endif /* EFM32GG_UART_USING_DMA */

    /* Register UART1 device */
    rt_hw_serial_register(&serial1,
                          "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          &uart1);
#endif /* EFM32GG_USING_UART1 */

#ifdef EFM32GG_USING_USART0
    /* Initialize usart0 controller */
    efm32gg_usart_init(&usart0);

    serial2.ops    = &usart_ops;
    serial2.int_rx = &usart0_int_rx;
    serial2.config = config;
#ifdef EFM32GG_UART_USING_DMA
    usart0.channel = EFM32GG_USART0_DMACH;
    usart0.txchn_cfg = &ust0tx_chnCfg;
#endif /* EFM32GG_UART_USING_DMA */

    /* Register UART0 device */
    rt_hw_serial_register(&serial2,
                          "usart0",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          &usart0);
#endif /* EFM32GG_USING_USART0 */

#ifdef EFM32GG_USING_USART1
    /* Initialize usart1 controller */
    efm32gg_usart_init(&usart1);

    serial3.ops    = &usart_ops;
    serial3.int_rx = &usart1_int_rx;
    serial3.config = config;
#ifdef EFM32GG_UART_USING_DMA
    usart1.channel = EFM32GG_USART1_DMACH;
    usart1.txchn_cfg = &ust1tx_chnCfg;
#endif /* EFM32GG_UART_USING_DMA */

    /* Register USART1 device */
    rt_hw_serial_register(&serial3,
                          "usart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          &usart1);
#endif /* EFM32GG_USING_USART1 */

    config.baud_rate = BAUD_RATE_9600;
#ifdef EFM32GG_USING_LEUART0
    /* Initialize leuart0 controller */
    efm32gg_leuart_init(&leuart0);

    serial5.ops    = &leuart_ops;
    serial5.int_rx = &leuart0_int_rx;
    serial5.config = config;
#ifdef EFM32GG_UART_USING_DMA
    leuart0.channel = EFM32GG_LEUART0_DMACH;
    leuart0.txchn_cfg = &le0tx_chnCfg;
#endif /* EFM32GG_UART_USING_DMA */

    rt_hw_serial_register(&serial5,
                          "leuart0",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          &leuart0);
#endif /* EFM32GG_USING_LEUART0 */

#ifdef EFM32GG_USING_LEUART1
    /* Initialize leuart1 controller */
    efm32gg_leuart_init(&leuart1);

    serial6.ops    = &leuart_ops;
    serial6.int_rx = &leuart1_int_rx;
    serial6.config = config;
#ifdef EFM32GG_UART_USING_DMA
    leuart1.channel = EFM32GG_LEUART1_DMACH;
    leuart1.txchn_cfg = &le1tx_chnCfg;
#endif /* EFM32GG_UART_USING_DMA */

    rt_hw_serial_register(&serial6,
                          "leuart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          &leuart1);
#endif  /* EFM32GG_USING_LEUART1 */
}

#endif /* RT_USING_SERIAL */
