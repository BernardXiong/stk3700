/*
 * File      : board.c
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

#include "board.h"
#include <drv_eint.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define IS_NVIC_VECTTAB(VECTTAB) 		(((VECTTAB) == RAM_MEM_BASE) || \
										((VECTTAB) == FLASH_MEM_BASE))
#define IS_NVIC_OFFSET(OFFSET) 			((OFFSET) < 0x000FFFFF)

/***************************************************************************//**
 * @addtogroup SysTick_clock_source
 * @{
 ******************************************************************************/
#define SysTick_CLKSource_HCLK_Div8		((uint32_t)0xFFFFFFFB)
#define SysTick_CLKSource_HCLK			((uint32_t)0x00000004)
#define IS_SYSTICK_CLK_SOURCE(SOURCE)	(((SOURCE) == SysTick_CLKSource_HCLK) || \
										((SOURCE) == SysTick_CLKSource_HCLK_Div8))
/***************************************************************************//**
 * @}
 ******************************************************************************/

void rt_hw_driver_init(void);

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/***************************************************************************//**
 * @brief
 *   Set the allocation and offset of the vector table
 *
 * @details
 *
 * @note
 *
 * @param[in] NVIC_VectTab
 *	 Indicate the vector table is allocated in RAM or ROM
 *
 * @param[in] Offset
 *   The vector table offset
 ******************************************************************************/
static void NVIC_SetVectorTable(
	rt_uint32_t NVIC_VectTab,
	rt_uint32_t Offset)
{
	/* Check the parameters */
	RT_ASSERT(IS_NVIC_VECTTAB(NVIC_VectTab));
	RT_ASSERT(IS_NVIC_OFFSET(Offset));

	SCB->VTOR = NVIC_VectTab | (Offset & (rt_uint32_t)0x1FFFFF80);
}

/***************************************************************************//**
 * @brief
 *   Configure the address of vector table
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
static void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
	/* Set the vector table allocated at 0x20000000 */
	NVIC_SetVectorTable(RAM_MEM_BASE, 0x0);
#else  /* VECT_TAB_FLASH  */
	/* Set the vector table allocated at 0x00000000 */
	NVIC_SetVectorTable(FLASH_MEM_BASE, 0x0);
#endif

	/* Set NVIC Preemption Priority Bits: 0 bit for pre-emption, 4 bits for
	   subpriority */
	NVIC_SetPriorityGrouping(0x7UL);

	/* Set Base Priority Mask Register */
	__set_BASEPRI(EFM32_BASE_PRI_DEFAULT);
}

/***************************************************************************//**
 * @brief
 *   Configure the SysTick clock source
 *
 * @details
 *
 * @note
 *
 * @param[in] SysTick_CLKSource
 *	 Specifies the SysTick clock source.
 *
 * @arg SysTick_CLKSource_HCLK_Div8
 * 	 AHB clock divided by 8 selected as SysTick clock source.
 *
 * @arg SysTick_CLKSource_HCLK
 *	 AHB clock selected as SysTick clock source.
 ******************************************************************************/
static void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource)
{
  /* Check the parameters */
  RT_ASSERT(IS_SYSTICK_CLK_SOURCE(SysTick_CLKSource));

  if (SysTick_CLKSource == SysTick_CLKSource_HCLK)
  {
    SysTick->CTRL |= SysTick_CLKSource_HCLK;
  }
  else
  {
    SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;
  }
}

/***************************************************************************//**
 * @brief
 *   Configure the SysTick for OS tick.
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
void  SysTick_Configuration(void)
{
	rt_uint32_t 	core_clock;
	rt_uint32_t 	cnts;

	core_clock = SystemCoreClockGet();
	cnts = core_clock / RT_TICK_PER_SECOND;

	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}

void setupSWO(void)
{
  /* Enable GPIO Clock. */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;
#if defined(_EFM32_GIANT_FAMILY) || defined(_EFM32_WONDER_FAMILY) || defined(_EFM32_LEOPARD_FAMILY)
  /* Set location 0 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

  /* Enable output on pin - GPIO Port F, Pin 2 */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
#else
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
#endif
  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= 1;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  DWT->CTRL = 0x400113FF;
  /* Set TPIU prescaler to 16. */
  TPI->ACPR = 0xf;
  /* Set protocol to NRZ */
  TPI->SPPR = 2;
  /* Disable continuous formatting */
  TPI->FFCR = 0x100;
  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;
}

/***************************************************************************//**
 * @brief
 *   Initialize the board.
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
void rt_hw_board_init(void)
{
	/* Chip errata */
	CHIP_Init();

	/* NVIC Configuration */
	NVIC_Configuration();

	/* Configure external oscillator */
	SystemHFXOClockSet(EFM32_HFXO_FREQUENCY);
	setupSWO();

	/* Configure the SysTick */
	SysTick_Configuration();

	rt_hw_driver_init();
}

/***************************************************************************//**
 * @brief
 *   Initialize the hardware drivers.
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
void rt_hw_driver_init(void)
{
    CMU_ClockEnable(cmuClock_HFPER, true);

    /* Enable GPIO */
    CMU_ClockEnable(cmuClock_GPIO, true);

    /* Enabling clock to the interface of the low energy modules */
    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Starting LFXO and waiting until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

	/* Select LFXO for Peripherals A */
    CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);

    /* Select LFXO for specified module (and wait for it to stabilize) */
#if (defined(EFM32GG_USING_LEUART0) || defined(EFM32GG_USING_LEUART1))
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
#endif

    /* Enable SWO */
#if defined(EFM32GG_USING_SWO)
    efm_swo_setup();
#endif

    /* Initialize external interrupt */
#ifdef EFM32GG_USING_EINT
    eint_init();
#endif
}

void SysTick_Handler(void)
{
	/* enter interrupt */
	rt_interrupt_enter();

	rt_tick_increase();

	/* leave interrupt */
	rt_interrupt_leave();
}
