/*
 * Copyright (c) 2018 zhtlab
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef _STM32L4_H_
#define _STM32L4_H_

#define __CM4_REV               1
#define __MPU_PRESENT           1
#define __FPU_PRESENT           1
#define __NVIC_PRIO_BITS        4
#define __Vendor_SysTickConfig  0

#ifndef __IO
#define __IO            volatile
#endif



#define __FPU_PRESENT           1



/*************************************************************
 * base address definitions
 */
#define AHB1_BASE               (0x40020000)
#define AHB2_BASE               (0x48000000)
#define APB1_BASE               (0x40000000)
#define APB2_BASE               (0x40010000)


/*************************************************************
 * interrupt request definitions
 */
enum irqNumbers {
  Reset_IRQn            =  (-15),
  NMI_IRQn              =  (-14),
  HardFault_IRQn        =  (-13),
  MemoryManagement_IRQn =  (-12),
  BusFault_IRQn         =  (-11),
  UsageFault_IRQn       =  (-10),
  SVCall_IRQn           =  ( -5),
  DebugMonitor_IRQn     =  ( -4),
  PendSV_IRQn           =  ( -2),
  SysTick_IRQn          =  ( -1),
  DMA1_CH1_IRQn         =  11,
  DMA1_CH2_IRQn,
  DMA1_CH3_IRQn,
  DMA1_CH4_IRQn,
  DMA1_CH5_IRQn,
  DMA1_CH6_IRQn,
  DMA1_CH7_IRQn,
  ADC1_IRQn,
  TIM1_BRK_TIM15_IRQn   =    24,
  TIM1_UP_TIM15_IRQn,
  TIM1_TRG_COM_IRQn,
  TIM1_CC_IRQn,
  TIM2_IRQn,
  TIM3_IRQn,
  I2C1_EV_IRQn            =  31,
  I2C1_ER_IRQn,
  I2C2_EV_IRQn,
  I2C2_ER_IRQn,
  SPI1_IRQn,
  SPI2_IRQn,
  USART1_IRQn,
  USART2_IRQn,
  USART3_IRQn,

  USB_FS_IRQn             =  67,

  I2C3_EV_IRQn            =  72,
  I2C3_ER_IRQn,
};

typedef int     IRQn_Type;

/***
 * GPIO
 */
#include        "../../STM/include/stm32Gpio.h"

#define GPIO_PTR	((stm32Dev_GPIO *) ((AHB2_BASE) + 0x0000))
/*#define GPIO		(GPIO_PTR[8])*/


/***
 * RCC
 */
#define RCC_CLOCK_LSI              32768
#define RCC_CLOCK_HSI           16000000
#define RCC_CLOCK_MSI_4MHZ       4000000

#include        "stm32l4Rcc.h"

#define RCC_PTR ((stm32Dev_RCC *) ((AHB1_BASE) + 0x1000))


#define CONFIG_STM32L4_POWERSW_FUNCBASE	(0)

struct _stStm32l4_IWDG {
  __IO uint32_t		kr;
#define KR_KEY_CLEAR		0xaaaa
#define KR_KEY_SET		0x5555
#define KR_KEY_START		0xcccc
  __IO uint32_t		pr;
#define PR_SHIFT		0
#define PR_DIV4			(0 << (PR_SHIFT))
#define PR_DIV8			(1 << (PR_SHIFT))
#define PR_DIV16		(2 << (PR_SHIFT))
#define PR_DIV32		(3 << (PR_SHIFT))
#define PR_DIV64		(4 << (PR_SHIFT))
#define PR_DIV128		(5 << (PR_SHIFT))
#define PR_DIV256		(6 << (PR_SHIFT))
#define PR_MASK			(7 << (PR_SHIFT))
  __IO uint32_t		rlr;
#define RLR_SHIFT		0
#define RLR_MASK		(0xfff << (RLR_SHIFT))
  __IO uint32_t		sr;
#define SR_PVU_SHIFT		0
#define SR_PVU_MASK		(1 << (SR_PVU_SHIFT))
#define SR_RVU_SHIFT		1
#define SR_RVU_MASK		(1 << (SR_RVU_SHIFT))
#define SR_WVU_SHIFT		2
#define SR_WVU_MASK		(1 << (SR_WVU_SHIFT))
  __IO uint32_t		winr;
#define WINR_SHIFT		0
#define WINR_MASK		(0xfff << (WINR_SHIFT))
};

#if 0
#define IWDG_PTR	((struct _stStm32l4_IWDG *) ((APB1_BASE) + 0x3000))
#define IWDG	(*IWDG_PTR)
#endif


/*******************************************
 * 05 PWR
 */
typedef struct {
  __IO uint32_t         CR1;
  __IO uint32_t         CR2;

#define PWR_CR2_USV_SHIFT       10
#define PWR_CR2_USV_MASK        (1 << (PWR_CR2_USV_SHIFT))
#define PWR_CR2_USV_NO          (0 << (PWR_CR2_USV_SHIFT))
#define PWR_CR2_USV_YES         (1 << (PWR_CR2_USV_SHIFT))
  __IO uint32_t         CR3;

  __IO uint32_t         CR4;

} stm32Dev_PWR;

#define PWR_PTR         ((stm32Dev_PWR *) (APB1_BASE + 0x7000))


/*******************************************
 * 07 CRS
 */

#include        "stm32Crs.h"

#define CRS_PTR        ((stm32Dev_CRS *) (APB1_BASE + 0x6000))


/*************************************************************
 * USART
 */
#include        "stm32Usart.h"

#define USART1_PTR      ((stm32DEV_USART *) ((APB2_BASE) + 0x3800))
#define USART_PTR       ((stm32Dev_USART *) ((APB1_BASE) + 0x4000))
#define USART2_PTR      (&USART_PTR[1])
#define USART3_PTR      (&USART_PTR[2])
#define USART4_PTR      (&USART_PTR[3])
#define LPUART1_PTR     ((stm32Dev_USART *) ((APB1_BASE) + 0x8000))


/*************************************************************
 * DMACH
 */
struct _stStm32l4_DMACH {
  __IO uint32_t	ccr;
#define CCR_MEM2MEM_SHIFT	14
#define CCR_MEM2MEM_MASK	(1 << (CCR_MEM2MEM_SHIFT))
#define CCR_MEM2MEM_NO		(0 << (CCR_MEM2MEM_SHIFT))
#define CCR_MEM2MEM_YES		(1 << (CCR_MEM2MEM_SHIFT))
#define CCR_PL_SHIFT		12
#define CCR_PL_MASK		(1 << (CCR_PL_SHIFT))
#define CCR_PL_LOW		(0 << (CCR_PL_SHIFT))
#define CCR_PL_MID		(1 << (CCR_PL_SHIFT))
#define CCR_PL_HID		(2 << (CCR_PL_SHIFT))
#define CCR_PL_VERYHIGH		(3 << (CCR_PL_SHIFT))
#define CCR_MSIZE_SHIFT		10
#define CCR_MSIZE_MASK		(1 << (CCR_MSIZE_SHIFT))
#define CCR_MSIZE_8BIT		(0 << (CCR_MSIZE_SHIFT))
#define CCR_MSIZE_16BIT		(1 << (CCR_MSIZE_SHIFT))
#define CCR_MSIZE_32BIT		(2 << (CCR_MSIZE_SHIFT))
#define CCR_MSIZE_RESERVED	(3 << (CCR_MSIZE_SHIFT))
#define CCR_PSIZE_SHIFT		8
#define CCR_PSIZE_MASK		(1 << (CCR_PSIZE_SHIFT))
#define CCR_PSIZE_8BIT		(0 << (CCR_PSIZE_SHIFT))
#define CCR_PSIZE_16BIT		(1 << (CCR_PSIZE_SHIFT))
#define CCR_PSIZE_32BIT		(2 << (CCR_PSIZE_SHIFT))
#define CCR_PSIZE_RESERVED	(3 << (CCR_PL_SHIFT))
#define CCR_MINC_SHIFT		7
#define CCR_MINC_MASK		(1 << (CCR_MINC_SHIFT))
#define CCR_MINC_NO		(0 << (CCR_MINC_SHIFT))
#define CCR_MINC_YES		(1 << (CCR_MINC_SHIFT))
#define CCR_PINC_SHIFT		6
#define CCR_PINC_MASK		(1 << (CCR_PINC_SHIFT))
#define CCR_PINC_NO		(0 << (CCR_PINC_SHIFT))
#define CCR_PINC_YES		(1 << (CCR_PINC_SHIFT))
#define CCR_CIRC_SHIFT		5
#define CCR_CIRC_MASK		(1 << (CCR_CIRC_SHIFT))
#define CCR_CIRC_NO		(0 << (CCR_CIRC_SHIFT))
#define CCR_CIRC_YES		(1 << (CCR_CIRC_SHIFT))
#define CCR_DIR_SHIFT		4
#define CCR_DIR_MASK		(1 << (CCR_DIR_SHIFT))
#define CCR_DIR_READ_PERI	(0 << (CCR_DIR_SHIFT))
#define CCR_DIR_READ_MEM	(1 << (CCR_DIR_SHIFT))
#define CCR_TEIE_SHIFT		3
#define CCR_TEIE_MASK		(1 << (CCR_TEIE_SHIFT))
#define CCR_TEIE_NO		(0 << (CCR_TEIE_SHIFT))
#define CCR_TEIE_YES		(1 << (CCR_TEIE_SHIFT))
#define CCR_HTIE_SHIFT		2
#define CCR_HTIE_MASK		(1 << (CCR_HTIE_SHIFT))
#define CCR_HTIE_NO		(0 << (CCR_HTIE_SHIFT))
#define CCR_HTIE_YES		(1 << (CCR_HTIE_SHIFT))
#define CCR_TCIE_SHIFT		1
#define CCR_TCIE_MASK		(1 << (CCR_TCIE_SHIFT))
#define CCR_TCIE_NO		(0 << (CCR_TCIE_SHIFT))
#define CCR_TCIE_YES		(1 << (CCR_TCIE_SHIFT))
#define CCR_EN_SHIFT		0
#define CCR_EN_MASK		(1 << (CCR_EN_SHIFT))
#define CCR_EN_NO		(0 << (CCR_EN_SHIFT))
#define CCR_EN_YES		(1 << (CCR_EN_SHIFT))

  __IO uint32_t	cndtr;
#define CNDTR_MASK		(0xffff)
  __IO uint32_t	cpar;
  __IO uint32_t	cmar;
  __IO uint32_t	reserved;
};
struct _stStm32l4_DMA {
  __IO uint32_t	isr;
#define ISR_TEIF_MASK(d)	(1 << (((d)<<2) + 3))
#define ISR_HTIF_MASK(d)	(1 << (((d)<<2) + 2))
#define ISR_TCIF_MASK(d)	(1 << (((d)<<2) + 1))
#define ISR_GIF_MASK(d)		(1 << (((d)<<2) + 0))

  __IO uint32_t	ifcr;
#define ISR_CTEIF_CLEAR(d)	(1 << (((d)<<2) + 3))
#define ISR_CHTIF_CLEAR(d)	(1 << (((d)<<2) + 2))
#define ISR_CTCIF_CLEAR(d)	(1 << (((d)<<2) + 1))
#define ISR_CGIF_CLEAR(d)	(1 << (((d)<<2) + 0))

  struct _stStm32l4_DMACH ch[7];
  __IO uint32_t	reserved90[5];
  __IO uint32_t	cselr;
#define CSELR_DMA_MASK(d)	(0xfUL << ((d)*4))
#define CSELR_DMA_SEL(d, x)	((x)   << ((d)*4))
  __IO uint32_t	reservedAC[213];
};
#define DMA_PTR	((struct _stStm32l4_DMA *)  ((AHB1_BASE) + 0x0000))
/*#define DMA	(DMA_PTR[2])*/

#define DMA_MODULE1	(0)
#define DMA_MODULE2	(1)

#define DMA_CH1		(0)
#define DMA_CH2		(1)
#define DMA_CH3		(2)
#define DMA_CH4		(3)
#define DMA_CH5		(4)
#define DMA_CH6		(5)
#define DMA_CH7		(6)



struct _stStm32l4_FLASH {
  __IO uint32_t		acr;
#define FLASH_LATENCY_SHIFT	(0)
#define FLASH_LATENCY_NONE (0 << (FLASH_LATENCY_SHIFT))
#define FLASH_LATENCY_1CLK	(1 << (FLASH_LATENCY_SHIFT))
#define FLASH_LATENCY_2CLK	(2 << (FLASH_LATENCY_SHIFT))
#define FLASH_LATENCY_3CLK	(3 << (FLASH_LATENCY_SHIFT))
#define FLASH_LATENCY_4CLK	(4 << (FLASH_LATENCY_SHIFT))

  __IO uint32_t		pdkeyr;
#define PDKEYR_KEY1	0x04152637
#define PDKEYR_KEY2	0xfafbfcfd
  __IO uint32_t		keyr;
#define KEYR_KEY1	0x45670123
#define KEYR_KEY2	0xcdef89ab
  __IO uint32_t		optkeyr;
#define OPTKEYR_KEY1	0x08192a3b
#define OPTKEYR_KEY2	0x4c5d6e7f

  __IO uint32_t		sr;
#define SR_PEMPTY_SHIFT		17
#define SR_PEMPTY_MASK		(1 << (SR_PEMPTY_SHIFT))
#define SR_PEMPTY_NO		(0 << (SR_PEMPTY_SHIFT))
#define SR_PEMPTY_YES		(1 << (SR_PEMPTY_SHIFT))
#define SR_BSY_SHIFT		16
#define SR_BSY_MASK		(1 << (SR_BSY_SHIFT))
#define SR_BSY_NO		(0 << (SR_BSY_SHIFT))
#define SR_BSY_YES		(1 << (SR_BSY_SHIFT))
#define SR_OPTVERR_SHIFT	15	
#define SR_OPTVERR_MASK		(1 << (SR_OPTVERR_SHIFT))
#define SR_OPTVERR_NO		(0 << (SR_OPTVERR_SHIFT))
#define SR_OPTVERR_YES		(1 << (SR_OPTVERR_SHIFT))
#define SR_RDERR_SHIFT		14
#define SR_RDERR_MASK		(1 << (SR_RDERR_SHIFT))
#define SR_RDERR_NO		(0 << (SR_RDERR_SHIFT))
#define SR_RDERR_YES		(1 << (SR_RDERR_SHIFT))
#define SR_FASTERR_SHIFT	9	
#define SR_FASTERR_MASK		(1 << (SR_FASTERR_SHIFT))
#define SR_FASTERR_NO		(0 << (SR_FASTERR_SHIFT))
#define SR_FASTERR_YES		(1 << (SR_FASTERR_SHIFT))
#define SR_MISERR_SHIFT		8
#define SR_MISERR_MASK		(1 << (SR_MISERR_SHIFT))
#define SR_MISERR_NO		(0 << (SR_MISERR_SHIFT))
#define SR_MISERR_YES		(1 << (SR_MISERR_SHIFT))
#define SR_PGSERR_SHIFT		7
#define SR_PGSERR_MASK		(1 << (SR_PGSERR_SHIFT))
#define SR_PGSERR_NO		(0 << (SR_PGSERR_SHIFT))
#define SR_PGSERR_YES		(1 << (SR_PGSERR_SHIFT))
#define SR_SIZERR_SHIFT		6
#define SR_SIZERR_MASK		(1 << (SR_SIZERR_SHIFT))
#define SR_SIZERR_NO		(0 << (SR_SIZERR_SHIFT))
#define SR_SIZERR_YES		(1 << (SR_SIZERR_SHIFT))
#define SR_PGAERR_SHIFT		5
#define SR_PGAERR_MASK		(1 << (SR_PGAERR_SHIFT))
#define SR_PGAERR_NO		(0 << (SR_PGAERR_SHIFT))
#define SR_PGAERR_YES		(1 << (SR_PGAERR_SHIFT))
#define SR_WRPERR_SHIFT		4
#define SR_WRPERR_MASK		(1 << (SR_WRPERR_SHIFT))
#define SR_WRPERR_NO		(0 << (SR_WRPERR_SHIFT))
#define SR_WRPERR_YES		(1 << (SR_WRPERR_SHIFT))
#define SR_PROGERR_SHIFT	3
#define SR_PROGERR_MASK		(1 << (SR_PROGERR_SHIFT))
#define SR_PROGERR_NO		(0 << (SR_PROGERR_SHIFT))
#define SR_PROGERR_YES		(1 << (SR_PROGERR_SHIFT))
#define SR_OPERR_SHIFT		1
#define SR_OPERR_MASK		(1 << (SR_OPERR_SHIFT))
#define SR_OPERR_NO		(0 << (SR_OPERR_SHIFT))
#define SR_OPERR_YES		(1 << (SR_OPERR_SHIFT))
#define SR_EOP_SHIFT		0
#define SR_EOP_MASK		(1 << (SR_EOP_SHIFT))
#define SR_EOP_NO		(0 << (SR_EOP_SHIFT))
#define SR_EOP_YES		(1 << (SR_EOP_SHIFT))

  __IO uint32_t		cr;
#define CR_LOCK_SHIFT		31
#define CR_LOCK_MASK		(1UL << (CR_LOCK_SHIFT))
#define CR_LOCK_NO		(0UL << (CR_LOCK_SHIFT))
#define CR_LOCK_YES		(1UL << (CR_LOCK_SHIFT))
#define CR_OPTLOCK_SHIFT	30
#define CR_OPTLOCK_MASK		(1 << (CR_OPTLOCK_SHIFT))
#define CR_OPTLOCK_NO		(0 << (CR_OPTLOCK_SHIFT))
#define CR_OPTLOCK_YES		(1 << (CR_OPTLOCK_SHIFT))
#define CR_OBL_LAUNCH_SHIFT	27
#define CR_OBL_LAUNCH_MASK	(1 << (CR_OBL_LAUNCH_SHIFT))
#define CR_OBL_LAUNCH_NO	(0 << (CR_OBL_LAUNCH_SHIFT))
#define CR_OBL_LAUNCH_YES	(1 << (CR_OBL_LAUNCH_SHIFT))
#define CR_RDERRIE_SHIFT	26
#define CR_RDERRIE_MASK		(1 << (CR_RDERRIE_SHIFT))
#define CR_RDERRIE_NO		(0 << (CR_RDERRIE_SHIFT))
#define CR_RDERRIE_YES		(1 << (CR_RDERRIE_SHIFT))
#define CR_ERRIE_SHIFT		25
#define CR_ERRIE_MASK		(1 << (CR_ERRIE_SHIFT))
#define CR_ERRIE_NO		(0 << (CR_ERRIE_SHIFT))
#define CR_ERRIE_YES		(1 << (CR_ERRIE_SHIFT))
#define CR_EOPIE_SHIFT		24
#define CR_EOPIE_MASK		(1 << (CR_EOPIE_SHIFT))
#define CR_EOPIE_NO		(0 << (CR_EOPIE_SHIFT))
#define CR_EOPIE_YES		(1 << (CR_EOPIE_SHIFT))
#define CR_FSTPG_SHIFT		18
#define CR_FSTPG_MASK		(1 << (CR_FSTPG_SHIFT))
#define CR_FSTPG_NO		(0 << (CR_FSTPG_SHIFT))
#define CR_FSTPG_YES		(1 << (CR_FSTPG_SHIFT))
#define CR_OPTSTRT_SHIFT	17
#define CR_OPTSTRT_MASK		(1 << (CR_OPTSTRT_SHIFT))
#define CR_OPTSTRT_NO		(0 << (CR_OPTSTRT_SHIFT))
#define CR_OPTSTRT_YES		(1 << (CR_OPTSTRT_SHIFT))
#define CR_STRT_SHIFT		16
#define CR_STRT_MASK		(1 << (CR_STRT_SHIFT))
#define CR_STRT_NO		(0 << (CR_STRT_SHIFT))
#define CR_STRT_YES		(1 << (CR_STRT_SHIFT))
#define CR_PNB_SHIFT		3
#define CR_PNB_MASK		(0xff << (CR_PNB_SHIFT))
#define CR_PNB_SECT(x)		((x) << (CR_PNB_SHIFT))
#define CR_MER1_SHIFT		2
#define CR_MER1_MASK		(1 << (CR_MER1_SHIFT))
#define CR_MER1_NO		(0 << (CR_MER1_SHIFT))
#define CR_MER1_YES		(1 << (CR_MER1_SHIFT))
#define CR_PER_SHIFT		1
#define CR_PER_MASK		(1 << (CR_PER_SHIFT))
#define CR_PER_NO		(0 << (CR_PER_SHIFT))
#define CR_PER_YES		(1 << (CR_PER_SHIFT))
#define CR_PG_SHIFT		0
#define CR_PG_MASK		(1 << (CR_PG_SHIFT))
#define CR_PG_NO		(0 << (CR_PG_SHIFT))
#define CR_PG_YES		(1 << (CR_PG_SHIFT))

  __IO uint32_t		eccr;
  __IO uint32_t		optr;
  __IO uint32_t		pcrop1st;
  __IO uint32_t		pcrop1er;
  __IO uint32_t		wrp1ar;
  __IO uint32_t		wrp1br;
};
#define FLASH_PTR	((struct _stStm32l4_FLASH *) ((AHB1_BASE) + 0x2000))

/****
 * TIM
 */
#include        "../../STM/include/stm32Tim.h"

#define TIM1_PTR	((stm32Dev_TIM *) (APB2_BASE + 0x2c00))
#define TIM2_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x0000))
#define TIM7_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x1000))
#define TIM8_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x1400))
#define TIM15_PTR	((stm32Dev_TIM *) (APB2_BASE + 0x4000))
#define TIM16_PTR	((stm32Dev_TIM *) (APB2_BASE + 0x4400))



/*******************************************
 * 16 ADC
 */
typedef struct {
  __IO uint32_t         ISR;            /* 0x00 */
#define ADC_ISR_OVR_SHIFT       (4)
#define ADC_ISR_OVR_MASK        (1 << (ADC_ISR_OVR_SHIFT))
#define ADC_ISR_OVR_NO          (0 << (ADC_ISR_OVR_SHIFT))
#define ADC_ISR_OVR_YES         (1 << (ADC_ISR_OVR_SHIFT))
#define ADC_ISR_EOS_SHIFT       (3)
#define ADC_ISR_EOS_MASK        (1 << (ADC_ISR_EOS_SHIFT))
#define ADC_ISR_EOS_NO          (0 << (ADC_ISR_EOS_SHIFT))
#define ADC_ISR_EOS_YES         (1 << (ADC_ISR_EOS_SHIFT))
#define ADC_ISR_EOC_SHIFT       (2)
#define ADC_ISR_EOC_MASK        (1 << (ADC_ISR_EOC_SHIFT))
#define ADC_ISR_EOC_NO          (0 << (ADC_ISR_EOC_SHIFT))
#define ADC_ISR_EOC_YES         (1 << (ADC_ISR_EOC_SHIFT))
#define ADC_ISR_EOSMP_SHIFT     (1)
#define ADC_ISR_EOSMP_MASK      (1 << (ADC_ISR_EOSMP_SHIFT))
#define ADC_ISR_EOSMP_NO        (0 << (ADC_ISR_EOSMP_SHIFT))
#define ADC_ISR_EOSMP_YES       (1 << (ADC_ISR_EOSMP_SHIFT))
#define ADC_ISR_ADRDY_SHIFT     (0)
#define ADC_ISR_ADRDY_MASK      (1 << (ADC_ISR_ADRDY_SHIFT))
#define ADC_ISR_ADRDY_NO        (0 << (ADC_ISR_ADRDY_SHIFT))
#define ADC_ISR_ADRDY_YES       (1 << (ADC_ISR_ADRDY_SHIFT))
  __IO uint32_t         IER;            /* 0x04 */
  __IO uint32_t         CR;             /* 0x08 */
#define ADC_CR_DEEPPWD_SHIFT    (29)
#define ADC_CR_DEEPPWD_MASK     (1 << (ADC_CR_DEEPPWD_SHIFT))
#define ADC_CR_DEEPPWD_NO       (0 << (ADC_CR_DEEPPWD_SHIFT))
#define ADC_CR_DEEPPWD_YES      (1 << (ADC_CR_DEEPPWD_SHIFT))
#define ADC_CR_ADVREGEN_SHIFT   (28)
#define ADC_CR_ADVREGEN_MASK    (1 << (ADC_CR_ADVREGEN_SHIFT))
#define ADC_CR_ADVREGEN_NO      (0 << (ADC_CR_ADVREGEN_SHIFT))
#define ADC_CR_ADVREGEN_YES     (1 << (ADC_CR_ADVREGEN_SHIFT))
  /* boost is not supported */
#define ADC_CR_BOOST_SHIFT      (8)
#define ADC_CR_BOOST_MASK       (1 << (ADC_CR_BOOST_SHIFT))
#define ADC_CR_BOOST_NO         (0 << (ADC_CR_BOOST_SHIFT))
#define ADC_CR_BOOST_YES        (0 << (ADC_CR_BOOST_SHIFT))
#define ADC_CR_ADSTP_SHIFT      (4)
#define ADC_CR_ADSTP_MASK       (1 << (ADC_CR_ADSTP_SHIFT))
#define ADC_CR_ADSTP_NO         (0 << (ADC_CR_ADSTP_SHIFT))
#define ADC_CR_ADSTP_YES        (1 << (ADC_CR_ADSTP_SHIFT))
#define ADC_CR_ADSTART_SHIFT    (2)
#define ADC_CR_ADSTART_MASK     (1 << (ADC_CR_ADSTART_SHIFT))
#define ADC_CR_ADSTART_NO       (0 << (ADC_CR_ADSTART_SHIFT))
#define ADC_CR_ADSTART_YES      (1 << (ADC_CR_ADSTART_SHIFT))
#define ADC_CR_ADDIS_SHIFT      (1)
#define ADC_CR_ADDIS_MASK       (1 << (ADC_CR_ADDIS_SHIFT))
#define ADC_CR_ADDIS_NO         (0 << (ADC_CR_ADDIS_SHIFT))
#define ADC_CR_ADDIS_YES        (1 << (ADC_CR_ADDIS_SHIFT))
#define ADC_CR_ADEN_SHIFT       (0)
#define ADC_CR_ADEN_MASK        (1 << (ADC_CR_ADEN_SHIFT))
#define ADC_CR_ADEN_NO          (0 << (ADC_CR_ADEN_SHIFT))
#define ADC_CR_ADEN_YES         (1 << (ADC_CR_ADEN_SHIFT))
  __IO uint32_t         CFGR;           /* 0x0c */
#define ADC_CFGR_CONT_SHIFT     (13)
#define ADC_CFGR_CONT_MASK      (1 << (ADC_CFGR_CONT_SHIFT))
#define ADC_CFGR_CONT_NO        (0 << (ADC_CFGR_CONT_SHIFT))
#define ADC_CFGR_CONT_YES       (1 << (ADC_CFGR_CONT_SHIFT))
#define ADC_CFGR_EXTEN_SHIFT    (10)
#define ADC_CFGR_EXTEN_MASK     (3 << (ADC_CFGR_EXTEN_SHIFT))
#define ADC_CFGR_EXTEN_DIS      (0 << (ADC_CFGR_EXTEN_SHIFT))
#define ADC_CFGR_EXTEN_RISING   (1 << (ADC_CFGR_EXTEN_SHIFT))
#define ADC_CFGR_EXTEN_FALLING  (2 << (ADC_CFGR_EXTEN_SHIFT))
#define ADC_CFGR_EXTEN_BOTH     (3 << (ADC_CFGR_EXTEN_SHIFT))
#define ADC_CFGR_EXTSEL_SHIFT   (6)
#define ADC_CFGR_EXTSEL_MASK    (0xf << (ADC_CFGR_EXTSEL_SHIFT))
#define ADC_CFGR_EXTSEL_VAL(x)  ((x << (ADC_CFGR_EXTSEL_SHIFT)) & ADC_CFGR_EXTSEL_MASK)
#define ADC_CFGR_ALIGN_SHIFT    (5)
#define ADC_CFGR_ALIGN_MASK     (1 << (ADC_CFGR_ALIGN_SHIFT))
#define ADC_CFGR_ALIGN_NO       (0 << (ADC_CFGR_ALIGN_SHIFT))
#define ADC_CFGR_ALIGN_YES      (1 << (ADC_CFGR_ALIGN_SHIFT))
#define ADC_CFGR_RES_SHIFT      (3)
#define ADC_CFGR_RES_MASK       (3 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_16BIT      (0 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_14BIT      (0 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_12BIT      (0 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_10BIT      (1 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_8BIT       (2 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_6BIT       (3 << (ADC_CFGR_RES_SHIFT))
  /* DMNGT is not supported */
#define ADC_CFGR_DMNGT_SHIFT    (0)
#define ADC_CFGR_DMNGT_MASK     (7 << (ADC_CFGR_DMNGT_SHIFT))
#define ADC_CFGR_DMNGT_REGULAR  (0 << (ADC_CFGR_DMNGT_SHIFT))
#define ADC_CFGR_DMNGT_DMAONESHOT (0 << (ADC_CFGR_DMNGT_SHIFT))
#define ADC_CFGR_DMNGT_DFSDM    (0 << (ADC_CFGR_DMNGT_SHIFT))
#define ADC_CFGR_DMNGT_DMACIRC  (0 << (ADC_CFGR_DMNGT_SHIFT))

  __IO uint32_t         CFGR2;          /* 0x10 */

/* 0x14, 0x18 SMPR1, 2 */
  __IO uint32_t         SMPR1;          /* 0x14 */
  __IO uint32_t         SMPR2;          /* 0x18 */
#define ADC_SMPR_SMP_SHIFT      (0)
#define ADC_SMPR_SMP_MASK       (7 << (ADC_SMPR_SMP_SHIFT))
#define ADC_SMPR_SMP_1_5CLKS    (0 << (ADC_SMPR_SMP_SHIFT))
#define ADC_SMPR_SMP_2_5CLKS    (1 << (ADC_SMPR_SMP_SHIFT))
#define ADC_SMPR_SMP_8_5CLKS    (2 << (ADC_SMPR_SMP_SHIFT))
#define ADC_SMPR_SMP_16_5CLKS   (3 << (ADC_SMPR_SMP_SHIFT))
#define ADC_SMPR_SMP_32_5CLKS   (4 << (ADC_SMPR_SMP_SHIFT))
#define ADC_SMPR_SMP_64_5CLKS   (5 << (ADC_SMPR_SMP_SHIFT))
#define ADC_SMPR_SMP_387_5CLKS  (6 << (ADC_SMPR_SMP_SHIFT))
#define ADC_SMPR_SMP_810_5CLKS  (7 << (ADC_SMPR_SMP_SHIFT))

  __IO uint32_t         PCSEL;          /* 0x1c */
  __IO uint32_t         LTR1;           /* 0x20 */
  __IO uint32_t         HTR1;           /* 0x24 */
  uint32_t              reserved28[2];
#define ADC_SQR1_L_SHIFT        (0)
#define ADC_SQR1_L_MASK         (0xf << (ADC_SQR1_L_SHIFT))
#define ADC_SQR1_L_VAL(x)       (((x) << (ADC_SQR1_L_SHIFT) & ADC_SQR1_L_MASK)
#define ADC_SQR1_SQ1_SHIFT      (6)
#define ADC_SQR1_SQ1_MASK       (0xf << (ADC_SQR1_SQ1_SHIFT))
#define ADC_SQR1_SQ1_VAL(x)     (((x) << (ADC_SQR1_SQ1_SHIFT)) & ADC_SQR1_SQ1_MASK)
#define ADC_SQR1_SQ2_SHIFT      (12)
#define ADC_SQR1_SQ2_MASK       (0xf << (ADC_SQR1_SQ2_SHIFT))
#define ADC_SQR1_SQ2_VAL(x)     (((x) << (ADC_SQR1_SQ2_SHIFT)) & ADC_SQR1_SQ2_MASK)
  __IO uint32_t         SQR1;           /* 0x30 */
  __IO uint32_t         SQR2;           /* 0x34 */
  __IO uint32_t         SQR3;           /* 0x38 */
  __IO uint32_t         SQR4;           /* 0x3c */
  __IO uint32_t         DR;             /* 0x40 */
  uint32_t              reserved44[7];
  __IO uint32_t         OFR1;           /* 0x60 */
  __IO uint32_t         OFR2;           /* 0x64 */
  __IO uint32_t         OFR3;           /* 0x68 */
  __IO uint32_t         OFR4;           /* 0x6c */
  uint32_t              reserved70[12];
  uint32_t              reservedA0[24];
  uint32_t              reserved100[0x200/4];

  __IO uint32_t         CSR;            /* 0x300 */
  uint32_t              reserved304;

  __IO uint32_t         CCR;            /* 0x308 */
#define ADC_CCR_PRESC_SHIFT     (18)
#define ADC_CCR_PRESC_MASK      (0xf << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV1      (0 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV2      (1 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV4      (2 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV6      (3 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV8      (4 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV10     (5 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV12     (6 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV16     (7 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV32     (8 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV64     (9 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV128    (10 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_PRESC_DIV256    (11 << (ADC_CCR_PRESC_SHIFT))
#define ADC_CCR_CKMODE_SHIFT    (16)
#define ADC_CCR_CKMODE_MASK     (3 << (ADC_CCR_CKMODE_SHIFT))
#define ADC_CCR_CKMODE_ASYNC    (0 << (ADC_CCR_CKMODE_SHIFT))
#define ADC_CCR_CKMODE_HCLK_DIV1 (1 << (ADC_CCR_CKMODE_SHIFT))
#define ADC_CCR_CKMODE_HCLK_DIV2 (2 << (ADC_CCR_CKMODE_SHIFT))
#define ADC_CCR_CKMODE_HCLK_DIV4 (3 << (ADC_CCR_CKMODE_SHIFT))
  /* CCR DUAL is not supported */
#define ADC_CCR_DELAY_SHIFT     (8)
#define ADC_CCR_DELAY_MASK      (0xf << (ADC_CCR_DELAY_SHIFT))
#define ADC_CCR_DELAY_VAL(x)    (0)
#define ADC_CCR_DUAL_SHIFT      (0)
#define ADC_CCR_DUAL_MASK       (0x1f << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_INDEPENDENT (0 << (ADC_CCR_DUAL_SHIFT))
} stm32Dev_ADC;

#define ADC1_PTR        ((stm32Dev_ADC *) ((AHB2_BASE) + 0x08040000))


/*******************************************
 * 17 DAC
 */
struct _stStm32l4_DAC {
  __IO uint32_t	cr;				/* 0x00 */
#define CR_CEN2_SHIFT		30
#define CR_CEN2_MASK		(1 << (CR_CEN2_SHIFT))
#define CR_CEN2_NO		(0 << (CR_CEN2_SHIFT))
#define CR_CEN2_YES		(1 << (CR_CEN2_SHIFT))
#define CR_DMAUDRIE2_SHIFT	29
#define CR_DMAUDRIE2_MASK	(1 << (CR_DMAUDRIE2_SHIFT))
#define CR_DMAUDRIE2_NO		(0 << (CR_DMAUDRIE2_SHIFT))
#define CR_DMAUDRIE2_YES	(1 << (CR_DMAUDRIE2_SHIFT))
#define CR_DMAEN2_SHIFT		28
#define CR_DMAEN2_MASK		(1 << (CR_DMAEN2_SHIFT))
#define CR_DMAEN2_NO		(0 << (CR_DMAEN2_SHIFT))
#define CR_DMAEN2_YES		(1 << (CR_DMAEN2_SHIFT))
#define CR_MAMP2_SHIFT		24
#define CR_MAMP2_MASK		(1 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_1LSB	(0 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_2LSB	(1 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_3LSB	(2 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_4LSB	(3 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_5LSB	(4 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_6LSB	(5 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_7LSB	(6 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_8LSB	(7 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_9LSB	(8 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_10LSB	(9 << (CR_MAMP2_SHIFT))
#define CR_MAMP2_UNMASK_11LSB	(10 << (CR_MAMP2_SHIFT))
#define CR_WAVE2_SHIFT		22
#define CR_WAVE2_MASK		(1 << (CR_WAVE2_SHIFT))
#define CR_WAVE2_DISABLE	(0 << (CR_WAVE2_SHIFT))
#define CR_WAVE2_GEN_NOISE	(1 << (CR_WAVE2_SHIFT))
#define CR_WAVE2_GEN_TRIANGLE	(2 << (CR_WAVE2_SHIFT))
#define CR_TSEL2_SHIFT		21
#define CR_TSEL2_MASK		(1 << (CR_TSEL2_SHIFT))
#define CR_TSEL2_TIM6TRGO	(0 << (CR_TSEL2_SHIFT))
#define CR_TSEL2_RESERVED1	(1 << (CR_TSEL2_SHIFT))
#define CR_TSEL2_TIM7TRGO	(2 << (CR_TSEL2_SHIFT))
#define CR_TSEL2_RESERVED3	(3 << (CR_TSEL2_SHIFT))
#define CR_TSEL2_TIM2TRGO	(4 << (CR_TSEL2_SHIFT))
#define CR_TSEL2_RESERVED5	(5 << (CR_TSEL2_SHIFT))
#define CR_TSEL2_EXTERNAL_LINE9	(6 << (CR_TSEL2_SHIFT))
#define CR_TSEL2_SOFTWARE_TRG	(7 << (CR_TSEL2_SHIFT))
#define CR_TEN2_SHIFT		18
#define CR_TEN2_MASK		(1 << (CR_TEN2_SHIFT))
#define CR_TEN2_NO		(0 << (CR_TEN2_SHIFT))
#define CR_TEN2_YES		(1 << (CR_TEN2_SHIFT))
#define CR_EN2_SHIFT		16
#define CR_EN2_MASK		(1 << (CR_EN2_SHIFT))
#define CR_EN2_NO		(0 << (CR_EN2_SHIFT))
#define CR_EN2_YES		(1 << (CR_EN2_SHIFT))
#define CR_CEN1_SHIFT		14
#define CR_CEN1_MASK		(1 << (CR_CEN1_SHIFT))
#define CR_CEN1_NO		(0 << (CR_CEN1_SHIFT))
#define CR_CEN1_YES		(1 << (CR_CEN1_SHIFT))
#define CR_DMAUDRIE1_SHIFT	13
#define CR_DMAUDRIE1_MASK	(1 << (CR_DMAUDRIE1_SHIFT))
#define CR_DMAUDRIE1_NO		(0 << (CR_DMAUDRIE1_SHIFT))
#define CR_DMAUDRIE1_YES	(1 << (CR_DMAUDRIE1_SHIFT))
#define CR_DMAEN1_SHIFT		12
#define CR_DMAEN1_MASK		(1 << (CR_DMAEN1_SHIFT))
#define CR_DMAEN1_NO		(0 << (CR_DMAEN1_SHIFT))
#define CR_DMAEN1_YES		(1 << (CR_DMAEN1_SHIFT))
#define CR_MAMP1_SHIFT		8
#define CR_MAMP1_MASK		(1 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_1LSB	(0 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_2LSB	(1 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_3LSB	(2 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_4LSB	(3 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_5LSB	(4 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_6LSB	(5 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_7LSB	(6 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_8LSB	(7 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_9LSB	(8 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_10LSB	(9 << (CR_MAMP1_SHIFT))
#define CR_MAMP1_UNMASK_11LSB	(10 << (CR_MAMP1_SHIFT))
#define CR_WAVE1_SHIFT		6
#define CR_WAVE1_MASK		(1 << (CR_WAVE1_SHIFT))
#define CR_WAVE1_DISABLE	(0 << (CR_WAVE1_SHIFT))
#define CR_WAVE1_GEN_NOISE	(1 << (CR_WAVE1_SHIFT))
#define CR_WAVE1_GEN_TRIANGLE	(2 << (CR_WAVE1_SHIFT))
#define CR_TSEL1_SHIFT		3
#define CR_TSEL1_MASK		(1 << (CR_TSEL1_SHIFT))
#define CR_TSEL1_TIM6TRGO	(0 << (CR_TSEL1_SHIFT))
#define CR_TSEL1_RESERVED1	(1 << (CR_TSEL1_SHIFT))
#define CR_TSEL1_TIM7TRGO	(2 << (CR_TSEL1_SHIFT))
#define CR_TSEL1_RESERVED3	(3 << (CR_TSEL1_SHIFT))
#define CR_TSEL1_TIM2TRGO	(4 << (CR_TSEL1_SHIFT))
#define CR_TSEL1_RESERVED5	(5 << (CR_TSEL1_SHIFT))
#define CR_TSEL1_EXTERNAL_LINE9	(6 << (CR_TSEL1_SHIFT))
#define CR_TSEL1_SOFTWARE_TRG	(7 << (CR_TSEL1_SHIFT))
#define CR_TEN1_SHIFT		2
#define CR_TEN1_MASK		(1 << (CR_TEN1_SHIFT))
#define CR_TEN1_NO		(0 << (CR_TEN1_SHIFT))
#define CR_TEN1_YES		(1 << (CR_TEN1_SHIFT))
#define CR_EN1_SHIFT		0
#define CR_EN1_MASK		(1 << (CR_EN1_SHIFT))
#define CR_EN1_NO		(0 << (CR_EN1_SHIFT))
#define CR_EN1_YES		(1 << (CR_EN1_SHIFT))

  __IO uint32_t	swtrgr;			/* 0x04 */
  __IO uint32_t	dhr12r1;		/* 0x08 */
  __IO uint32_t	dhr12l1;		/* 0x0c */
  __IO uint32_t	dhr8r1;			/* 0x10 */
  __IO uint32_t	dhr12r2;		/* 0x14 */
  __IO uint32_t	dhr12l2;		/* 0x18 */
  __IO uint32_t	dhr8r2;			/* 0x1c */
  __IO uint32_t	dhr12rd;		/* 0x20 */
  __IO uint32_t	dhr12ld;		/* 0x24 */
  __IO uint32_t	dhr8rd;			/* 0x28 */
  __IO uint32_t	dor1;			/* 0x2c */
  __IO uint32_t	dor2;			/* 0x30 */
  __IO uint32_t	sr;			/* 0x34 */
  __IO uint32_t	ccr;			/* 0x38 */
  __IO uint32_t	mcr;			/* 0x3e */
#define MCR_MODE2_SHIFT			16
#define MCR_MODE2_MASK			(1 << (MCR_MODE2_SHIFT))
#define MCR_MODE2_NORMAL_EXTPIN_BUF	(0 << (MCR_MODE2_SHIFT))
#define MCR_MODE2_NORMAL_ONCHIP_BUF	(1 << (MCR_MODE2_SHIFT))
#define MCR_MODE2_NORMAL_EXTPIN		(2 << (MCR_MODE2_SHIFT))
#define MCR_MODE2_NORMAL_ONCHIP		(3 << (MCR_MODE2_SHIFT))
#define MCR_MODE2_SH_EXTPIN_BUF		(4 << (MCR_MODE2_SHIFT))
#define MCR_MODE2_SH_ONCHIP_BUF		(5 << (MCR_MODE2_SHIFT))
#define MCR_MODE2_SH_EXTPIN		(6 << (MCR_MODE2_SHIFT))
#define MCR_MODE2_SH_ONCHIP		(7 << (MCR_MODE2_SHIFT))
#define MCR_MODE1_SHIFT			0
#define MCR_MODE1_MASK			(1 << (MCR_MODE1_SHIFT))
#define MCR_MODE1_NORMAL_EXTPIN_BUF	(0 << (MCR_MODE1_SHIFT))
#define MCR_MODE1_NORMAL_ONCHIP_BUF	(1 << (MCR_MODE1_SHIFT))
#define MCR_MODE1_NORMAL_EXTPIN		(2 << (MCR_MODE1_SHIFT))
#define MCR_MODE1_NORMAL_ONCHIP		(3 << (MCR_MODE1_SHIFT))
#define MCR_MODE1_SH_EXTPIN_BUF		(4 << (MCR_MODE1_SHIFT))
#define MCR_MODE1_SH_ONCHIP_BUF		(5 << (MCR_MODE1_SHIFT))
#define MCR_MODE1_SH_EXTPIN		(6 << (MCR_MODE1_SHIFT))
#define MCR_MODE1_SH_ONCHIP		(7 << (MCR_MODE1_SHIFT))

  __IO uint32_t	shsr1;			/* 0x40 */
  __IO uint32_t	shsr2;			/* 0x44 */
  __IO uint32_t	shhr;			/* 0x48 */
  __IO uint32_t	shrr;			/* 0x4c */
};

#define DAC_PTR	((struct _stStm32l4_DAC *) (APB1_BASE + 0x7400))


/*******************************************
 * 35 I2C
 */
#define I2C_MODULE_COUNT                (4)

#include        "../../STM/include/stm32I2c.h"

#define I2C1_PTR        ((stm32Dev_I2C *) (APB1_BASE + 0x5400))
#define I2C2_PTR        ((stm32Dev_I2C *) (APB1_BASE + 0x5800))
#define I2C3_PTR        ((stm32Dev_I2C *) (APB1_BASE + 0x5c00))
#define I2C4_PTR        ((stm32Dev_I2C *) (APB1_BASE + 0x8400))


/*******************************************
 * 38 SPI
 */
#define SPI_MODULE_COUNT                (3)

#include        "stm32Spi16.h"

#define SPI1_PTR        ((stm32Dev_SPI *) (APB2_BASE + 0x3000))
#define SPI2_PTR        ((stm32Dev_SPI *) (APB1_BASE + 0x3800))
#define SPI3_PTR        ((stm32Dev_SPI *) (APB1_BASE + 0x3c00))


/*******************************************
 * 43 USB
 */
#define USB_MODULE_COUNT                (1)

/* EP0 - EP7 */
#define USB_MAX_EPIN            7
#define USB_MAX_EPOUT           7

#include        "stm32Usb16.h"

#define USBSRAM8_PTR    (__IO uint8_t *) (APB1_BASE + 0x6c00))
#define USBSRAM16_PTR   (__IO uint16_t *) (APB1_BASE + 0x6c00))
#define USBSRAM_PTR     ((stm32Dev_USB_SRAM_Header *) (APB1_BASE + 0x6c00))
#define USBSRAM_SIZE    (1024)

#define USB_FS_PTR         ((stm32Dev_USB *) (APB1_BASE + 0x6800))

#define UsbGetTxFifoPtr8(x)     ((__IO unit8_t  *)&USBSRAM8_PTR[USBSRAM_PTR->ep[x].txAddr])
#define UsbGetRxFifoPtr8(x)     ((__IO unit8_t  *)&USBSRAM8_PTR[USBSRAM_PTR->ep[x].rxAddr])
#define UsbGetTxFifoPtr16(x)    ((__IO unit16_t *)&USBSRAM8_PTR[USBSRAM_PTR->ep[x].txAddr])
#define UsbGetRxFifoPtr16(x)    ((__IO unit16_t *)&USBSRAM8_PTR[USBSRAM_PTR->ep[x].rxAddr])

#endif
