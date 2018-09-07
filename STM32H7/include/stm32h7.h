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

#ifndef _STM32H7_H_
#define _STM32H7_H_

#define __CM7_REV               0x0100  /* Cortex-M7 r1p0 */
#define __MPU_PRESENT           1
#define __FPU_PRESENT           1
#define __NVIC_PRIO_BITS        4
#define __Vendor_SysTickConfig  0

#ifndef __IO
#define __IO            volatile
#endif


/*************************************************************
 * interrupt request definitions
 */
#if 1
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

  TIM2_IRQn             =    28,
  TIM3_IRQn,
  TIM4_IRQn,

  SPI1_IRQn             =    35,
  SPI2_IRQn,
  USART1_IRQn,
  USART2_IRQn,
  USART3_IRQn,

  TIM5_IRQn             =    50,

  DMA1_Stream3_IRQn     =    85,

  OTG_FS_IRQn           =   101,
};

typedef int     IRQn_Type;
#endif


/*************************************************************
 * base address definitions
 */
#define AHB1_BASE               (0x40020000)
#define AHB2_BASE               (0x48000000)
#define AHB3_BASE               (0x52000000)
#define AHB4_BASE               (0x58020000)
#define APB1_BASE               (0x40000000)
#define APB2_BASE               (0x40010000)
#define APB3_BASE               (0x50000000)
#define APB4_BASE               (0x58000000)


/*************************************************************
 * clock definitions
 */
#define RCC_CLOCK_HSI           64000000UL
#define RCC_CLOCK_CSI            4000000UL
#define RCC_CLOCK_D1CPRETABLE   {0, 0, 0, 0,  1, 2, 3, 4,  1, 2, 3, 4,  6, 7, 8, 9}
#define RCC_CLOCK_D1HPRETABLE   {0, 0, 0, 0,  0, 0, 0, 0,  1, 2, 3, 4,  6, 7, 8, 9}
#define RCC_CLOCK_D1P3PRETABLE  {0, 0, 0, 0,  1, 2, 3, 4};
#define RCC_CLOCK_D2P1PRETABLE  {0, 0, 0, 0,  1, 2, 3, 4};
#define RCC_CLOCK_D2P2PRETABLE  {0, 0, 0, 0,  1, 2, 3, 4};
#define RCC_CLOCK_D3P4PRETABLE  {0, 0, 0, 0,  1, 2, 3, 4};

#define RCC_PTR         ((stm32Dev_RCC *) (AHB4_BASE + 0x4400))


/*************************************************************
 * bus definitions
 */
#define BUS_PERIPHERAL          0x40000000



/*************************************************************
 * GPIO definitions
 */
#include        "stm32Gpio.h"

#define GPIO_PTR        ((stm32Dev_GPIO *) ((AHB4_BASE)))


/*** DMA ***********************************************/
/* DMA LISR, HISR, LIFCR, HIFCR */
#define DMA_LISR_S3_SHIFT	(22)
#define DMA_LISR_ALL3_MASK	(0x7d << (DMA_LISR_S3_SHIFT))
#define DMA_LISR_FEIF3_MASK	(1 << ((DMA_LISR_S3_SHIFT)+0))
#define DMA_LISR_DMEIF3_MASK	(1 << ((DMA_LISR_S3_SHIFT)+2))
#define DMA_LISR_TEIF3_MASK	(1 << ((DMA_LISR_S3_SHIFT)+3))
#define DMA_LISR_HTIF3_MASK	(1 << ((DMA_LISR_S3_SHIFT)+4))
#define DMA_LISR_TCIF3_MASK	(1 << ((DMA_LISR_S3_SHIFT)+5))
#define DMA_LISR_S2_SHIFT	(16)
#define DMA_LISR_ALL2_MASK	(0x7d << (DMA_LISR_S2_SHIFT))
#define DMA_LISR_FEIF2_MASK	(1 << ((DMA_LISR_S2_SHIFT)+0))
#define DMA_LISR_DMEIF2_MASK	(1 << ((DMA_LISR_S2_SHIFT)+2))
#define DMA_LISR_TEIF2_MASK	(1 << ((DMA_LISR_S2_SHIFT)+3))
#define DMA_LISR_HTIF2_MASK	(1 << ((DMA_LISR_S2_SHIFT)+4))
#define DMA_LISR_TCIF2_MASK	(1 << ((DMA_LISR_S2_SHIFT)+5))
#define DMA_LISR_S1_SHIFT	(6)
#define DMA_LISR_ALL1_MASK	(0x7d << (DMA_LISR_S1_SHIFT))
#define DMA_LISR_FEIF1_MASK	(1 << ((DMA_LISR_S1_SHIFT)+0))
#define DMA_LISR_DMEIF1_MASK	(1 << ((DMA_LISR_S1_SHIFT)+2))
#define DMA_LISR_TEIF1_MASK	(1 << ((DMA_LISR_S1_SHIFT)+3))
#define DMA_LISR_HTIF1_MASK	(1 << ((DMA_LISR_S1_SHIFT)+4))
#define DMA_LISR_TCIF1_MASK	(1 << ((DMA_LISR_S1_SHIFT)+5))
#define DMA_LISR_S0_SHIFT	(0)
#define DMA_LISR_ALL0_MASK	(0x7d << (DMA_LISR_S0_SHIFT))
#define DMA_LISR_FEIF0_MASK	(1 << ((DMA_LISR_S0_SHIFT)+0))
#define DMA_LISR_DMEIF0_MASK	(1 << ((DMA_LISR_S0_SHIFT)+2))
#define DMA_LISR_TEIF0_MASK	(1 << ((DMA_LISR_S0_SHIFT)+3))
#define DMA_LISR_HTIF0_MASK	(1 << ((DMA_LISR_S0_SHIFT)+4))
#define DMA_LISR_TCIF0_MASK	(1 << ((DMA_LISR_S0_SHIFT)+5))

#define DMA_HISR_S7_SHIFT	(22)
#define DMA_HISR_ALL7_MASK	(0x7d << (DMA_HISR_S7_SHIFT))
#define DMA_HISR_FEIF7_MASK	(1 << ((DMA_HISR_S7_SHIFT)+0))
#define DMA_HISR_DMEIF7_MASK	(1 << ((DMA_HISR_S7_SHIFT)+2))
#define DMA_HISR_TEIF7_MASK	(1 << ((DMA_HISR_S7_SHIFT)+3))
#define DMA_HISR_HTIF7_MASK	(1 << ((DMA_HISR_S7_SHIFT)+4))
#define DMA_HISR_TCIF7_MASK	(1 << ((DMA_HISR_S7_SHIFT)+5))
#define DMA_HISR_S6_SHIFT	(16)
#define DMA_HISR_ALL6_MASK	(0x7d << (DMA_HISR_S6_SHIFT))
#define DMA_HISR_FEIF6_MASK	(1 << ((DMA_HISR_S6_SHIFT)+0))
#define DMA_HISR_DMEIF6_MASK	(1 << ((DMA_HISR_S6_SHIFT)+2))
#define DMA_HISR_TEIF6_MASK	(1 << ((DMA_HISR_S6_SHIFT)+3))
#define DMA_HISR_HTIF6_MASK	(1 << ((DMA_HISR_S6_SHIFT)+4))
#define DMA_HISR_TCIF6_MASK	(1 << ((DMA_HISR_S6_SHIFT)+5))
#define DMA_HISR_S5_SHIFT	(6)
#define DMA_HISR_ALL5_MASK	(0x7d << (DMA_HISR_S5_SHIFT))
#define DMA_HISR_FEIF5_MASK	(1 << ((DMA_HISR_S5_SHIFT)+0))
#define DMA_HISR_DMEIF5_MASK	(1 << ((DMA_HISR_S5_SHIFT)+2))
#define DMA_HISR_TEIF5_MASK	(1 << ((DMA_HISR_S5_SHIFT)+3))
#define DMA_HISR_HTIF5_MASK	(1 << ((DMA_HISR_S5_SHIFT)+4))
#define DMA_HISR_TCIF5_MASK	(1 << ((DMA_HISR_S5_SHIFT)+5))
#define DMA_HISR_S4_SHIFT	(0)
#define DMA_HISR_ALL4_MASK	(0x7d << (DMA_HISR_S4_SHIFT))
#define DMA_HISR_FEIF4_MASK	(1 << ((DMA_HISR_S4_SHIFT)+0))
#define DMA_HISR_DMEFIF4_MASK	(1 << ((DMA_HISR_S4_SHIFT)+2))
#define DMA_HISR_TEIF4_MASK	(1 << ((DMA_HISR_S4_SHIFT)+3))
#define DMA_HISR_HTIF4_MASK	(1 << ((DMA_HISR_S4_SHIFT)+4))
#define DMA_HISR_TCIF4_MASK	(1 << ((DMA_HISR_S4_SHIFT)+5))

/* DMA CR */
#define DMA_MBURST_SHIFT	(23)
#define DMA_MBURST_MASK		(3 << (DMA_MBURST_SHIFT))
#define DMA_MBURST_1		(0 << (DMA_MBURST_SHIFT))
#define DMA_MBURST_4		(1 << (DMA_MBURST_SHIFT))
#define DMA_MBURST_8		(2 << (DMA_MBURST_SHIFT))
#define DMA_MBURST_16		(3 << (DMA_MBURST_SHIFT))
#define DMA_PBURST_SHIFT	(21)
#define DMA_PBURST_MASK		(3 << (DMA_PBURST_SHIFT))
#define DMA_PBURST_1		(0 << (DMA_PBURST_SHIFT))
#define DMA_PBURST_4		(1 << (DMA_PBURST_SHIFT))
#define DMA_PBURST_8		(2 << (DMA_PBURST_SHIFT))
#define DMA_PBURST_16		(3 << (DMA_PBURST_SHIFT))
#define DMA_CT_SHIFT		(19)
#define DMA_CT_MASK		(1 << (DMA_CT_SHIFT))
#define DMA_CT_M0AR		(0 << (DMA_CT_SHIFT))
#define DMA_CT_M1AR		(1 << (DMA_CT_SHIFT))
#define DMA_DBM_SHIFT		(18)
#define DMA_DBM_MASK		(1 << (DMA_DBM_SHIFT))
#define DMA_DBM_NO		(0 << (DMA_DBM_SHIFT))
#define DMA_DBM_YES		(1 << (DMA_DBM_SHIFT))
#define DMA_PL_SHIFT		(16)
#define DMA_PL_MASK		(1 << (DMA_PL_SHIFT))
#define DMA_PL_LOW		(0 << (DMA_PL_SHIFT))
#define DMA_PL_MED		(1 << (DMA_PL_SHIFT))
#define DMA_PL_HIGH		(2 << (DMA_PL_SHIFT))
#define DMA_PL_VHIGH		(3 << (DMA_PL_SHIFT))
#define DMA_PINCOS_SHIFT	(15)
#define DMA_PINCOS_MASK		(1 << (DMA_PINCOS_SHIFT))
#define DMA_PINCOS_PSIZE	(0 << (DMA_PINCOS_SHIFT))
#define DMA_PINCOS_WORD		(1 << (DMA_PINCOS_SHIFT))
#define DMA_MSIZE_SHIFT		(13)
#define DMA_MSIZE_MASK		(3 << (DMA_MSIZE_SHIFT))
#define DMA_MSIZE_BYTE		(0 << (DMA_MSIZE_SHIFT))
#define DMA_MSIZE_HWORD		(1 << (DMA_MSIZE_SHIFT))
#define DMA_MSIZE_WORD		(2 << (DMA_MSIZE_SHIFT))
#define DMA_MSIZE_RESERVED	(3 << (DMA_MSIZE_SHIFT))
#define DMA_PSIZE_SHIFT		(11)
#define DMA_PSIZE_MASK		(3 << (DMA_PSIZE_SHIFT))
#define DMA_PSIZE_BYTE		(0 << (DMA_PSIZE_SHIFT))
#define DMA_PSIZE_HWORD		(1 << (DMA_PSIZE_SHIFT))
#define DMA_PSIZE_WORD		(2 << (DMA_PSIZE_SHIFT))
#define DMA_PSIZE_RESERVED	(3 << (DMA_PSIZE_SHIFT))
#define DMA_MINC_SHIFT		(10)
#define DMA_MINC_MASK		(1 << (DMA_MINC_SHIFT))
#define DMA_MINC_FIXED		(0 << (DMA_MINC_SHIFT))
#define DMA_MINC_INC		(1 << (DMA_MINC_SHIFT))
#define DMA_PINC_SHIFT		(9)
#define DMA_PINC_MASK		(1 << (DMA_PINC_SHIFT))
#define DMA_PINC_FIXED		(0 << (DMA_PINC_SHIFT))
#define DMA_PINC_INC		(1 << (DMA_PINC_SHIFT))
#define DMA_CIRC_SHIFT		(8)
#define DMA_CIRC_MASK		(1 << (DMA_CIRC_SHIFT))
#define DMA_CIRC_NO		(0 << (DMA_CIRC_SHIFT))
#define DMA_CIRC_YES		(1 << (DMA_CIRC_SHIFT))
#define DMA_DIR_SHIFT		(6)
#define DMA_DIR_MASK		(1 << (DMA_DIR_SHIFT))
#define DMA_DIR_PTOM		(0 << (DMA_DIR_SHIFT))
#define DMA_DIR_MTOP		(1 << (DMA_DIR_SHIFT))
#define DMA_DIR_MTOM		(2 << (DMA_DIR_SHIFT))
#define DMA_DIR_RESERVED	(3 << (DMA_DIR_SHIFT))
#define DMA_PFCTRL_SHIFT	(5)
#define DMA_PFCTRL_MASK		(1 << (DMA_PFCTRL_SHIFT))
#define DMA_PFCTRL_DMA		(0 << (DMA_PFCTRL_SHIFT))
#define DMA_PFCTRL_PERI		(1 << (DMA_PFCTRL_SHIFT))
#define DMA_TCIE_SHIFT		(4)
#define DMA_TCIE_MASK		(1 << (DMA_TCIE_SHIFT))
#define DMA_TCIE_DIS		(0 << (DMA_TCIE_SHIFT))
#define DMA_TCIE_EN		(1 << (DMA_TCIE_SHIFT))
#define DMA_HTIE_SHIFT		(3)
#define DMA_HTIE_MASK		(1 << (DMA_HTIE_SHIFT))
#define DMA_HTIE_DIS		(0 << (DMA_HTIE_SHIFT))
#define DMA_HTIE_EN		(1 << (DMA_HTIE_SHIFT))
#define DMA_TEIE_SHIFT		(2)
#define DMA_TEIE_MASK		(1 << (DMA_TEIE_SHIFT))
#define DMA_TEIE_DIS		(0 << (DMA_TEIE_SHIFT))
#define DMA_TEIE_EN		(1 << (DMA_TEIE_SHIFT))
#define DMA_DMEIE_SHIFT		(1)
#define DMA_DMEIE_MASK		(1 << (DMA_DMEIE_SHIFT))
#define DMA_DMEIE_DIS		(0 << (DMA_DMEIE_SHIFT))
#define DMA_DMEIE_EN		(1 << (DMA_DMEIE_SHIFT))
#define DMA_EN_SHIFT		(0)
#define DMA_EN_MASK		(1 << (DMA_EN_SHIFT))
#define DMA_EN_NO		(0 << (DMA_EN_SHIFT))
#define DMA_EN_YES		(1 << (DMA_EN_SHIFT))


typedef struct {
  __IO uint32_t         CR;
  __IO uint32_t         NDTR;
  __IO uint32_t         PAR;
  __IO uint32_t         M0AR;
  __IO uint32_t         M1AR;
  __IO uint32_t         FCR;
} stm32Dev_DMA_Stream;
typedef struct {
  __IO uint32_t         LISR;
  __IO uint32_t         HISR;
  __IO uint32_t         LIFCR;
  __IO uint32_t         HIFCR;
  stm32Dev_DMA_Stream   S[8];
} stm32Dev_DMA;

#define DMA1_PTR        ((stm32Dev_DMA *) (AHB1_BASE + 0x0000))
#define DMA2_PTR        ((stm32Dev_DMA *) (AHB1_BASE + 0x0400))


enum stm32Dev_DmaReq {
  DMA_REQUEST_ADC1 = 9,
  DMA_REQUEST_ADC2,

  DMA_REQUEST_DAC1 = 67,
  DMA_REQUEST_DAC2,
};


/*******************************************
 * 06 PWR
 */
typedef struct {
  __IO uint32_t         CR1;
  __IO uint32_t         CSR2;
  __IO uint32_t         CR2;
  __IO uint32_t         CR3;
#define PWR_CR3_USB33DEN_SHIFT  (14)
#define PWR_CR3_USB33DEN_MASK   (1 << (PWR_CR3_USB33DEN_SHIFT))
#define PWR_CR3_USB33DEN_NO     (0 << (PWR_CR3_USB33DEN_SHIFT))
#define PWR_CR3_USB33DEN_YES    (1 << (PWR_CR3_USB33DEN_SHIFT))
#define PWR_CR3_SCUEN_SHIFT     (2)
#define PWR_CR3_SCUEN_MASK      (1 << (PWR_CR3_SCUEN_SHIFT))
#define PWR_CR3_SCUEN_NO        (0 << (PWR_CR3_SCUEN_SHIFT))
#define PWR_CR3_SCUEN_YES       (1 << (PWR_CR3_SCUEN_SHIFT))
#define PWR_CR3_LDOEN_SHIFT     (1)
#define PWR_CR3_LDOEN_MASK      (1 << (PWR_CR3_LDOEN_SHIFT))
#define PWR_CR3_LDOEN_NO        (0 << (PWR_CR3_LDOEN_SHIFT))
#define PWR_CR3_LDOEN_YES       (1 << (PWR_CR3_LDOEN_SHIFT))
#define PWR_CR3_BYPASS_SHIFT    (0)
#define PWR_CR3_BYPASS_MASK     (1 << (PWR_CR3_BYPASS_SHIFT))
#define PWR_CR3_BYPASS_NO       (0 << (PWR_CR3_BYPASS_SHIFT))
#define PWR_CR3_BYPASS_YES      (1 << (PWR_CR3_BYPASS_SHIFT))
  __IO uint32_t         CPUCR;
  uint32_t              reserved14;
  __IO uint32_t         D3CR;
#define PWR_D3CR_VOS_SHIFT      (14)
#define PWR_D3CR_VOS_MASK       (3 << (PWR_D3CR_VOS_SHIFT))
#define PWR_D3CR_VOS_RESERVED   (0 << (PWR_D3CR_VOS_SHIFT))
#define PWR_D3CR_VOS_3          (1 << (PWR_D3CR_VOS_SHIFT))
#define PWR_D3CR_VOS_2          (2 << (PWR_D3CR_VOS_SHIFT))
#define PWR_D3CR_VOS_1          (3 << (PWR_D3CR_VOS_SHIFT))
#define PWR_D3CR_VOSRDY_SHIFT   (13)
#define PWR_D3CR_VOSRDY_MASK    (1 << (PWR_D3CR_VOSRDY_SHIFT))

  uint32_t              reserved1c;
  __IO uint32_t         PWR_WKUPCR;
  __IO uint32_t         PWR_WKUPFR;
  __IO uint32_t         PWR_WKUPEPR;
} stm32Dev_PWR;

#define PWR_PTR ((stm32Dev_PWR *) (AHB4_BASE + 0x4800))

/*******************************************
 * 08 RCC
 */
#include        "stm32h7Rcc.h"


/*******************************************
 * 09 CRS
 */

typedef struct {
/*** 0x00  CRS CR */
  __IO uint32_t         CR;
#define CRS_CR_TRIM_SHIFT       (8)
#define CRS_CR_TRIM_MASK        (0x3f << (CRS_CR_TRIM_SHIFT))
#define CRS_CR_TRIM_VAL(x)      ((x) << (CRS_CR_TRIM_SHIFT))

#define CRS_CR_AUTOTRIMEN_SHIFT (6)
#define CRS_CR_AUTOTRIMEN_MASK  (1 << (CRS_CR_AUTOTRIMEN_SHIFT))
#define CRS_CR_AUTOTRIMEN_NO    (0 << (CRS_CR_AUTOTRIMEN_SHIFT))
#define CRS_CR_AUTOTRIMEN_YES   (1 << (CRS_CR_AUTOTRIMEN_SHIFT))
#define CRS_CR_CEN_SHIFT        (5)
#define CRS_CR_CEN_MASK         (1 << (CRS_CR_CEN_SHIFT))
#define CRS_CR_CEN_NO           (0 << (CRS_CR_CEN_SHIFT))
#define CRS_CR_CEN_YES          (1 << (CRS_CR_CEN_SHIFT))

/*** 0x04  CRS CFGR */
  __IO uint32_t         CFGR;
#define CRS_CFGR_SYNCPOL_SHIFT      (31)
#define CRS_CFGR_SYNCPOL_MASK       (1 << (CRS_CFGR_SYNCPOL_SHIFT))
#define CRS_CFGR_SYNCPOL_RISING     (0 << (CRS_CFGR_SYNCPOL_SHIFT))
#define CRS_CFGR_SYNCPOL_FALLING    (1 << (CRS_CFGR_SYNCPOL_SHIFT))
#define CRS_CFGR_SYNCSRC_SHIFT      (28)
#define CRS_CFGR_SYNCSRC_MASK       (3 << (CRS_CFGR_SYNCSRC_SHIFT))
#define CRS_CFGR_SYNCSRC_USB2SOF    (0 << (CRS_CFGR_SYNCSRC_SHIFT))
#define CRS_CFGR_SYNCSRC_LSE        (1 << (CRS_CFGR_SYNCSRC_SHIFT))
#define CRS_CFGR_SYNCSRC_USB1SOF    (2 << (CRS_CFGR_SYNCSRC_SHIFT))
#define CRS_CFGR_SYNCSRC_RESERVED   (3 << (CRS_CFGR_SYNCSRC_SHIFT))
#define CRS_CFGR_SYNCDIV_SHIFT      (24)
#define CRS_CFGR_SYNCDIV_MASK       (7 << (CRS_CFGR_SYNCDIV_SHIFT))
#define CRS_CFGR_SYNCDIV_VAL(x)     ((x) << (CRS_CFGR_SYNCDIV_SHIFT))
#define CRS_CFGR_SYNCDIV_DIV1       (0 << (CRS_CFGR_SYNCDIV_SHIFT))
#define CRS_CFGR_SYNCDIV_DIV2       (1 << (CRS_CFGR_SYNCDIV_SHIFT))
#define CRS_CFGR_SYNCDIV_DIV4       (2 << (CRS_CFGR_SYNCDIV_SHIFT))
#define CRS_CFGR_SYNCDIV_DIV8       (3 << (CRS_CFGR_SYNCDIV_SHIFT))
#define CRS_CFGR_SYNCDIV_DIV16      (4 << (CRS_CFGR_SYNCDIV_SHIFT))
#define CRS_CFGR_SYNCDIV_DIV32      (5 << (CRS_CFGR_SYNCDIV_SHIFT))
#define CRS_CFGR_SYNCDIV_DIV64      (6 << (CRS_CFGR_SYNCDIV_SHIFT))
#define CRS_CFGR_SYNCDIV_DIV128     (7 << (CRS_CFGR_SYNCDIV_SHIFT))
#define CRS_CFGR_FELIM_SHIFT        (16)
#define CRS_CFGR_FELIM_MASK         (0xff << (CRS_CFGR_FELIM_SHIFT))
#define CRS_CFGR_FELIM_VAL(x)       ((x) << (CRS_CFGR_FELIM_SHIFT))
#define CRS_CFGR_RELOAD_SHIFT       (0)
#define CRS_CFGR_RELOAD_MASK        (0xffff << (CRS_CFGR_RELOAD_SHIFT))
#define CRS_CFGR_RELOAD_VAL(x)      (((x)-1) << (CRS_CFGR_RELOAD_SHIFT))
#define CRS_CFGR_RELOAD_48MHZ       ((48000000/1000-1) << (CRS_CFGR_RELOAD_SHIFT))

  __IO uint32_t         ISR;
  __IO uint32_t         ICR;
} stm32Dev_CRS;

#define CRS_PTR        ((stm32Dev_CRS *) (APB1_BASE + 0x8400))


/*******************************************
 * 15 DMA
 */
#define DMA_CR_DIR_SHIFT        (6)
#define DMA_CR_DIR_MASK         (3 << (DMA_CR_DIR_SHIFT))
#define DMA_CR_DIR_PtoM         (0 << (DMA_CR_DIR_SHIFT))
#define DMA_CR_DIR_MtoP         (1 << (DMA_CR_DIR_SHIFT))
#define DMA_CR_DIR_MtoM         (2 << (DMA_CR_DIR_SHIFT))
#define DMA_CR_DIR_RESERVED     (3 << (DMA_CR_DIR_SHIFT))

#define DMA_CR_MSIZE_SHIFT      (13)
#define DMA_CR_MSIZE_MASK       (3 << (DMA_CR_MSIZE_SHIFT))
#define DMA_CR_MSIZE_8BITS      (0 << (DMA_CR_MSIZE_SHIFT))
#define DMA_CR_MSIZE_16BITS     (1 << (DMA_CR_MSIZE_SHIFT))
#define DMA_CR_MSIZE_32BITS     (2 << (DMA_CR_MSIZE_SHIFT))
#define DMA_CR_MSIZE_RESERVED   (3 << (DMA_CR_MSIZE_SHIFT))

#define DMA_CR_PSIZE_SHIFT      (11)
#define DMA_CR_PSIZE_MASK       (3 << (DMA_CR_PSIZE_SHIFT))
#define DMA_CR_PSIZE_8BITS      (0 << (DMA_CR_PSIZE_SHIFT))
#define DMA_CR_PSIZE_16BITS     (1 << (DMA_CR_PSIZE_SHIFT))
#define DMA_CR_PSIZE_32BITS     (2 << (DMA_CR_PSIZE_SHIFT))
#define DMA_CR_PSIZE_RESERVED   (3 << (DMA_CR_PSIZE_SHIFT))


/*******************************************
 * 17 DMAMUX
 */
#define DMAMUX_REQ_SPI1RX       37
#define DMAMUX_REQ_SPI1TX       38
#define DMAMUX_REQ_SPI2RX       39
#define DMAMUX_REQ_SPI2TX       40
#define DMAMUX_REQ_SPI3RX       61
#define DMAMUX_REQ_SPI3TX       62
#define DMAMUX_REQ_SPI4RX       83
#define DMAMUX_REQ_SPI4TX       84
#define DMAMUX_REQ_SPI5RX       85
#define DMAMUX_REQ_SPI5TX       86
#define DMAMUX_REQ_SPI6RX       11
#define DMAMUX_REQ_SPI6TX       12
#define DMAMUX_MODULE_SPIRX_TBL {0, 0, 0, 0, 0, 0, 0x20}        /* [7:4]: module number, [3:0]: reseved */
#define DMAMUX_MODULE_SPITX_TBL {0, 0, 0, 0, 0, 0, 0x20}        /* [7:4]: module number, [3:0]: reseved */
#define DMAMUX_REQ_SPIRX_TBL   {0, DMAMUX_REQ_SPI1RX, DMAMUX_REQ_SPI2RX, DMAMUX_REQ_SPI3RX, DMAMUX_REQ_SPI4RX, DMAMUX_REQ_SPI5RX, DMAMUX_REQ_SPI6RX}
#define DMAMUX_REQ_SPITX_TBL   {0, DMAMUX_REQ_SPI1TX, DMAMUX_REQ_SPI2TX, DMAMUX_REQ_SPI3TX, DMAMUX_REQ_SPI4TX, DMAMUX_REQ_SPI5TX, DMAMUX_REQ_SPI6TX}


/*******************************************
 * 22 FMC
 */
#define FMC_BCR1_MWID_SHIFT     (4)
#define FMC_BCR1_MWID_MASK      (3 << (FMC_BCR1_MWID_SHIFT))
#define FMC_BCR1_MWID_8BIT      (0 << (FMC_BCR1_MWID_SHIFT))
#define FMC_BCR1_MWID_16BIT     (1 << (FMC_BCR1_MWID_SHIFT))
#define FMC_BCR1_MWID_32BIT     (2 << (FMC_BCR1_MWID_SHIFT))
#define FMC_BCR1_MWID_RESERVED  (3 << (FMC_BCR1_MWID_SHIFT))
#define FMC_BCR1_MTYP_SHIFT     (2)
#define FMC_BCR1_MTYP_MASK      (3 << (FMC_BCR1_MTYP_SHIFT))
#define FMC_BCR1_MTYP_SRAM      (0 << (FMC_BCR1_MTYP_SHIFT))
#define FMC_BCR1_MTYP_PSRAM     (1 << (FMC_BCR1_MTYP_SHIFT))
#define FMC_BCR1_MTYP_NORFLASH  (2 << (FMC_BCR1_MTYP_SHIFT))
#define FMC_BCR1_MTYP_RESERVED  (3 << (FMC_BCR1_MTYP_SHIFT))

#define FMC_BCR2_DATAST_SHIFT   (8)
#define FMC_BCR2_DATAST_MASK    (0xff << (FMC_BCR2_DATAST_SHIFT))
#define FMC_BCR2_DATAST(x)      ((x)  << (FMC_BCR2_DATAST_SHIFT))


/*******************************************
 * 25 ADC
 */
#define ADC_CFGR_EXTEN_SHIFT    (10)
#define ADC_CFGR_EXTEN_MASK     (3 << (ADC_CFGR_EXTEN_SHIFT))
#define ADC_CFGR_EXTEN_DIS      (0 << (ADC_CFGR_EXTEN_SHIFT))
#define ADC_CFGR_EXTEN_RISING   (1 << (ADC_CFGR_EXTEN_SHIFT))
#define ADC_CFGR_EXTEN_FALLING  (2 << (ADC_CFGR_EXTEN_SHIFT))
#define ADC_CFGR_EXTEN_BOTH     (3 << (ADC_CFGR_EXTEN_SHIFT))
#define ADC_CFGR_EXTSEL_SHIFT   (5)
#define ADC_CFGR_EXTSEL_MASK    (0x1f << (ADC_CFGR_EXTSEL_SHIFT))
#define ADC_CFGR_EXTSEL_TIM1CC1 (0 << (ADC_CFGR_EXTSEL_SHIFT))
#define ADC_CFGR_EXTSEL_TIM1CC2 (1 << (ADC_CFGR_EXTSEL_SHIFT))
#define ADC_CFGR_EXTSEL_TIM1CC3 (2 << (ADC_CFGR_EXTSEL_SHIFT))
#define ADC_CFGR_EXTSEL_TIM2CC2 (3 << (ADC_CFGR_EXTSEL_SHIFT))
#define ADC_CFGR_EXTSEL_TIM3TRGO (4 << (ADC_CFGR_EXTSEL_SHIFT))
#define ADC_CFGR_EXTSEL_TIM4CC4 (5 << (ADC_CFGR_EXTSEL_SHIFT))
#define ADC_CFGR_RES_SHIFT      (2)
#define ADC_CFGR_RES_MASK       (7 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_16BIT      (0 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_14BIT      (1 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_12BIT      (2 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_10BIT      (3 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_RES_8BIT       (4 << (ADC_CFGR_RES_SHIFT))
#define ADC_CFGR_DMNGT_SHIFT    (0)
#define ADC_CFGR_DMNGT_MASK     (7 << (ADC_CFGR_DMNGT_SHIFT))
#define ADC_CFGR_DMNGT_REGULAR  (0 << (ADC_CFGR_DMNGT_SHIFT))
#define ADC_CFGR_DMNGT_DMAONESHOT (1 << (ADC_CFGR_DMNGT_SHIFT))
#define ADC_CFGR_DMNGT_DFSDM    (2 << (ADC_CFGR_DMNGT_SHIFT))
#define ADC_CFGR_DMNGT_DMACIRC  (3 << (ADC_CFGR_DMNGT_SHIFT))
/* 0x14, 0x18 SMPR1, 2 */
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


/*** 0x308 CCR */
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
#define ADC_CCR_DAMDF_SHIFT     (14)
#define ADC_CCR_DAMDF_MASK      (3 << (ADC_CCR_DAMDF_SHIFT))
#define ADC_CCR_DAMDF_DUALDAC   (0 << (ADC_CCR_DAMDF_SHIFT))
#define ADC_CCR_DAMDF_RESERVED  (1 << (ADC_CCR_DAMDF_SHIFT))
#define ADC_CCR_DAMDF_10BIT     (2 << (ADC_CCR_DAMDF_SHIFT))
#define ADC_CCR_DAMDF_8BIT      (3 << (ADC_CCR_DAMDF_SHIFT))
#define ADC_CCR_DELAY_SHIFT     (8)
#define ADC_CCR_DELAY_MASK      (0xf << (ADC_CCR_DELAY_SHIFT))
#define ADC_CCR_DELAY_VAL(x)    ((x) << (ADC_CCR_DELAY_SHIFT))
#define ADC_CCR_DUAL_SHIFT      (0)
#define ADC_CCR_DUAL_MASK       (0x1f << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_INDEPENDENT (0 << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_COMBINE_INJECTED  (1 << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_COMBINE_ALTERNATE (2 << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_COMBINE_INTER_ALT (3 << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_RESERVED4         (4 << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_INJECTED          (5 << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_REGULAR           (6 << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_INTEREAVED        (7 << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_RESERVED8         (8 << (ADC_CCR_DUAL_SHIFT))
#define ADC_CCR_DUAL_ALTERNATE_TRIGGER (9 << (ADC_CCR_DUAL_SHIFT))



/*******************************************
 * 39 TIM2,3,4,5
 * 40 TIM
 * 41 TIM
 * 42 TIM
 */

#include        "../../STM/include/stm32Tim.h"

#define TIM2_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x0000))
#define TIM3_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x0400))
#define TIM4_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x0800))
#define TIM5_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x0c00))
#define TIM6_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x1000))
#define TIM7_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x1400))
#define TIM12_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x1800))
#define TIM13_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x1c00))
#define TIM14_PTR	((stm32Dev_TIM *) (APB1_BASE + 0x2000))

#define TIM1_PTR	((stm32Dev_TIM *) (APB2_BASE + 0x0000))
#define TIM8_PTR	((stm32Dev_TIM *) (APB2_BASE + 0x0400))
#define TIM15_PTR	((stm32Dev_TIM *) (APB2_BASE + 0x4000))
#define TIM16_PTR	((stm32Dev_TIM *) (APB2_BASE + 0x4400))
#define TIM17_PTR	((stm32Dev_TIM *) (APB2_BASE + 0x4800))


/*******************************************
 * 48 USART
 */
#if 0
/*** 0x004  USART CR2 */
#define USART_CR2_STOP_SHIFT    (12)
#define USART_CR2_STOP_MASK     (3 << (USART_CR2_STOP_SHIFT))
#define USART_CR2_STOP_1BIT     (0 << (USART_CR2_STOP_SHIFT))
#define USART_CR2_STOP_0_5BIT   (1 << (USART_CR2_STOP_SHIFT))
#define USART_CR2_STOP_2BIT     (2 << (USART_CR2_STOP_SHIFT))
#define USART_CR2_STOP_1_5BIT   (3 << (USART_CR2_STOP_SHIFT))

/*** 0x008  USART CR3 */
#define USART_CR3_RXFTCFG_SHIFT       (25)
#define USART_CR3_RXFTCFG_MASK        (7 << (USART_CR3_RXFTCFG_SHIFT))
#define USART_CR3_RXFTCFG_1_8         (0 << (USART_CR3_RXFTCFG_SHIFT))
#define USART_CR3_RXFTCFG_1_4         (1 << (USART_CR3_RXFTCFG_SHIFT))
#define USART_CR3_RXFTCFG_1_2         (2 << (USART_CR3_RXFTCFG_SHIFT))
#define USART_CR3_RXFTCFG_3_4         (3 << (USART_CR3_RXFTCFG_SHIFT))
#define USART_CR3_RXFTCFG_7_8         (4 << (USART_CR3_RXFTCFG_SHIFT))
#define USART_CR3_RXFTCFG_FULL        (5 << (USART_CR3_RXFTCFG_SHIFT))


#define USART_RX_FIFO_SIZE         (16)
#define USART_TX_FIFO_SIZE         (16)
#endif

#include        "../../STM/include/stm32Usart.h"

#define USART1_PTR      ((stm32DEV_USART *) ((APB2_BASE) + 0x1000))
#define USART6_PTR      ((stm32DEV_USART *) ((APB2_BASE) + 0x1400))

#define USART_PTR       ((stm32Dev_USART *) ((APB1_BASE) + 0x4000))
#define USART2_PTR      (&USART_PTR[1])
#define USART3_PTR      (&USART_PTR[2])
#define USART4_PTR      (&USART_PTR[3])        /* actual name is UART4 */
#define UART5_PTR       (&USART_PTR[4])
#define UART6_PTR       ((stm32DEV_USART *) ((APB1_BASE) + 0x7800))
#define UART7_PTR       ((stm32DEV_USART *) ((APB1_BASE) + 0x7c00))
#define LPUART1_PTR     ((stm32Dev_USART *) ((APB4_BASE) + 0x0c00))


/*******************************************
 * 50 SPI
 */
#define SPI_MODULE_COUNT                (6)


#define SPI_SR_RXPLVL_SHIFT             (13)
#define SPI_SR_RXPLVL_MASK              (3 << (SPI_SR_RXPLVL_SHIFT))



  typedef struct {
  __IO uint32_t         CR1;            /* 0x00 */
#define SPI_CR1_HDDIR_SHIFT             (11)
#define SPI_CR1_HDDIR_MASK              (1 << (SPI_CR1_HDDIR_SHIFT))
#define SPI_CR1_HDDIR_NO                (0 << (SPI_CR1_HDDIR_SHIFT))
#define SPI_CR1_HDDIR_YES               (1 << (SPI_CR1_HDDIR_SHIFT))
#define SPI_CR1_CSUSP_SHIFT             (10)
#define SPI_CR1_CSUSP_MASK              (1 << (SPI_CR1_CSUSP_SHIFT))
#define SPI_CR1_CSUSP_NO                (0 << (SPI_CR1_CSUSP_SHIFT))
#define SPI_CR1_CSUSP_YES               (1 << (SPI_CR1_CSUSP_SHIFT))
#define SPI_CR1_CSTART_SHIFT            (9)
#define SPI_CR1_CSTART_MASK             (1 << (SPI_CR1_CSTART_SHIFT))
#define SPI_CR1_CSTART_NO               (0 << (SPI_CR1_CSTART_SHIFT))
#define SPI_CR1_CSTART_YES              (1 << (SPI_CR1_CSTART_SHIFT))
#define SPI_CR1_MASRX_SHIFT             (8)
#define SPI_CR1_MASRX_MASK              (1 << (SPI_CR1_MASRX_SHIFT))
#define SPI_CR1_MASRX_NO                (0 << (SPI_CR1_MASRX_SHIFT))
#define SPI_CR1_MASRX_YES               (1 << (SPI_CR1_MASRX_SHIFT))
#define SPI_CR1_SPE_SHIFT               (0)
#define SPI_CR1_SPE_MASK                (1 << (SPI_CR1_SPE_SHIFT))
#define SPI_CR1_SPE_NO                  (0 << (SPI_CR1_SPE_SHIFT))
#define SPI_CR1_SPE_YES                 (1 << (SPI_CR1_SPE_SHIFT))

  __IO uint32_t         CR2;            /* 0x04 */
#define SPI_CR2_TSER_SHIFT              (16)
#define SPI_CR2_TSER_MASK               (0xffff << (SPI_CR2_TSER_SHIFT))
#define SPI_CR2_TSIZE_SHIFT             (0)
#define SPI_CR2_TSIZE_MASK              (0xffff << (SPI_CR2_TSIZE_SHIFT))

  __IO uint32_t         CFG1;           /* 0x08 */
#define SPI_CFG1_MBR_SHIFT              (28)
#define SPI_CFG1_MBR_MASK               (7 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV2               (0 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV4               (1 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV8               (2 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV16              (3 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV32              (4 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV64              (5 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV128             (6 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV256             (7 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_TXDMAEN_SHIFT          (15)
#define SPI_CFG1_TXDMAEN_MASK           (1 << (SPI_CFG1_TXDMAEN_SHIFT))
#define SPI_CFG1_TXDMAEN_NO             (0 << (SPI_CFG1_TXDMAEN_SHIFT))
#define SPI_CFG1_TXDMAEN_YES            (1 << (SPI_CFG1_TXDMAEN_SHIFT))
#define SPI_CFG1_RXDMAEN_SHIFT          (14)
#define SPI_CFG1_RXDMAEN_MASK           (1 << (SPI_CFG1_RXDMAEN_SHIFT))
#define SPI_CFG1_RXDMAEN_NO             (0 << (SPI_CFG1_RXDMAEN_SHIFT))
#define SPI_CFG1_RXDMAEN_YES            (1 << (SPI_CFG1_RXDMAEN_SHIFT))
#define SPI_CFG1_FTHLV_SHIFT            (5)
#define SPI_CFG1_FTHLV_MASK             (15 << (SPI_CFG1_FTHLV_SHIFT))
#define SPI_CFG1_FTHLV_VAL(x)           (((x)-1)  << (SPI_CFG1_FTHLV_SHIFT))
#define SPI_CFG1_FTHLV_1BIT             (0  << (SPI_CFG1_FTHLV_SHIFT))
#define SPI_CFG1_FTHLV_8BIT             (7  << (SPI_CFG1_FTHLV_SHIFT))
#define SPI_CFG1_FTHLV_16BIT            (15  << (SPI_CFG1_FTHLV_SHIFT))
#define SPI_CFG1_DSIZE_SHIFT            (0)
#define SPI_CFG1_DSIZE_MASK             (31 << (SPI_CFG1_DSIZE_SHIFT))
#define SPI_CFG1_DSIZE_VAL(x)           (((x)-1)  << (SPI_CFG1_DSIZE_SHIFT))
#define SPI_CFG1_DSIZE_1BIT             (0  << (SPI_CFG1_DSIZE_SHIFT))
#define SPI_CFG1_DSIZE_8BIT             (7  << (SPI_CFG1_DSIZE_SHIFT))
#define SPI_CFG1_DSIZE_16BIT            (15 << (SPI_CFG1_DSIZE_SHIFT))
#define SPI_CFG1_DSIZE_32BIT            (31 << (SPI_CFG1_DSIZE_SHIFT))

  __IO uint32_t         CFG2;            /* 0x0c */
#define SPI_CFG2_AFCNTR_SHIFT           (31)
#define SPI_CFG2_AFCNTR_MASK            (3 << (SPI_CFG2_AFCNTR_SHIFT))
#define SPI_CFG2_AFCNTR_NO              (0 << (SPI_CFG2_AFCNTR_SHIFT))
#define SPI_CFG2_AFCNTR_YES             (1 << (SPI_CFG2_AFCNTR_SHIFT))
#define SPI_CFG2_CPOL_SHIFT             (25)
#define SPI_CFG2_CPOL_MASK              (3 << (SPI_CFG2_CPOL_SHIFT))
#define SPI_CFG2_CPOL_NO                (0 << (SPI_CFG2_CPOL_SHIFT))
#define SPI_CFG2_CPOL_YES               (1 << (SPI_CFG2_CPOL_SHIFT))
#define SPI_CFG2_CPHA_SHIFT             (24)
#define SPI_CFG2_CPHA_MASK              (3 << (SPI_CFG2_CPHA_SHIFT))
#define SPI_CFG2_CPHA_NO                (0 << (SPI_CFG2_CPHA_SHIFT))
#define SPI_CFG2_CPHA_YES               (1 << (SPI_CFG2_CPHA_SHIFT))
#define SPI_CFG2_LSBFRST_SHIFT          (23)
#define SPI_CFG2_LSBFRST_MASK           (3 << (SPI_CFG2_LSBFRST_SHIFT))
#define SPI_CFG2_LSBFRST_NO             (0 << (SPI_CFG2_LSBFRST_SHIFT))
#define SPI_CFG2_LSBFRST_YES            (1 << (SPI_CFG2_LSBFRST_SHIFT))
#define SPI_CFG2_MASTER_SHIFT           (22)
#define SPI_CFG2_MASTER_MASK            (3 << (SPI_CFG2_MASTER_SHIFT))
#define SPI_CFG2_MASTER_NO              (0 << (SPI_CFG2_MASTER_SHIFT))
#define SPI_CFG2_MASTER_YES             (1 << (SPI_CFG2_MASTER_SHIFT))
#define SPI_CFG2_SP_SHIFT               (19)
#define SPI_CFG2_SP_MASK                (3 << (SPI_CFG2_SP_SHIFT))
#define SPI_CFG2_SP_MOTOROLA            (0 << (SPI_CFG2_SP_SHIFT))
#define SPI_CFG2_SP_TI                  (1 << (SPI_CFG2_SP_SHIFT))
#define SPI_CFG2_COMM_SHIFT             (17)
#define SPI_CFG2_COMM_MASK              (3 << (SPI_CFG2_COMM_SHIFT))
#define SPI_CFG2_COMM_FULL_DUP          (0 << (SPI_CFG2_COMM_SHIFT))
#define SPI_CFG2_COMM_SIMPLEX_TX        (1 << (SPI_CFG2_COMM_SHIFT))
#define SPI_CFG2_COMM_SIMPLEX_RX        (2 << (SPI_CFG2_COMM_SHIFT))
#define SPI_CFG2_COMM_HALF_DUP          (3 << (SPI_CFG2_COMM_SHIFT))
  __IO uint32_t         IER;            /* 0x10 */
  __IO uint32_t         SR;             /* 0x14 */
#define SPI_SR_RXPCNT_SHIFT             (13)
#define SPI_SR_RXPCNT_MASK              (7 << (SPI_SR_RXPCNT_SHIFT))
  __IO uint32_t         IFCR;           /* 0x18 */
#define SPI_IFCR_CLEAR_ALL_SHIFT        (0)
#define SPI_IFCR_CLEAR_ALL              (0x0ff8 << (SPI_IFCR_CLEAR_ALL_SHIFT))
  uint32_t              reserved1c;     /* 0x1c */
  __IO uint32_t         TXDR;           /* 0x20 */
  uint32_t              reserved24[3];  /* 0x24 */
  __IO uint32_t         RXDR;           /* 0x30 */
  uint32_t              reserved34[3];  /* 0x34 */
  __IO uint32_t         CRCPOLY;        /* 0x40 */
  __IO uint32_t         TXCRC;          /* 0x44 */
  __IO uint32_t         RXCRC;          /* 0x48 */
  __IO uint32_t         UDRDR;          /* 0x4c */
  __IO uint32_t         I2CCFGR;        /* 0x50 */
} stm32Dev_SPI;

/*******************************************
 * 57 USB
 */
typedef struct {
  __IO uint32_t         CTL;
#define USB_EPCTL_EPENA_SHIFT   (31)
#define USB_EPCTL_EPENA_MASK    (1 << (USB_EPCTL_EPENA_SHIFT))
#define USB_EPCTL_EPENA_NO      (0 << (USB_EPCTL_EPENA_SHIFT))
#define USB_EPCTL_EPENA_YES     (1 << (USB_EPCTL_EPENA_SHIFT))
#define USB_EPCTL_SNAK_SHIFT    (27)
#define USB_EPCTL_SNAK_MASK     (1 << (USB_EPCTL_SNAK_SHIFT))
#define USB_EPCTL_SNAK          (1 << (USB_EPCTL_SNAK_SHIFT))
#define USB_EPCTL_CNAK_SHIFT    (26)
#define USB_EPCTL_CNAK_MASK     (1 << (USB_EPCTL_CNAK_SHIFT))
#define USB_EPCTL_CNAK          (1 << (USB_EPCTL_CNAK_SHIFT))
#define USB_EPCTL_TXFNUM_SHIFT  (22)
#define USB_EPCTL_TXFNUM_MASK   (0xf << (USB_EPCTL_TXFNUM_SHIFT))
#define USB_EPCTL_TXFNUM_VAL(x) ((x) << (USB_EPCTL_TXFNUM_SHIFT))
#define USB_EPCTL_STALL_SHIFT   (21)
#define USB_EPCTL_STALL_MASK    (1 << (USB_EPCTL_STALL_SHIFT))
#define USB_EPCTL_STALL_NO      (0 << (USB_EPCTL_STALL_SHIFT))
#define USB_EPCTL_STALL_YES     (1 << (USB_EPCTL_STALL_SHIFT))
#define USB_EPCTL_EPTYP_SHIFT   (18)
#define USB_EPCTL_EPTYP_MASK    (3 << (USB_EPCTL_EPTYP_SHIFT))
#define USB_EPCTL_EPTYP_CTRL    (0 << (USB_EPCTL_EPTYP_SHIFT))
#define USB_EPCTL_EPTYP_ISOC    (1 << (USB_EPCTL_EPTYP_SHIFT))
#define USB_EPCTL_EPTYP_BULK    (2 << (USB_EPCTL_EPTYP_SHIFT))
#define USB_EPCTL_EPTYP_INTR    (3 << (USB_EPCTL_EPTYP_SHIFT))
#define USB_EPCTL_USBAEP_SHIFT  (15)
#define USB_EPCTL_USBAEP_MASK   (1 << (USB_EPCTL_USBAEP_SHIFT))
#define USB_EPCTL_USBAEP_NO     (0 << (USB_EPCTL_USBAEP_SHIFT))
#define USB_EPCTL_USBAEP_YES    (1 << (USB_EPCTL_USBAEP_SHIFT))
  uint32_t              reserved04;
  __IO uint32_t         INT;
#define USB_EPINT_TXFE_SHIFT    (7)
#define USB_EPINT_TXFE_MASK     (1 << (USB_EPINT_TXFE_SHIFT))
#define USB_EPINT_XFRC_SHIFT    (0)
#define USB_EPINT_XFRC_MASK     (1 << (USB_EPINT_XFRC_SHIFT))

  uint32_t              reserved0c;
  __IO uint32_t         SIZ;
#define USB_EPSIZ_STUPCNT_SHIFT (29)
#define USB_EPSIZ_STUPCNT_MASK  (3 << (USB_EPSIZ_STUPCNT_SHIFT))
#define USB_EPSIZ_STUPCNT_1PKTS (1 << (USB_EPSIZ_STUPCNT_SHIFT))
#define USB_EPSIZ_STUPCNT_2PKTS (2 << (USB_EPSIZ_STUPCNT_SHIFT))
#define USB_EPSIZ_STUPCNT_3PKTS (3 << (USB_EPSIZ_STUPCNT_SHIFT))
#define USB_EPSIZ_PKTCNT_SHIFT  (19)
#define USB_EPSIZ_PKTCNT_MASK   (1 << (USB_EPSIZ_PKTCNT_SHIFT))
#define USB_EPSIZ_PKTCNT_ZERO   (0 << (USB_EPSIZ_PKTCNT_SHIFT))
#define USB_EPSIZ_PKTCNT_ONE    (1 << (USB_EPSIZ_PKTCNT_SHIFT))
#define USB_EPSIZ_PKTCNT_TWO    (2 << (USB_EPSIZ_PKTCNT_SHIFT))
#define USB_EPSIZ_PKTCNT_THREE  (3 << (USB_EPSIZ_PKTCNT_SHIFT))
#define USB_EPSIZ_XFRSIZ_SHIFT  (0)
#define USB_EPSIZ_XFRSIZ_MASK   (0x7f << (USB_EPSIZ_XFRSIZ_SHIFT))
#define USB_EPSIZ_XFRSIZ_VAL(x) (((x) << (USB_EPSIZ_XFRSIZ_SHIFT)) & (USB_EPSIZ_XFRSIZ_MASK))
  __IO uint32_t         DMA;            /* out only */
  __IO uint32_t         STS;            /* in only */
  uint32_t              reserved1c;
} stm32Usb320aDevEp_t;

typedef struct {
  /* 0x000 */
  __IO uint32_t         GOTGCTL;
  __IO uint32_t         GOTGINT;
  __IO uint32_t         GAHBCFG;
  __IO uint32_t         GUSBCFG;
  __IO uint32_t         GRSTCTL;
#define USB_GRSTCTL_TXFNUM_SHIFT          (6)
#define USB_GRSTCTL_TXFNUM_MASK           (0x1f << (USB_GRSTCTL_TXFNUM_SHIFT))
#define USB_GRSTCTL_TXFNUM(x)             (((x) << (USB_GRSTCTL_TXFNUM_SHIFT)) & (USB_GRSTCTL_TXFNUM_MASK))
#define USB_GRSTCTL_TXFNUM_ALL             (0x10 << (USB_GRSTCTL_TXFNUM_SHIFT))
#define USB_GRSTCTL_TXFFLSH_SHIFT          (5)
#define USB_GRSTCTL_TXFFLSH_MASK           (1 << (USB_GRSTCTL_TXFFLSH_SHIFT))
#define USB_GRSTCTL_TXFFLSH_DIS            (0 << (USB_GRSTCTL_TXFFLSH_SHIFT))
#define USB_GRSTCTL_TXFFLSH_EN             (1 << (USB_GRSTCTL_TXFFLSH_SHIFT))
#define USB_GRSTCTL_RXFFLSH_SHIFT          (4)
#define USB_GRSTCTL_RXFFLSH_MASK           (1 << (USB_GRSTCTL_RXFFLSH_SHIFT))
#define USB_GRSTCTL_RXFFLSH_DIS            (0 << (USB_GRSTCTL_RXFFLSH_SHIFT))
#define USB_GRSTCTL_RXFFLSH_EN             (1 << (USB_GRSTCTL_RXFFLSH_SHIFT))

  __IO uint32_t         GINTSTS;
  __IO uint32_t         GINTMSK;

  __IO uint32_t         GRXSTSR;                /* debug */
  __IO uint32_t         GRXSTSP;                /* normal use */
#define USB_GRXSTSP_FRMNUM_SHIFT         (21)
#define USB_GRXSTSP_FRMNUM_MASK          (0xf << (USB_GRXSTSP_FRMNUM_SHIFT))
#define USB_GRXSTSP_PKTSTS_SHIFT         (17)
#define USB_GRXSTSP_PKTSTS_MASK          (0xf << (USB_GRXSTSP_PKTSTS_SHIFT))
#define USB_GRXSTSP_PKTSTS_OUT_NAK       (1 << (USB_GRXSTSP_PKTSTS_SHIFT))
#define USB_GRXSTSP_PKTSTS_OUT_RECV      (2 << (USB_GRXSTSP_PKTSTS_SHIFT))
#define USB_GRXSTSP_PKTSTS_INOUT_COMPLETE (3 << (USB_GRXSTSP_PKTSTS_SHIFT))
#define USB_GRXSTSP_PKTSTS_SETUP_COPMLETE (4 << (USB_GRXSTSP_PKTSTS_SHIFT))
#define USB_GRXSTSP_PKTSTS_TOGGLE_ERROR  (5 << (USB_GRXSTSP_PKTSTS_SHIFT))
#define USB_GRXSTSP_PKTSTS_SETUP_RECV    (6 << (USB_GRXSTSP_PKTSTS_SHIFT))
#define USB_GRXSTSP_PKTSTS_CH_HALT       (7 << (USB_GRXSTSP_PKTSTS_SHIFT))
#define USB_GRXSTSP_DPID_SHIFT           (15)
#define USB_GRXSTSP_DPID_MASK            (3 << (USB_GRXSTSP_DPID_SHIFT))
#define USB_GRXSTSP_DPID_DATA0           (0 << (USB_GRXSTSP_DPID_SHIFT))
#define USB_GRXSTSP_DPID_DATA1           (1 << (USB_GRXSTSP_DPID_SHIFT))
#define USB_GRXSTSP_DPID_DATA2           (2 << (USB_GRXSTSP_DPID_SHIFT))
#define USB_GRXSTSP_DPID_MDATA           (3 << (USB_GRXSTSP_DPID_SHIFT))
#define USB_GRXSTSP_BCNT_SHIFT           (4)
#define USB_GRXSTSP_BCNT_MASK            (0x3ff << (USB_GRXSTSP_BCNT_SHIFT))
#define USB_GRXSTSP_EPNUM_SHIFT          (0)
#define USB_GRXSTSP_EPNUM_MASK           (0xf << (USB_GRXSTSP_EPNUM_SHIFT))

  __IO uint32_t         GRXFSIZ;
#define USB_GRXSIZ_RXFD_SHIFT           (0)
#define USB_GRXSIZ_RXFD_MASK            (0xffff << (USB_GRXSIZ_RXFD_SHIFT))
  __IO uint32_t         DIEPTXF0_HNPTXFSIZ;
#define USB_DIEPTXF0SIZ_FD_SHIFT        (16)
#define USB_DIEPTXF0SIZ_FD_MASK         (0xffff << (USB_DIEPTXF0SIZ_FD_SHIFT))
#define USB_DIEPTXF0SIZ_SA_SHIFT        (0)
#define USB_DIEPTXF0SIZ_SA_MASK         (0xffff << (USB_DIEPTXF0SIZ_SA_SHIFT))
  __IO uint32_t         HNPTXSTS;
  __IO uint32_t         GI2CCTL;
  uint32_t              reserved_034;
  __IO uint32_t         GCCFG;
  __IO uint32_t         CID;
  __IO uint32_t         GSNPSID;
  __IO uint32_t         GHWCFG1;
  __IO uint32_t         GHWCFG2;
  __IO uint32_t         GHWCFG3;
  uint32_t              reserved_050;
  __IO uint32_t         GLPMCFG;
  __IO uint32_t         GPWRDN;
  __IO uint32_t         GDFIFOCFG;
  __IO uint32_t         GADPCTL;
  uint32_t              reserved_064[39];
  /* 0x100 */
  __IO uint32_t         HPTXFSIZ;
  __IO uint32_t         DIEPTXF[15];

  /* 0x140 */
  uint32_t              reserved_140[0xb0];

  /* 0x400   host controller */
  __IO uint32_t         HCFGM;
  __IO uint32_t         HFIR;
  __IO uint32_t         HAFNUM;
  uint32_t              reserved_40c;
  __IO uint32_t         XPTXSTS;
  __IO uint32_t         HAINT;
  __IO uint32_t         HAINTMSK;
  uint32_t              reserved_41c[0x9];
  __IO uint32_t         HPRT;
  uint32_t              reserved_444[0xef];

  /* 0x800   device controller */
  __IO uint32_t         DCFG;
#define USB_DCFG_DAD_SHIFT              (4)
#define USB_DCFG_DAD_MASK               (0x7f << (USB_DCFG_DAD_SHIFT))
#define USB_DCFG_DSPD_SHIFT             (0)
#define USB_DCFG_DSPD_MASK              (3 << (USB_DCFG_DSPD_SHIFT))
#define USB_DCFG_DSPD_HS                (0 << (USB_DCFG_DSPD_SHIFT))
#define USB_DCFG_DSPD_FS_IN_HS          (1 << (USB_DCFG_DSPD_SHIFT))
#define USB_DCFG_DSPD_RESERVED2         (2 << (USB_DCFG_DSPD_SHIFT))
#define USB_DCFG_DSPD_FS                (3 << (USB_DCFG_DSPD_SHIFT))

  __IO uint32_t         DCTL;
#define USB_DCTL_CGINAK_SHIFT             (8)
#define USB_DCTL_CGINAK                 (1 << (USB_DCTL_CGINAK_SHIFT))
#define USB_DCTL_SDIS_SHIFT             (1)
#define USB_DCTL_SDIS_MASK              (1 << (USB_DCTL_SDIS_SHIFT))
#define USB_DCTL_SDIS_CONNECT           (0 << (USB_DCTL_SDIS_SHIFT))
#define USB_DCTL_SDIS_DISCONNECT        (1 << (USB_DCTL_SDIS_SHIFT))
  __IO uint32_t         DSTS;
#define USB_DSTS_ENUMSPD_SHIFT          (1)
#define USB_DSTS_ENUMSPD_MASK           (3 << (USB_DSTS_ENUMSPD_SHIFT))
#define USB_DSTS_ENUMSPD_HIGH           (0 << (USB_DSTS_ENUMSPD_SHIFT))
#define USB_DSTS_ENUMSPD_FULL           (3 << (USB_DSTS_ENUMSPD_SHIFT))

  uint32_t              Reserved_80c;
  __IO uint32_t         DIEPMSK;
  __IO uint32_t         DOEPMSK;
  __IO uint32_t         DAINT;
#define USB_DAINT_OEPINT_SHIFT          (16)
#define USB_DAINT_OEPINT_MASK           (0xffff << (USB_DAINT_OEPINT_SHIFT))
#define USB_DAINT_OEPINT_BIT(x)         (((1<<(x)) << (USB_DAINT_OEPINT_SHIFT)) & (USB_DAINT_OEPINT_MASK))
#define USB_DAINT_IEPINT_SHIFT          (0)
#define USB_DAINT_IEPINT_MASK           (0xffff << (USB_DAINT_IEPINT_SHIFT))
#define USB_DAINT_IEPINT_BIT(x)         (((1<<(x)) << (USB_DAINT_IEPINT_SHIFT)) & (USB_DAINT_IEPINT_MASK))

  __IO uint32_t         DAINTMSK;
  uint32_t              reserved_820;
  uint32_t              reserved_824;
  __IO uint32_t         DVBUSDIS;
  __IO uint32_t         DVBUSPULSE;
  __IO uint32_t         DTHRCTL;
  __IO uint32_t         DIEPEMPMSK;
  __IO uint32_t         DEACHINT;
  __IO uint32_t         DEACHINTMSK;
  uint32_t              reserved_840;
  __IO uint32_t         DINEP1MSK;
  uint32_t              reserved_848;
  __IO uint32_t         DOUTEP1MSK;
  uint32_t              reserved_850[0x2c];

  /* 0x900 */
  stm32Usb320aDevEp_t   in[16];         /* 0x20 * 0x10pcs = 0x200 */

  /* 0xb00 */
  stm32Usb320aDevEp_t   out[16];        /* 0x20 * 0x10pcs = 0x200 */

  /* 0xd00 */
  uint32_t              reserved_d00[0x40];


  /* 0xe00 */
  __IO uint32_t         PCGCCTL;
  uint32_t              reserved_e04[0x7f];

  /* 0x1000  FIFO */
#define USB_FIFO_SIZE   0x1000
  __IO uint32_t         DFIFO[15][USB_FIFO_SIZE/4];

} stm32Usb320aDev_t;


#define USB1_HS                 ((stm32Usb320aDev_t *) (BUS_PERIPHERAL+0x40000))
#define USB2_FS                 ((stm32Usb320aDev_t *) (BUS_PERIPHERAL+0x80000))


#define USB_MODULE_TBL                  {NULL, USB1_OTG_HS, USB2_OTG_FS}


#define USB_OTG_CORE_ID_310A            0x4F54310A
#define USB_OTG_CORE_ID_320A            0x4F54320A

/*** 0x08 OTG_GAHBCFG     x is available to set 1 to 16 */
#define USB_GAHBCFG_HBSTLEN_SHIFT       (1)
#define USB_GAHBCFG_HBSTLEN_MASK        (15 << (USB_GAHBCFG_HBSTLEN_SHIFT))
#define USB_GAHBCFG_HBSTLEN_1WORD       (0 << (USB_GAHBCFG_HBSTLEN_SHIFT))
#define USB_GAHBCFG_HBSTLEN_4WORDS      (3 << (USB_GAHBCFG_HBSTLEN_SHIFT))
#define USB_GAHBCFG_HBSTLEN_8WORDS      (3 << (USB_GAHBCFG_HBSTLEN_SHIFT))
#define USB_GAHBCFG_HBSTLEN_16WORDS     (7 << (USB_GAHBCFG_HBSTLEN_SHIFT))


/*** 0x0c OTG_GUSBCFG */
#define USB_GUSBCFG_TRDT_SHIFT          (10)
#define USB_GUSBCFG_TRDT_MASK           (15 << (USB_GUSBCFG_TRDT_SHIFT))
#define USB_GUSBCFG_TRDT_OVER32_0MHZ    (6 << (USB_GUSBCFG_TRDT_SHIFT))
#define USB_GUSBCFG_TRDT_27_5TO32_0MHZ  (7 << (USB_GUSBCFG_TRDT_SHIFT))
#define USB_GUSBCFG_TRDT_24_0TO27_5MHZ  (8 << (USB_GUSBCFG_TRDT_SHIFT))
#define USB_GUSBCFG_TRDT_21_8TO24_0MHZ  (9 << (USB_GUSBCFG_TRDT_SHIFT))
#define USB_GUSBCFG_TRDT_20_0TO21_8MHZ  (10 << (USB_GUSBCFG_TRDT_SHIFT))
#define USB_GUSBCFG_TRDT_18_5TO20_0MHZ  (11 << (USB_GUSBCFG_TRDT_SHIFT))
#define USB_GUSBCFG_TRDT_17_2TO18_5MHZ  (12 << (USB_GUSBCFG_TRDT_SHIFT))
#define USB_GUSBCFG_TRDT_16_0TO17_2MHZ  (13 << (USB_GUSBCFG_TRDT_SHIFT))
#define USB_GUSBCFG_TRDT_15_0TO16_0MHZ  (14 << (USB_GUSBCFG_TRDT_SHIFT))
#define USB_GUSBCFG_TRDT_14_2TO15_0MHZ  (15 << (USB_GUSBCFG_TRDT_SHIFT))


#endif
