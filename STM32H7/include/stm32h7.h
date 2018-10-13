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

  DMA1_STR0_IRQn        =    11,
  DMA1_STR1_IRQn,
  DMA1_STR2_IRQn,
  DMA1_STR3_IRQn,
  DMA1_STR4_IRQn,
  DMA1_STR5_IRQn,
  DMA1_STR6_IRQn,
  TIM2_IRQn             =    28,
  TIM3_IRQn,
  TIM4_IRQn,

  I2C1EV_IRQn             =    31,
  I2C1ER_IRQn,
  I2C2EV_IRQn,
  I2C2ER_IRQn,
  SPI1_IRQn,
  SPI2_IRQn,
  USART1_IRQn,
  USART2_IRQn,
  USART3_IRQn,

  DMA1_STR7_IRQn        =    47,

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
 * 14 MDMA
 */
/*******************************************
 * 15 DMA1,2
 */


/* DMA CR */
#if 0

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
#endif



typedef struct {
  __IO uint32_t         CR;
#define DMA_CR_MBURST_SHIFT     (23)
#define DMA_CR_MBURST_MASK      (3 << (DMA_CR_MBURST_SHIFT))
#define DMA_CR_MBURST_1         (0 << (DMA_CR_MBURST_SHIFT))
#define DMA_CR_MBURST_4         (1 << (DMA_CR_MBURST_SHIFT))
#define DMA_CR_MBURST_8         (2 << (DMA_CR_MBURST_SHIFT))
#define DMA_CR_MBURST_16        (3 << (DMA_CR_MBURST_SHIFT))
#define DMA_CR_PBURST_SHIFT     (21)
#define DMA_CR_PBURST_MASK      (3 << (DMA_CR_PBURST_SHIFT))
#define DMA_CR_PBURST_1         (0 << (DMA_CR_PBURST_SHIFT))
#define DMA_CR_PBURST_4         (1 << (DMA_CR_PBURST_SHIFT))
#define DMA_CR_PBURST_8         (2 << (DMA_CR_PBURST_SHIFT))
#define DMA_CR_PBURST_16        (3 << (DMA_CR_PBURST_SHIFT))
#define DMA_CR_CT_SHIFT         (19)
#define DMA_CR_CT_MASK          (1 << (DMA_CR_CT_SHIFT))
#define DMA_CR_CT_M0AR          (0 << (DMA_CR_CT_SHIFT))
#define DMA_CR_CT_M1AR          (1 << (DMA_CR_CT_SHIFT))
#define DMA_CR_DBM_SHIFT        (18)
#define DMA_CR_DBM_MASK         (1 << (DMA_CR_DBM_SHIFT))
#define DMA_CR_DBM_NO           (0 << (DMA_CR_DBM_SHIFT))
#define DMA_CR_DBM_YES          (1 << (DMA_CR_DBM_SHIFT))
#define DMA_CR_PL_SHIFT         (16)
#define DMA_CR_PL_MASK          (1 << (DMA_CR_PL_SHIFT))
#define DMA_CR_PL_LOW           (0 << (DMA_CR_PL_SHIFT))
#define DMA_CR_PL_MED           (1 << (DMA_CR_PL_SHIFT))
#define DMA_CR_PL_HIGH          (2 << (DMA_CR_PL_SHIFT))
#define DMA_CR_PL_VHIGH         (3 << (DMA_CR_PL_SHIFT))
#define DMA_CR_PINCOS_SHIFT     (15)
#define DMA_CR_PINCOS_MASK      (1 << (DMA_CR_PINCOS_SHIFT))
#define DMA_CR_PINCOS_PSIZE     (0 << (DMA_CR_PINCOS_SHIFT))
#define DMA_CR_PINCOS_WORD      (1 << (DMA_CR_PINCOS_SHIFT))
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
#define DMA_CR_MINC_SHIFT       (10)
#define DMA_CR_MINC_MASK        (1 << (DMA_CR_MINC_SHIFT))
#define DMA_CR_MINC_FIXED       (0 << (DMA_CR_MINC_SHIFT))
#define DMA_CR_MINC_INC         (1 << (DMA_CR_MINC_SHIFT))
#define DMA_CR_PINC_SHIFT       (9)
#define DMA_CR_PINC_MASK        (1 << (DMA_CR_PINC_SHIFT))
#define DMA_CR_PINC_FIXED       (0 << (DMA_CR_PINC_SHIFT))
#define DMA_CR_PINC_INC         (1 << (DMA_CR_PINC_SHIFT))
#define DMA_CR_CIRC_SHIFT       (8)
#define DMA_CR_CIRC_MASK        (1 << (DMA_CR_CIRC_SHIFT))
#define DMA_CR_CIRC_NO          (0 << (DMA_CR_CIRC_SHIFT))
#define DMA_CR_CIRC_YES         (1 << (DMA_CR_CIRC_SHIFT))
#define DMA_CR_DIR_SHIFT        (6)
#define DMA_CR_DIR_MASK         (1 << (DMA_CR_DIR_SHIFT))
#define DMA_CR_DIR_PTOM         (0 << (DMA_CR_DIR_SHIFT))
#define DMA_CR_DIR_MTOP         (1 << (DMA_CR_DIR_SHIFT))
#define DMA_CR_DIR_MTOM         (2 << (DMA_CR_DIR_SHIFT))
#define DMA_CR_DIR_RESERVED     (3 << (DMA_CR_DIR_SHIFT))
#define DMA_CR_PFCTRL_SHIFT     (5)
#define DMA_CR_PFCTRL_MASK      (1 << (DMA_CR_PFCTRL_SHIFT))
#define DMA_CR_PFCTRL_DMA       (0 << (DMA_CR_PFCTRL_SHIFT))
#define DMA_CR_PFCTRL_PERI      (1 << (DMA_CR_PFCTRL_SHIFT))
#define DMA_CR_TCIE_SHIFT       (4)
#define DMA_CR_TCIE_MASK        (1 << (DMA_CR_TCIE_SHIFT))
#define DMA_CR_TCIE_DIS         (0 << (DMA_CR_TCIE_SHIFT))
#define DMA_CR_TCIE_EN          (1 << (DMA_CR_TCIE_SHIFT))
#define DMA_CR_HTIE_SHIFT       (3)
#define DMA_CR_HTIE_MASK        (1 << (DMA_CR_HTIE_SHIFT))
#define DMA_CR_HTIE_DIS         (0 << (DMA_CR_HTIE_SHIFT))
#define DMA_CR_HTIE_EN          (1 << (DMA_CR_HTIE_SHIFT))
#define DMA_CR_TEIE_SHIFT       (2)
#define DMA_CR_TEIE_MASK        (1 << (DMA_CR_TEIE_SHIFT))
#define DMA_CR_TEIE_DIS         (0 << (DMA_CR_TEIE_SHIFT))
#define DMA_CR_TEIE_EN          (1 << (DMA_CR_TEIE_SHIFT))
#define DMA_CR_DMEIE_SHIFT      (1)
#define DMA_CR_DMEIE_MASK       (1 << (DMA_CR_DMEIE_SHIFT))
#define DMA_CR_DMEIE_DIS        (0 << (DMA_CR_DMEIE_SHIFT))
#define DMA_CR_DMEIE_EN         (1 << (DMA_CR_DMEIE_SHIFT))
#define DMA_CR_EN_SHIFT         (0)
#define DMA_CR_EN_MASK          (1 << (DMA_CR_EN_SHIFT))
#define DMA_CR_EN_NO            (0 << (DMA_CR_EN_SHIFT))
#define DMA_CR_EN_YES           (1 << (DMA_CR_EN_SHIFT))
  __IO uint32_t         NDTR;
  __IO uint32_t         PAR;
  __IO uint32_t         M0AR;
  __IO uint32_t         M1AR;
  __IO uint32_t         FCR;
} stm32Dev_DMA_Stream;

typedef struct {
  __IO uint32_t         LISR;
/* DMA LISR, HISR, LIFCR, HIFCR */
#define DMA_LISR_S3_SHIFT       (22)
#define DMA_LISR_ALL3_MASK      (0x7d << (DMA_LISR_S3_SHIFT))
#define DMA_LISR_FEIF3_MASK     (1 << ((DMA_LISR_S3_SHIFT)+0))
#define DMA_LISR_DMEIF3_MASK    (1 << ((DMA_LISR_S3_SHIFT)+2))
#define DMA_LISR_TEIF3_MASK     (1 << ((DMA_LISR_S3_SHIFT)+3))
#define DMA_LISR_HTIF3_MASK     (1 << ((DMA_LISR_S3_SHIFT)+4))
#define DMA_LISR_TCIF3_MASK     (1 << ((DMA_LISR_S3_SHIFT)+5))
#define DMA_LISR_S2_SHIFT       (16)
#define DMA_LISR_ALL2_MASK      (0x7d << (DMA_LISR_S2_SHIFT))
#define DMA_LISR_FEIF2_MASK     (1 << ((DMA_LISR_S2_SHIFT)+0))
#define DMA_LISR_DMEIF2_MASK    (1 << ((DMA_LISR_S2_SHIFT)+2))
#define DMA_LISR_TEIF2_MASK     (1 << ((DMA_LISR_S2_SHIFT)+3))
#define DMA_LISR_HTIF2_MASK     (1 << ((DMA_LISR_S2_SHIFT)+4))
#define DMA_LISR_TCIF2_MASK     (1 << ((DMA_LISR_S2_SHIFT)+5))
#define DMA_LISR_S1_SHIFT       (6)
#define DMA_LISR_ALL1_MASK      (0x7d << (DMA_LISR_S1_SHIFT))
#define DMA_LISR_FEIF1_MASK     (1 << ((DMA_LISR_S1_SHIFT)+0))
#define DMA_LISR_DMEIF1_MASK    (1 << ((DMA_LISR_S1_SHIFT)+2))
#define DMA_LISR_TEIF1_MASK     (1 << ((DMA_LISR_S1_SHIFT)+3))
#define DMA_LISR_HTIF1_MASK     (1 << ((DMA_LISR_S1_SHIFT)+4))
#define DMA_LISR_TCIF1_MASK     (1 << ((DMA_LISR_S1_SHIFT)+5))
#define DMA_LISR_S0_SHIFT       (0)
#define DMA_LISR_ALL0_MASK      (0x7d << (DMA_LISR_S0_SHIFT))
#define DMA_LISR_FEIF0_MASK     (1 << ((DMA_LISR_S0_SHIFT)+0))
#define DMA_LISR_DMEIF0_MASK    (1 << ((DMA_LISR_S0_SHIFT)+2))
#define DMA_LISR_TEIF0_MASK     (1 << ((DMA_LISR_S0_SHIFT)+3))
#define DMA_LISR_HTIF0_MASK     (1 << ((DMA_LISR_S0_SHIFT)+4))
#define DMA_LISR_TCIF0_MASK     (1 << ((DMA_LISR_S0_SHIFT)+5))

  __IO uint32_t         HISR;
#define DMA_HISR_S7_SHIFT       (22)
#define DMA_HISR_ALL7_MASK      (0x7d << (DMA_HISR_S7_SHIFT))
#define DMA_HISR_FEIF7_MASK     (1 << ((DMA_HISR_S7_SHIFT)+0))
#define DMA_HISR_DMEIF7_MASK    (1 << ((DMA_HISR_S7_SHIFT)+2))
#define DMA_HISR_TEIF7_MASK     (1 << ((DMA_HISR_S7_SHIFT)+3))
#define DMA_HISR_HTIF7_MASK     (1 << ((DMA_HISR_S7_SHIFT)+4))
#define DMA_HISR_TCIF7_MASK     (1 << ((DMA_HISR_S7_SHIFT)+5))
#define DMA_HISR_S6_SHIFT       (16)
#define DMA_HISR_ALL6_MASK      (0x7d << (DMA_HISR_S6_SHIFT))
#define DMA_HISR_FEIF6_MASK     (1 << ((DMA_HISR_S6_SHIFT)+0))
#define DMA_HISR_DMEIF6_MASK    (1 << ((DMA_HISR_S6_SHIFT)+2))
#define DMA_HISR_TEIF6_MASK     (1 << ((DMA_HISR_S6_SHIFT)+3))
#define DMA_HISR_HTIF6_MASK     (1 << ((DMA_HISR_S6_SHIFT)+4))
#define DMA_HISR_TCIF6_MASK     (1 << ((DMA_HISR_S6_SHIFT)+5))
#define DMA_HISR_S5_SHIFT       (6)
#define DMA_HISR_ALL5_MASK      (0x7d << (DMA_HISR_S5_SHIFT))
#define DMA_HISR_FEIF5_MASK     (1 << ((DMA_HISR_S5_SHIFT)+0))
#define DMA_HISR_DMEIF5_MASK    (1 << ((DMA_HISR_S5_SHIFT)+2))
#define DMA_HISR_TEIF5_MASK     (1 << ((DMA_HISR_S5_SHIFT)+3))
#define DMA_HISR_HTIF5_MASK     (1 << ((DMA_HISR_S5_SHIFT)+4))
#define DMA_HISR_TCIF5_MASK     (1 << ((DMA_HISR_S5_SHIFT)+5))
#define DMA_HISR_S4_SHIFT       (0)
#define DMA_HISR_ALL4_MASK      (0x7d << (DMA_HISR_S4_SHIFT))
#define DMA_HISR_FEIF4_MASK     (1 << ((DMA_HISR_S4_SHIFT)+0))
#define DMA_HISR_DMEFIF4_MASK   (1 << ((DMA_HISR_S4_SHIFT)+2))
#define DMA_HISR_TEIF4_MASK     (1 << ((DMA_HISR_S4_SHIFT)+3))
#define DMA_HISR_HTIF4_MASK     (1 << ((DMA_HISR_S4_SHIFT)+4))
#define DMA_HISR_TCIF4_MASK     (1 << ((DMA_HISR_S4_SHIFT)+5))

  __IO uint32_t         LIFCR;
  __IO uint32_t         HIFCR;
  stm32Dev_DMA_Stream   S[8];
} stm32Dev_DMA;

#define DMA1_PTR        ((stm32Dev_DMA *) (AHB1_BASE + 0x0000))
#define DMA2_PTR        ((stm32Dev_DMA *) (AHB1_BASE + 0x0400))




/*******************************************
 * 16 BDMA 
 */


/*******************************************
 * 17 DMAMUX
 */
typedef struct {
  __IO uint32_t         CCR[16];        /* 0x00 */
  uint32_t              reserved40[16];
  __IO uint32_t         CSR;            /* 0x80 */
  __IO uint32_t         CFR;            /* 0x84 */
  uint32_t              reserved8c[30];
  __IO uint32_t         RGxCR[4];       /* 0x100 */
  uint32_t              reserved110[14];
  __IO uint32_t         RGSR;           /* 0x140 */
  __IO uint32_t         RGCFR;          /* 0x144 */
  __IO uint32_t         RGSR2;          /* 0x148 */
  __IO uint32_t         RGCFR2;         /* 0x14c */
  
  
} stm32Dev_DMAMUX;

#define DMAMUX1_PTR     ((stm32Dev_DMAMUX *) ((AHB1_BASE) + 0x0800))
#define DMAMUX2_PTR     ((stm32Dev_DMAMUX *) ((AHB4_BASE) + 0x8000))

enum stm32Dev_DmaReq {
  /* DMAMUX1 */
  DMAMUX_REQ_ADC1 = 9,
  DMAMUX_REQ_ADC2,

  DMAMUX_REQ_SPI6RX = 11,
  DMAMUX_REQ_SPI6TX,

  DMAMUX_REQ_TIM2CH1 = 18,
  DMAMUX_REQ_TIM2CH2,
  DMAMUX_REQ_TIM2CH3,
  DMAMUX_REQ_TIM2CH4,
  DMAMUX_REQ_TIM2UP,
  DMAMUX_REQ_TIM3CH1,
  DMAMUX_REQ_TIM3CH2,
  DMAMUX_REQ_TIM3CH3,
  DMAMUX_REQ_TIM3CH4,
  DMAMUX_REQ_TIM3UP,
  DMAMUX_REQ_TIM3TRG,
  DMAMUX_REQ_TIM4CH1,
  DMAMUX_REQ_TIM4CH2,
  DMAMUX_REQ_TIM4CH3,
  DMAMUX_REQ_TIM4UP,

  DMAMUX_REQ_I2C1RX = 33,
  DMAMUX_REQ_I2C1TX,
  DMAMUX_REQ_I2C2RX,
  DMAMUX_REQ_I2C2TX,
  DMAMUX_REQ_SPI1RX,
  DMAMUX_REQ_SPI1TX,
  DMAMUX_REQ_SPI2RX,
  DMAMUX_REQ_SPI2TX,


  DMAMUX_REQ_SPI3RX = 61,
  DMAMUX_REQ_SPI3TX,


  DMAMUX_REQ_DAC1 = 67,
  DMAMUX_REQ_DAC2,
  DMAMUX_REQ_I2C3RX = 73,
  DMAMUX_REQ_I2C3TX,
  DMAMUX_REQ_SPI4RX = 83,
  DMAMUX_REQ_SPI4TX,
  DMAMUX_REQ_SPI5RX,
  DMAMUX_REQ_SPI5TX,


  /* DMAMUX2 */
  DMAMUX_REQ_I2C4RX = 13,
  DMAMUX_REQ_I2C4TX,
};

#define DMAMUX_MODULE_I2CRX_TBL {0, 0, 0, 0, 0x20}        /* [7:4]: module number, [3:0]: reseved */
#define DMAMUX_MODULE_I2CTX_TBL {0, 0, 0, 0, 0x20}        /* [7:4]: module number, [3:0]: reseved */
#define DMAMUX_REQ_I2CRX_TBL   {0, DMAMUX_REQ_I2C1RX, DMAMUX_REQ_I2C2RX, DMAMUX_REQ_I2C3RX, DMAMUX_REQ_I2C4RX}
#define DMAMUX_REQ_I2CTX_TBL   {0, DMAMUX_REQ_I2C1TX, DMAMUX_REQ_I2C2TX, DMAMUX_REQ_I2C3TX, DMAMUX_REQ_I2C4TX}
  
#define DMAMUX_MODULE_SPIRX_TBL {0, 0, 0, 0, 0, 0, 0x20}        /* [7:4]: module number, [3:0]: reseved */
#define DMAMUX_MODULE_SPITX_TBL {0, 0, 0, 0, 0, 0, 0x20}        /* [7:4]: module number, [3:0]: reseved */
#define DMAMUX_REQ_SPIRX_TBL   {0, DMAMUX_REQ_SPI1RX, DMAMUX_REQ_SPI2RX, DMAMUX_REQ_SPI3RX, DMAMUX_REQ_SPI4RX, DMAMUX_REQ_SPI5RX, DMAMUX_REQ_SPI6RX}
#define DMAMUX_REQ_SPITX_TBL   {0, DMAMUX_REQ_SPI1TX, DMAMUX_REQ_SPI2TX, DMAMUX_REQ_SPI3TX, DMAMUX_REQ_SPI4TX, DMAMUX_REQ_SPI5TX, DMAMUX_REQ_SPI6TX}


/*******************************************
 * 22 FMC
 */
typedef struct {
  __IO uint32_t         CR;
#define FMC_BCR_FMCEN_SHIFT     (31)
#define FMC_BCR_FMCEN_MASK      (1 << (FMC_BCR_FMCEN_SHIFT))
#define FMC_BCR_FMCEN_NO        (0 << (FMC_BCR_FMCEN_SHIFT))
#define FMC_BCR_FMCEN_YES       (1 << (FMC_BCR_FMCEN_SHIFT))
#define FMC_BCR_ASYNCWAIT_SHIFT (15)
#define FMC_BCR_ASYNCWAIT_MASK  (1 << (FMC_BCR_ASYNCWAIT_SHIFT))
#define FMC_BCR_ASYNCWAIT_NO    (0 << (FMC_BCR_ASYNCWAIT_SHIFT))
#define FMC_BCR_ASYNCWAIT_YES   (1 << (FMC_BCR_ASYNCWAIT_SHIFT))
#define FMC_BCR_WREN_SHIFT      (12)
#define FMC_BCR_WREN_MASK       (1 << (FMC_BCR_WREN_SHIFT))
#define FMC_BCR_WREN_NO         (0 << (FMC_BCR_WREN_SHIFT))
#define FMC_BCR_WREN_YES        (1 << (FMC_BCR_WREN_SHIFT))
#define FMC_BCR_MWID_SHIFT      (4)
#define FMC_BCR_MWID_MASK       (3 << (FMC_BCR_MWID_SHIFT))
#define FMC_BCR_MWID_8BIT       (0 << (FMC_BCR_MWID_SHIFT))
#define FMC_BCR_MWID_16BIT      (1 << (FMC_BCR_MWID_SHIFT))
#define FMC_BCR_MWID_32BIT      (2 << (FMC_BCR_MWID_SHIFT))
#define FMC_BCR_MWID_RESERVED   (3 << (FMC_BCR_MWID_SHIFT))
#define FMC_BCR_MTYP_SHIFT      (2)
#define FMC_BCR_MTYP_MASK       (3 << (FMC_BCR_MTYP_SHIFT))
#define FMC_BCR_MTYP_SRAM       (0 << (FMC_BCR_MTYP_SHIFT))
#define FMC_BCR_MTYP_PSRAM      (1 << (FMC_BCR_MTYP_SHIFT))
#define FMC_BCR_MTYP_NORFLASH   (2 << (FMC_BCR_MTYP_SHIFT))
#define FMC_BCR_MTYP_RESERVED   (3 << (FMC_BCR_MTYP_SHIFT))
#define FMC_BCR_MUX_SHIFT       (1)
#define FMC_BCR_MUXEN_MASK      (1 << (FMC_BCR_MUXEN_SHIFT))
#define FMC_BCR_MUXEN_NO        (0 << (FMC_BCR_MUXEN_SHIFT))
#define FMC_BCR_MUXEN_YES       (1 << (FMC_BCR_MUXEN_SHIFT))
#define FMC_BCR_MBKEN_SHIFT     (0)
#define FMC_BCR_MBKEN_MASK      (1 << (FMC_BCR_MBKEN_SHIFT))
#define FMC_BCR_MBKEN_NO        (0 << (FMC_BCR_MBKEN_SHIFT))
#define FMC_BCR_MBKEN_YES       (1 << (FMC_BCR_MBKEN_SHIFT))

  __IO uint32_t         TR;
#define FMC_BTR_DATAST_SHIFT    (8)
#define FMC_BTR_DATAST_MASK     (0xff << (FMC_BTR_DATAST_SHIFT))
#define FMC_BTR_DATAST(x)       ((x)  << (FMC_BTR_DATAST_SHIFT))
} stm32Dev_FMC_bank;

typedef struct {
  stm32Dev_FMC_bank     B[4];           /*  0x00 --  0x1c */
  uint32_t              reserved20[0x18];
  __IO uint32_t         PCR;            /* 0x80 */
  __IO uint32_t         SR;             /* 0x84 */
  __IO uint32_t         PATT;           /* 0x88 */
  uint32_t              reserved90[2];
  __IO uint32_t         ECCR;           /* 0x94 */
  uint32_t              reserved9c[0x2a];
  stm32Dev_FMC_bank     BW[4];          /* 0x100 -- 0x11c */
  uint32_t              reserved120[8];
  __IO uint32_t         SDCR1;          /* 0x140 */
  __IO uint32_t         SDCR2;          /* 0x144 */
  __IO uint32_t         SDTR1;          /* 0x148 */
  __IO uint32_t         SDTR2;          /* 0x14c */
  __IO uint32_t         SDCMR;          /* 0x150 */
  __IO uint32_t         SDRTR;          /* 0x154 */
  __IO uint32_t         SDSR;           /* 0x148 */


  __IO uint32_t         BCR[4];           /* 0x00 */

} stm32Dev_FMC;

#define FMC_PTR         ((stm32Dev_FMC *) (AHB3_BASE + 0x4000))


/*******************************************
 * 25 ADC
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
#define ADC_CR_BOOST_SHIFT      (8)
#define ADC_CR_BOOST_MASK       (1 << (ADC_CR_BOOST_SHIFT))
#define ADC_CR_BOOST_NO         (0 << (ADC_CR_BOOST_SHIFT))
#define ADC_CR_BOOST_YES        (1 << (ADC_CR_BOOST_SHIFT))
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
  __IO uint32_t         CFR;            /* 0x30c */
  __IO uint32_t         CFR2;           /* 0x30c */
} stm32Dev_ADC;

#define ADC1_PTR        ((stm32Dev_ADC *) ((AHB1_BASE) + 0x2000))
#define ADC2_PTR        ((stm32Dev_ADC *) ((AHB1_BASE) + 0x2100))
#define ADC3_PTR        ((stm32Dev_ADC *) ((AHB4_BASE) + 0x6000))


/*******************************************
 * 26 DAC
 */
typedef struct {
  __IO uint32_t         CR;             /* 0x00 */
#define DAC_CR_TSEL_SHIFT               (2)
#define DAC_CR_TSEL_MASK                (0xf << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_SWTRIG              (0 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_TIM1_TRGO           (1 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_TIM2_TRGO           (2 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_TIM4_TRGO           (3 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_TIM5_TRGO           (4 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_TIM6_TRGO           (5 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_TIM7_TRGO           (6 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_TIM8_TRGO           (7 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_TIM15_TRGO          (8 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_HRTIM_DACTRG1       (9 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_HRTIM_DACTRG2       (10 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_LPTIM1              (11 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_LPTIM2              (12 << (DAC_CR_TSEL_SHIFT))
#define DAC_CR_TSEL_EXTI9               (13 << (DAC_CR_TSEL_SHIFT))

#define DAC_CR_DMAEN_SHIFT              (12)
#define DAC_CR_DMAEN_MASK               (1 << (DAC_CR_DMAEN_SHIFT))
#define DAC_CR_DMAEN_NO                 (0 << (DAC_CR_DMAEN_SHIFT))
#define DAC_CR_DMAEN_YES                (1 << (DAC_CR_DMAEN_SHIFT))
#define DAC_CR_TEN_SHIFT                (1)
#define DAC_CR_TEN_MASK                 (1 << (DAC_CR_TEN_SHIFT))
#define DAC_CR_TEN_NO                   (0 << (DAC_CR_TEN_SHIFT))
#define DAC_CR_TEN_YES                  (1 << (DAC_CR_TEN_SHIFT))
#define DAC_CR_EN_SHIFT                 (0)
#define DAC_CR_EN_MASK                  (1 << (DAC_CR_EN_SHIFT))
#define DAC_CR_EN_NO                    (0 << (DAC_CR_EN_SHIFT))
#define DAC_CR_EN_YES                   (1 << (DAC_CR_EN_SHIFT))
  __IO uint32_t         SWTRGR;         /* 0x04 */

  __IO uint32_t         DHR12R1;        /* 0x08 ch1 12bit right */
  __IO uint32_t         DHR12L1;        /* 0x0c ch1 12bit left  */
  __IO uint32_t         DHR8R1;         /* 0x10 ch1  8bit right */
  __IO uint32_t         DHR12R2;        /* 0x14 ch2 12bit right */
  __IO uint32_t         DHR12L2;        /* 0x18 ch2 12bit left  */
  __IO uint32_t         DHR8R2;         /* 0x1c ch2  8bit right */

  __IO uint32_t         DHR12RD;        /* 0x20 both ch 12bit right */
  __IO uint32_t         DHR12LD;        /* 0x24 both ch 12bit left */
  __IO uint32_t         DHR8LD;         /* 0x28 both ch  8bit right */

  __IO uint32_t         DOR1;           /* 0x2c ch1 */
  __IO uint32_t         DOR2;           /* 0x30 ch2 */

  __IO uint32_t         SR;             /* 0x34 */
  __IO uint32_t         CCR;            /* 0x38 */
  __IO uint32_t         MCR;            /* 0x3c */

  __IO uint32_t         SHSR1;          /* 0x40 */
  __IO uint32_t         SHSR2;          /* 0x44 */
  __IO uint32_t         SHHR;           /* 0x48 */
  __IO uint32_t         SHRR;           /* 0x4c */

} stm32Dev_DAC;

#define DAC1_PTR        ((stm32Dev_DAC *) ((APB1_BASE) + 0x7400))


/*******************************************
 * 39 TIM2,3,4,5
 * 40 TIM
 * 41 TIM
 * 42 TIM
 */
#define TIM_MODULE_COUNT                (17)

#include        "stm32Tim.h"

#define TIM2_PTR        ((stm32Dev_TIM *) (APB1_BASE + 0x0000))
#define TIM3_PTR        ((stm32Dev_TIM *) (APB1_BASE + 0x0400))
#define TIM4_PTR        ((stm32Dev_TIM *) (APB1_BASE + 0x0800))
#define TIM5_PTR        ((stm32Dev_TIM *) (APB1_BASE + 0x0c00))
#define TIM6_PTR        ((stm32Dev_TIM *) (APB1_BASE + 0x1000))
#define TIM7_PTR        ((stm32Dev_TIM *) (APB1_BASE + 0x1400))
#define TIM12_PTR       ((stm32Dev_TIM *) (APB1_BASE + 0x1800))
#define TIM13_PTR       ((stm32Dev_TIM *) (APB1_BASE + 0x1c00))
#define TIM14_PTR       ((stm32Dev_TIM *) (APB1_BASE + 0x2000))

#define TIM1_PTR        ((stm32Dev_TIM *) (APB2_BASE + 0x0000))
#define TIM8_PTR        ((stm32Dev_TIM *) (APB2_BASE + 0x0400))
#define TIM15_PTR       ((stm32Dev_TIM *) (APB2_BASE + 0x4000))
#define TIM16_PTR       ((stm32Dev_TIM *) (APB2_BASE + 0x4400))
#define TIM17_PTR       ((stm32Dev_TIM *) (APB2_BASE + 0x4800))


/*******************************************
 * 47 I2C
 */
#define I2C_MODULE_COUNT                (4)

#include        "stm32I2c.h"

#define I2C1_PTR        ((stm32Dev_I2C *) (APB1_BASE + 0x5400))
#define I2C2_PTR        ((stm32Dev_I2C *) (APB1_BASE + 0x5800))
#define I2C3_PTR        ((stm32Dev_I2C *) (APB1_BASE + 0x5c00))
#define I2C4_PTR        ((stm32Dev_I2C *) (APB4_BASE + 0x1c00))


/*******************************************
 * 48 USART
 */
#define USART_MODULE_COUNT              (4)

#include        "stm32Usart.h"

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

#include        "stm32Spi32.h"

#define SPI1_PTR        ((stm32Dev_SPI *) ((APB2_BASE) + 0x3000))
#define SPI4_PTR        ((stm32Dev_SPI *) ((APB2_BASE) + 0x3400))
#define SPI5_PTR        ((stm32Dev_SPI *) ((APB2_BASE) + 0x5000))
#define SPI2_PTR        ((stm32Dev_SPI *) ((APB1_BASE) + 0x3800))
#define SPI3_PTR        ((stm32Dev_SPI *) ((APB1_BASE) + 0x3c00))
#define SPI6_PTR        ((stm32Dev_SPI *) ((APB4_BASE) + 0x1400))


/*******************************************
 * 57 USB
 */
#define USB_EPOUT_COUNTS        9     /* max number of ep  plus 1 */
#define USB_EPIN_COUNTS         9     /* max number of ep  plus 1 */
typedef struct {
  __IO uint32_t         CTL;
#define USB_EPCTL_EPENA_SHIFT   (31)
#define USB_EPCTL_EPENA_MASK    (1 << (USB_EPCTL_EPENA_SHIFT))
#define USB_EPCTL_EPENA_NO      (0 << (USB_EPCTL_EPENA_SHIFT))
#define USB_EPCTL_EPENA_YES     (1 << (USB_EPCTL_EPENA_SHIFT))
#define USB_EPCTL_EPDIS_SHIFT   (30)
#define USB_EPCTL_EPDIS_MASK    (1 << (USB_EPCTL_EPDIS_SHIFT))
#define USB_EPCTL_EPDIS_NO      (0 << (USB_EPCTL_EPDIS_SHIFT))
#define USB_EPCTL_EPDIS_YES     (1 << (USB_EPCTL_EPDIS_SHIFT))
#define USB_EPCTL_SD1PID_SODDFRM_SHIFT  (29)
#define USB_EPCTL_SD1PID_SODDFRM_MASK   (1 << (USB_EPCTL_SD1PID_SODDFRM_SHIFT))
#define USB_EPCTL_SD1PID_SODDFRM        (1 << (USB_EPCTL_SD1PID_SODDFRM_SHIFT))
#define USB_EPCTL_SD0PID_SEVNFRM_SHIFT  (28)
#define USB_EPCTL_SD0PID_SEVNFRM_MASK   (1 << (USB_EPCTL_SD0PID_SEVNFRM_SHIFT))
#define USB_EPCTL_SD0PID_SEVNFRM        (1 << (USB_EPCTL_SD0PID_SEVNFRM_SHIFT))
#define USB_EPCTL_SNAK_SHIFT    (27)
#define USB_EPCTL_SNAK_MASK     (1 << (USB_EPCTL_SNAK_SHIFT))
#define USB_EPCTL_SNAK          (1 << (USB_EPCTL_SNAK_SHIFT))
#define USB_EPCTL_CNAK_SHIFT    (26)
#define USB_EPCTL_CNAK_MASK     (1 << (USB_EPCTL_CNAK_SHIFT))
#define USB_EPCTL_CNAK          (1 << (USB_EPCTL_CNAK_SHIFT))
#define USB_EPCTL_TXFNUM_SHIFT  (22)
#define USB_EPCTL_TXFNUM_MASK   (0xf  << (USB_EPCTL_TXFNUM_SHIFT))
#define USB_EPCTL_TXFNUM_VAL(x) (((x) << (USB_EPCTL_TXFNUM_SHIFT)) & USB_EPCTL_TXFNUM_MASK)
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
#define USB_EPCTL_MPSIZ_SHIFT   (0)
#define USB_EPCTL_MPSIZ_MASK    (0x3ff << (USB_EPCTL_MPSIZ_SHIFT))
#define USB_EPCTL_MPSIZ_VAL(x)  ((x) << (USB_EPCTL_MPSIZ_SHIFT))

  uint32_t              reserved04;
  __IO uint32_t         INT;            /* 0x08 */
#define USB_EPINT_NAK_SHIFT     (13)
#define USB_EPINT_NAK_MASK      (1 << (USB_EPINT_NAK_SHIFT))
#define USB_EPINT_TXFE_SHIFT    (7)
#define USB_EPINT_TXFE_MASK     (1 << (USB_EPINT_TXFE_SHIFT))
#define USB_EPINT_B2BSTUP_SHIFT (6)
#define USB_EPINT_B2BSTUP_MASK  (1 << (USB_EPINT_B2BSTUP_SHIFT))
#define USB_EPINT_STSPHSRX_SHIFT (5)
#define USB_EPINT_STSPHSRX_MASK  (1 << (USB_EPINT_STSPHSRX_SHIFT))
#define USB_EPINT_OTEPDIS_SHIFT (4)
#define USB_EPINT_OTEPDIS_MASK  (1 << (USB_EPINT_OTEPDIS_SHIFT))
#define USB_EPINT_STUP_SHIFT    (3)
#define USB_EPINT_STUP_MASK     (1 << (USB_EPINT_STUP_SHIFT))
#define USB_EPINT_EPDISD_SHIFT  (1)
#define USB_EPINT_EPDISD_MASK   (1 << (USB_EPINT_EPDISD_SHIFT))
#define USB_EPINT_XFRC_SHIFT    (0)
#define USB_EPINT_XFRC_MASK     (1 << (USB_EPINT_XFRC_SHIFT))

  uint32_t              reserved0c;
  __IO uint32_t         SIZ;            /* 0x10 */
  /* STUPCNT is for SETUP OUT packet */
#define USB_EPSIZ_STUPCNT_SHIFT (29)
#define USB_EPSIZ_STUPCNT_MASK  (3 << (USB_EPSIZ_STUPCNT_SHIFT))
#define USB_EPSIZ_STUPCNT_1PKTS (1 << (USB_EPSIZ_STUPCNT_SHIFT))
#define USB_EPSIZ_STUPCNT_2PKTS (2 << (USB_EPSIZ_STUPCNT_SHIFT))
#define USB_EPSIZ_STUPCNT_3PKTS (3 << (USB_EPSIZ_STUPCNT_SHIFT))
  /* MCNT is for IN packet and EP1,2, ... */
#define USB_EPSIZ_MCNT_SHIFT    (29)
#define USB_EPSIZ_MCNT_MASK     (3 << (USB_EPSIZ_MCNT_SHIFT))
#define USB_EPSIZ_MCNT_1PKTS    (1 << (USB_EPSIZ_MCNT_SHIFT))
#define USB_EPSIZ_MCNT_2PKTS    (2 << (USB_EPSIZ_MCNT_SHIFT))
#define USB_EPSIZ_MCNT_3PKTS    (3 << (USB_EPSIZ_MCNT_SHIFT))
#define USB_EPSIZ_PKTCNT_SHIFT  (19)
#define USB_EPSIZ_PKTCNT_MASK   (1 << (USB_EPSIZ_PKTCNT_SHIFT))
#define USB_EPSIZ_PKTCNT_ZERO   (0 << (USB_EPSIZ_PKTCNT_SHIFT))
#define USB_EPSIZ_PKTCNT_ONE    (1 << (USB_EPSIZ_PKTCNT_SHIFT))
#define USB_EPSIZ_PKTCNT_TWO    (2 << (USB_EPSIZ_PKTCNT_SHIFT))
#define USB_EPSIZ_PKTCNT_THREE  (3 << (USB_EPSIZ_PKTCNT_SHIFT))
#define USB_EPSIZ_XFRSIZ_SHIFT  (0)
#define USB_EPSIZ_XFRSIZ_MASK   (0x7f << (USB_EPSIZ_XFRSIZ_SHIFT))
#define USB_EPSIZ_XFRSIZ_VAL(x) (((x) << (USB_EPSIZ_XFRSIZ_SHIFT)) & (USB_EPSIZ_XFRSIZ_MASK))
  __IO uint32_t         DMA;            /* 0x14  out only */
  __IO uint32_t         STS;            /* 0x18  in only */
  uint32_t              reserved1c;
} stm32Usb320aDevEp_t;

typedef struct {
  /* 0x00 */
  __IO uint32_t         GOTGCTL;
#define USB_GOTGCTL_BVALOVAL_SHIFT      (7)
#define USB_GOTGCTL_BVALOVAL_MASK       (1 << (USB_GOTGCTL_BVALOVAL_SHIFT))
#define USB_GOTGCTL_BVALOVAL_NOS        (0 << (USB_GOTGCTL_BVALOVAL_SHIFT))
#define USB_GOTGCTL_BVALOVAL_YES        (1 << (USB_GOTGCTL_BVALOVAL_SHIFT))
#define USB_GOTGCTL_BVALOEN_SHIFT       (6)
#define USB_GOTGCTL_BVALOEN_MASK        (1 << (USB_GOTGCTL_BVALOEN_SHIFT))
#define USB_GOTGCTL_BVALOEN_NO          (0 << (USB_GOTGCTL_BVALOEN_SHIFT))
#define USB_GOTGCTL_BVALOEN_YES         (1 << (USB_GOTGCTL_BVALOEN_SHIFT))

  __IO uint32_t         GOTGINT;
#define USB_GOTGINT_SEDET_SHIFT         (2)
#define USB_GOTGINT_SEDET_MASK          (1 << (USB_GOTGINT_SEDET_SHIFT))
#define USB_GOTGINT_SEDET_DIS           (0 << (USB_GOTGINT_SEDET_SHIFT))
#define USB_GOTGINT_SEDET_EN            (1 << (USB_GOTGINT_SEDET_SHIFT))


/*** 0x08 OTG_GAHBCFG     x is available to set 1 to 16 */
  __IO uint32_t         GAHBCFG;
#define USB_GAHBCFG_DMAEN_SHIFT         (4)
#define USB_GAHBCFG_DMAEN_MASK          (1 << (USB_GAHBCFG_DMAEN_SHIFT))
#define USB_GAHBCFG_DMAEN_NO            (0 << (USB_GAHBCFG_DMAEN_SHIFT))
#define USB_GAHBCFG_DMAEN_YES           (1 << (USB_GAHBCFG_DMAEN_SHIFT))
#define USB_GAHBCFG_HBSTLEN_SHIFT       (1)
#define USB_GAHBCFG_HBSTLEN_MASK        (15 << (USB_GAHBCFG_HBSTLEN_SHIFT))
#define USB_GAHBCFG_HBSTLEN_1WORD       (0 << (USB_GAHBCFG_HBSTLEN_SHIFT))
#define USB_GAHBCFG_HBSTLEN_4WORDS      (3 << (USB_GAHBCFG_HBSTLEN_SHIFT))
#define USB_GAHBCFG_HBSTLEN_8WORDS      (3 << (USB_GAHBCFG_HBSTLEN_SHIFT))
#define USB_GAHBCFG_HBSTLEN_16WORDS     (7 << (USB_GAHBCFG_HBSTLEN_SHIFT))
#define USB_GAHBCFG_GINTMSK_SHIFT       (0)
#define USB_GAHBCFG_GINTMSK_MASK        (1 << (USB_GAHBCFG_GINTMSK_SHIFT))
#define USB_GAHBCFG_GINTMSK_NO          (0 << (USB_GAHBCFG_GINTMSK_SHIFT))
#define USB_GAHBCFG_GINTMSK_YES         (1 << (USB_GAHBCFG_GINTMSK_SHIFT))

  __IO uint32_t         GUSBCFG;
  __IO uint32_t         GRSTCTL;
#define USB_GRSTCTL_TXFNUM_SHIFT        (6)
#define USB_GRSTCTL_TXFNUM_MASK         (0x1f << (USB_GRSTCTL_TXFNUM_SHIFT))
#define USB_GRSTCTL_TXFNUM(x)           (((x) << (USB_GRSTCTL_TXFNUM_SHIFT)) & (USB_GRSTCTL_TXFNUM_MASK))
#define USB_GRSTCTL_TXFNUM_ALL          (0x10 << (USB_GRSTCTL_TXFNUM_SHIFT))
#define USB_GRSTCTL_TXFFLSH_SHIFT       (5)
#define USB_GRSTCTL_TXFFLSH_MASK        (1 << (USB_GRSTCTL_TXFFLSH_SHIFT))
#define USB_GRSTCTL_TXFFLSH_DIS         (0 << (USB_GRSTCTL_TXFFLSH_SHIFT))
#define USB_GRSTCTL_TXFFLSH_EN          (1 << (USB_GRSTCTL_TXFFLSH_SHIFT))
#define USB_GRSTCTL_RXFFLSH_SHIFT       (4)
#define USB_GRSTCTL_RXFFLSH_MASK        (1 << (USB_GRSTCTL_RXFFLSH_SHIFT))
#define USB_GRSTCTL_RXFFLSH_DIS         (0 << (USB_GRSTCTL_RXFFLSH_SHIFT))
#define USB_GRSTCTL_RXFFLSH_EN          (1 << (USB_GRSTCTL_RXFFLSH_SHIFT))
#define USB_GRSTCTL_CSRST_SHIFT         (0)
#define USB_GRSTCTL_CSRST_MASK          (1 << (USB_GRSTCTL_CSRST_SHIFT))
#define USB_GRSTCTL_CSRST_DIS           (0 << (USB_GRSTCTL_CSRST_SHIFT))
#define USB_GRSTCTL_CSRST_EN            (1 << (USB_GRSTCTL_CSRST_SHIFT))

  __IO uint32_t         GINTSTS;
  __IO uint32_t         GINTMSK;
#define USB_GINTSTS_WKUINT_SHIFT        (31)
#define USB_GINTSTS_WKUINT_MASK         (1 << (USB_GINTSTS_WKUINT_SHIFT))
#define USB_GINTSTS_WKUINT_EN           (1 << (USB_GINTSTS_WKUINT_SHIFT))
#define USB_GINTSTS_SRQINT_SHIFT        (30)
#define USB_GINTSTS_SRQINT_MASK         (1 << (USB_GINTSTS_SRQINT_SHIFT))
#define USB_GINTSTS_SRQINT_EN           (1 << (USB_GINTSTS_SRQINT_SHIFT))
#define USB_GINTSTS_DISCINT_SHIFT       (29)
#define USB_GINTSTS_DISCINT_MASK        (1 << (USB_GINTSTS_DISCINT_SHIFT))
#define USB_GINTSTS_DISCINT_EN          (1 << (USB_GINTSTS_DISCINT_SHIFT))
#define USB_GINTSTS_CIDSCHG_SHIFT       (28)
#define USB_GINTSTS_CIDSCHG_MASK        (1 << (USB_GINTSTS_CIDSCHG_SHIFT))
#define USB_GINTSTS_CIDSCHG_EN          (1 << (USB_GINTSTS_CIDSCHG_SHIFT))
#define USB_GINTSTS_LPMINT_SHIFT        (27)
#define USB_GINTSTS_LPMINT_MASK         (1 << (USB_GINTSTS_LPMINT_SHIFT))
#define USB_GINTSTS_LPMINT_EN           (1 << (USB_GINTSTS_LPMINT_SHIFT))
#define USB_GINTSTS_PXFRM_IISOOXFR_SHIFT      (21)
#define USB_GINTSTS_PXFRM_IISOOXFR_MASK       (1 << (USB_GINTSTS_PXFRM_IISOOXFR_SHIFT))
#define USB_GINTSTS_PXFRM_IISOOXFR_EN         (1 << (USB_GINTSTS_PXFRM_IISOOXFR_SHIFT))
#define USB_GINTSTS_IISOIXFR_SHIFT      (20)
#define USB_GINTSTS_IISOIXFR_MASK       (1 << (USB_GINTSTS_IISOIXFR_SHIFT))
#define USB_GINTSTS_IISOIXFR_EN         (1 << (USB_GINTSTS_IISOIXFR_SHIFT))
#define USB_GINTSTS_OEPINT_SHIFT        (19)
#define USB_GINTSTS_OEPINT_MASK         (1 << (USB_GINTSTS_OEPINT_SHIFT))
#define USB_GINTSTS_OEPINT_EN           (1 << (USB_GINTSTS_OEPINT_SHIFT))
#define USB_GINTSTS_IEPINT_SHIFT        (18)
#define USB_GINTSTS_IEPINT_MASK         (1 << (USB_GINTSTS_IEPINT_SHIFT))
#define USB_GINTSTS_IEPINT_EN           (1 << (USB_GINTSTS_IEPINT_SHIFT))

#define USB_GINTSTS_ENUMDNE_SHIFT       (13)
#define USB_GINTSTS_ENUMDNE_MASK        (1 << (USB_GINTSTS_ENUMDNE_SHIFT))
#define USB_GINTSTS_ENUMDNE_EN          (1 << (USB_GINTSTS_ENUMDNE_SHIFT))
#define USB_GINTSTS_USBRST_SHIFT        (12)
#define USB_GINTSTS_USBRST_MASK         (1 << (USB_GINTSTS_USBRST_SHIFT))
#define USB_GINTSTS_USBRST_EN           (1 << (USB_GINTSTS_USBRST_SHIFT))
#define USB_GINTSTS_USBSUSP_SHIFT       (11)
#define USB_GINTSTS_USBSUSP_MASK        (1 << (USB_GINTSTS_USBSUSP_SHIFT))
#define USB_GINTSTS_USBSUSP_EN          (1 << (USB_GINTSTS_USBSUSP_SHIFT))
#define USB_GINTSTS_ESUSP_SHIFT         (10)
#define USB_GINTSTS_ESUSP_MASK          (1 << (USB_GINTSTS_ESUSP_SHIFT))
#define USB_GINTSTS_ESUSP_EN            (1 << (USB_GINTSTS_ESUSP_SHIFT))
#define USB_GINTSTS_RXFLVL_SHIFT        (4)
#define USB_GINTSTS_RXFLVL_MASK         (1 << (USB_GINTSTS_RXFLVL_SHIFT))
#define USB_GINTSTS_RXFLVL_EN           (1 << (USB_GINTSTS_RXFLVL_SHIFT))
#define USB_GINTSTS_SOF_SHIFT           (3)
#define USB_GINTSTS_SOF_MASK            (1 << (USB_GINTSTS_SOF_SHIFT))
#define USB_GINTSTS_SOF_EN              (1 << (USB_GINTSTS_SOF_SHIFT))
#define USB_GINTSTS_OTGINT_SHIFT        (2)
#define USB_GINTSTS_OTGINT_MASK         (1 << (USB_GINTSTS_OTGINT_SHIFT))
#define USB_GINTSTS_OTGINT_EN           (1 << (USB_GINTSTS_OTGINT_SHIFT))
#define USB_GINTSTS_MMIS_SHIFT          (1)
#define USB_GINTSTS_MMIS_MASK           (1 << (USB_GINTSTS_MMIS_SHIFT))
#define USB_GINTSTS_MMIS_EN             (1 << (USB_GINTSTS_MMIS_SHIFT))
#define USB_GINTSTS_CMOD_SHIFT          (0)
#define USB_GINTSTS_CMOD_MASK           (1 << (USB_GINTSTS_CMOD_SHIFT))

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
#define USB_DIEPTXF_FD_SHIFT            (16)
#define USB_DIEPTXF_FD_MASK             (0xffff << (USB_DIEPTXF_FD_SHIFT))
#define USB_DIEPTXF_SA_SHIFT            (0)
#define USB_DIEPTXF_SA_MASK             (0xffff << (USB_DIEPTXF_SA_SHIFT))
  __IO uint32_t         HNPTXSTS;
  __IO uint32_t         GI2CCTL;
  uint32_t              reserved_034;
  __IO uint32_t         GCCFG;
#define USB_GCCFG_VBDEN_SHIFT           (21)
#define USB_GCCFG_VBDEN_MASK            (3 << (USB_GCCFG_VBDEN_SHIFT))
#define USB_GCCFG_VBDEN_NO              (0 << (USB_GCCFG_VBDEN_SHIFT))
#define USB_GCCFG_VBDEN_YES             (1 << (USB_GCCFG_VBDEN_SHIFT))
#define USB_GCCFG_SDEN_SHIFT            (20)
#define USB_GCCFG_SDEN_MASK             (3 << (USB_GCCFG_SDEN_SHIFT))
#define USB_GCCFG_SDEN_NO               (0 << (USB_GCCFG_SDEN_SHIFT))
#define USB_GCCFG_SDEN_YES              (1 << (USB_GCCFG_SDEN_SHIFT))
#define USB_GCCFG_PDEN_SHIFT            (19)
#define USB_GCCFG_PDEN_MASK             (3 << (USB_GCCFG_PDEN_SHIFT))
#define USB_GCCFG_PDEN_NO               (0 << (USB_GCCFG_PDEN_SHIFT))
#define USB_GCCFG_PDEN_YES              (1 << (USB_GCCFG_PDEN_SHIFT))
#define USB_GCCFG_PWRDWN_SHIFT          (16)
#define USB_GCCFG_PWRDWN_MASK           (3 << (USB_GCCFG_PWRDWN_SHIFT))
#define USB_GCCFG_PWRDWN_NO             (0 << (USB_GCCFG_PWRDWN_SHIFT))
#define USB_GCCFG_PWRDWN_YES            (1 << (USB_GCCFG_PWRDWN_SHIFT))
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
#define USB_HPRT_PRST_SHIFT             (8)
#define USB_HPRT_PRST_MASK              (1 << (USB_HPRT_PRST_SHIFT))
#define USB_HPRT_PRST_NO                (0 << (USB_HPRT_PRST_SHIFT))
#define USB_HPRT_PRST_YES               (1 << (USB_HPRT_PRST_SHIFT))
#define USB_HPRT_POCCHNG_SHIFT          (5)
#define USB_HPRT_POCCHNG_MASK           (1 << (USB_HPRT_POCCHNG_SHIFT))
#define USB_HPRT_POCCHNG_NO             (0 << (USB_HPRT_POCCHNG_SHIFT))
#define USB_HPRT_POCCHNG_YES            (1 << (USB_HPRT_POCCHNG_SHIFT))
#define USB_HPRT_PENCHNG_SHIFT          (5)
#define USB_HPRT_PENCHNG_MASK           (1 << (USB_HPRT_PENCHNG_SHIFT))
#define USB_HPRT_PENCHNG_NO             (0 << (USB_HPRT_PENCHNG_SHIFT))
#define USB_HPRT_PENCHNG_YES            (1 << (USB_HPRT_PENCHNG_SHIFT))
#define USB_HPRT_PENA_SHIFT             (2)
#define USB_HPRT_PENA_MASK              (1 << (USB_HPRT_PENA_SHIFT))
#define USB_HPRT_PENA_NO                (0 << (USB_HPRT_PENA_SHIFT))
#define USB_HPRT_PENA_YES               (1 << (USB_HPRT_PENA_SHIFT))
#define USB_HPRT_PCDET_SHIFT            (1)
#define USB_HPRT_PCDET_MASK             (1 << (USB_HPRT_PCDET_SHIFT))
#define USB_HPRT_PCDET_NO               (0 << (USB_HPRT_PCDET_SHIFT))
#define USB_HPRT_PCDET_YES              (1 << (USB_HPRT_PCDET_SHIFT))
#define USB_HPRT_PCSTS_SHIFT            (0)
#define USB_HPRT_PCSTS_MASK             (1 << (USB_HPRT_PCSTS_SHIFT))
#define USB_HPRT_PCSTS_NO               (0 << (USB_HPRT_PCSTS_SHIFT))
#define USB_HPRT_PCSTS_YES              (1 << (USB_HPRT_PCSTS_SHIFT))
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
#define USB_DCTL_CGONAK_SHIFT           (10)
#define USB_DCTL_CGONAK_MASK            (1 << (USB_DCTL_CGONAK_SHIFT))
#define USB_DCTL_CGONAK_HIGH            (0 << (USB_DCTL_CGONAK_SHIFT))
#define USB_DCTL_CGONAK_FULL            (1 << (USB_DCTL_CGONAK_SHIFT))
#define USB_DCTL_SGONAK_SHIFT           (9)
#define USB_DCTL_SGONAK_MASK            (1 << (USB_DCTL_SGONAK_SHIFT))
#define USB_DCTL_SGONAK_HIGH            (0 << (USB_DCTL_SGONAK_SHIFT))
#define USB_DCTL_SGONAK_FULL            (1 << (USB_DCTL_SGONAK_SHIFT))
#define USB_DCTL_CGINAK_SHIFT           (8)
#define USB_DCTL_CGINAK_MASK            (1 << (USB_DCTL_CGINAK_SHIFT))
#define USB_DCTL_CGINAK_HIGH            (0 << (USB_DCTL_CGINAK_SHIFT))
#define USB_DCTL_CGINAK_FULL            (1 << (USB_DCTL_CGINAK_SHIFT))
#define USB_DCTL_SGINAK_SHIFT           (7)
#define USB_DCTL_SGINAK_MASK            (1 << (USB_DCTL_SGINAK_SHIFT))
#define USB_DCTL_SGINAK_HIGH            (0 << (USB_DCTL_SGINAK_SHIFT))
#define USB_DCTL_SGINAK_FULL            (1 << (USB_DCTL_SGINAK_SHIFT))
#define USB_DCTL_SDIS_SHIFT             (1)
#define USB_DCTL_SDIS_MASK              (1 << (USB_DCTL_SDIS_SHIFT))
#define USB_DCTL_SDIS_CONNECT           (0 << (USB_DCTL_SDIS_SHIFT))
#define USB_DCTL_SDIS_DISCONNECT        (1 << (USB_DCTL_SDIS_SHIFT))
#define USB_DCTL_RWUSIG_SHIFT           (0)
#define USB_DCTL_RWUSIG_MASK            (1 << (USB_DCTL_RWUSIG_SHIFT))
#define USB_DCTL_RWUSIG_CONNECT         (0 << (USB_DCTL_RWUSIG_SHIFT))
#define USB_DCTL_RWUSIG_DISCONNECT      (1 << (USB_DCTL_RWUSIG_SHIFT))

  __IO uint32_t         DSTS;
#define USB_DSTS_FNSOF_SHIFT            (8)
#define USB_DSTS_FNSOF_MASK             (0x3ff << (USB_DSTS_FNSOF_SHIFT))
#define USB_DSTS_FNSOF_LSB_MASK         (1     << (USB_DSTS_FNSOF_SHIFT))
#define USB_DSTS_EERR_SHIFT             (3)
#define USB_DSTS_EERR_MASK              (1 << (USB_DSTS_EERR_SHIFT))
#define USB_DSTS_EERR_NO                (0 << (USB_DSTS_EERR_SHIFT))
#define USB_DSTS_EERR_YES               (1 << (USB_DSTS_EERR_SHIFT))
#define USB_DSTS_ENUMSPD_SHIFT          (1)
#define USB_DSTS_ENUMSPD_MASK           (3 << (USB_DSTS_ENUMSPD_SHIFT))
#define USB_DSTS_ENUMSPD_HIGH           (0 << (USB_DSTS_ENUMSPD_SHIFT))
#define USB_DSTS_ENUMSPD_FULL           (3 << (USB_DSTS_ENUMSPD_SHIFT))
#define USB_DSTS_SUSPSTS_SHIFT          (0)
#define USB_DSTS_SUSPSTS_MASK           (1 << (USB_DSTS_SUSPSTS_SHIFT))
#define USB_DSTS_SUSPSTS_NO             (0 << (USB_DSTS_SUSPSTS_SHIFT))
#define USB_DSTS_SUSPSTS_YES            (1 << (USB_DSTS_SUSPSTS_SHIFT))

  uint32_t              Reserved_80c;
  __IO uint32_t         DIEPMSK;
#define USB_DIEPMSK_NAKM_SHIFT          (12)
#define USB_DIEPMSK_NAKM_MASK           (1 << (USB_DIEPMSK_NAKM_SHIFT))
#define USB_DIEPMSK_NAKM_EN             (1 << (USB_DIEPMSK_NAKM_SHIFT))
#define USB_DIEPMSK_INEPNEM_SHIFT       (6)
#define USB_DIEPMSK_INEPNEM_MASK        (1 << (USB_DIEPMSK_INEPNEM_SHIFT))
#define USB_DIEPMSK_INEPNEM_EN          (1 << (USB_DIEPMSK_INEPNEM_SHIFT))
#define USB_DIEPMSK_INEPNMM_SHIFT       (5)
#define USB_DIEPMSK_INEPNMM_MASK        (1 << (USB_DIEPMSK_INEPNMM_SHIFT))
#define USB_DIEPMSK_INEPNMM_EN          (1 << (USB_DIEPMSK_INEPNMM_SHIFT))
#define USB_DIEPMSK_ITTXFEMSK_SHIFT     (4)
#define USB_DIEPMSK_ITTXFEMSK_MASK      (1 << (USB_DIEPMSK_ITTXFEMSK_SHIFT))
#define USB_DIEPMSK_ITTXFEMSK_EN        (1 << (USB_DIEPMSK_ITTXFEMSK_SHIFT))
#define USB_DIEPMSK_TOM_SHIFT           (3)
#define USB_DIEPMSK_TOM_MASK            (1 << (USB_DIEPMSK_TOM_SHIFT))
#define USB_DIEPMSK_TOM_EN              (1 << (USB_DIEPMSK_TOM_SHIFT))
#define USB_DIEPMSK_EPDM_SHIFT          (1)
#define USB_DIEPMSK_EPDM_MASK           (1 << (USB_DIEPMSK_EPDM_SHIFT))
#define USB_DIEPMSK_EPDM_EN             (1 << (USB_DIEPMSK_EPDM_SHIFT))
#define USB_DIEPMSK_XFRCM_SHIFT         (0)
#define USB_DIEPMSK_XFRCM_MASK          (1 << (USB_DIEPMSK_XFRCM_SHIFT))
#define USB_DIEPMSK_XFRCM_EN            (1 << (USB_DIEPMSK_XFRCM_SHIFT))
  __IO uint32_t         DOEPMSK;
#define USB_DOEPMSK_NYETM_SHIFT         (14)
#define USB_DOEPMSK_NYETM_MASK          (1 << (USB_DOEPMSK_NYETM_SHIFT))
#define USB_DOEPMSK_NYETM_EN            (1 << (USB_DOEPMSK_NYETM_SHIFT))
#define USB_DOEPMSK_BOIM_SHIFT          (9)
#define USB_DOEPMSK_BOIM_MASK           (1 << (USB_DOEPMSK_BOIM_SHIFT))
#define USB_DOEPMSK_BOIM_EN             (1 << (USB_DOEPMSK_BOIM_SHIFT))
#define USB_DOEPMSK_TXFURM_SHIFT        (8)
#define USB_DOEPMSK_TXFURM_MASK         (1 << (USB_DOEPMSK_TXFURM_SHIFT))
#define USB_DOEPMSK_TXFURM_EN           (1 << (USB_DOEPMSK_TXFURM_SHIFT))
#define USB_DOEPMSK_B2BSTUPM_SHIFT      (6)
#define USB_DOEPMSK_B2BSTUPM_MASK       (1 << (USB_DOEPMSK_B2BSTUPM_SHIFT))
#define USB_DOEPMSK_B2BSTUPM_EN         (1 << (USB_DOEPMSK_B2BSTUPM_SHIFT))
#define USB_DOEPMSK_ETEPDM_SHIFT        (4)
#define USB_DOEPMSK_ETEPDM_MASK         (1 << (USB_DOEPMSK_ETEPDM_SHIFT))
#define USB_DOEPMSK_ETEPDM_EN           (1 << (USB_DOEPMSK_ETEPDM_SHIFT))
#define USB_DOEPMSK_STUPM_SHIFT         (3)
#define USB_DOEPMSK_STUPM_MASK          (1 << (USB_DOEPMSK_STUPM_SHIFT))
#define USB_DOEPMSK_STUPM_EN            (1 << (USB_DOEPMSK_STUPM_SHIFT))
#define USB_DOEPMSK_EPDM_SHIFT          (1)
#define USB_DOEPMSK_EPDM_MASK           (1 << (USB_DOEPMSK_EPDM_SHIFT))
#define USB_DOEPMSK_EPDM_EN             (1 << (USB_DOEPMSK_EPDM_SHIFT))
#define USB_DOEPMSK_XFRCM_SHIFT         (0)
#define USB_DOEPMSK_XFRCM_MASK          (1 << (USB_DOEPMSK_XFRCM_SHIFT))
#define USB_DOEPMSK_XFRCM_EN            (1 << (USB_DOEPMSK_XFRCM_SHIFT))

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
#define USB_DTHRCTL_RXTHRLEN_SHIFT      (17)
#define USB_DTHRCTL_RXTHRLEN_MASK       (0x3ff << (USB_DTHRCTL_RXTHRLEN_SHIFT))
#define USB_DTHRCTL_RXTHRLEN_VAL(x)     (((x) << (USB_DTHRCTL_RXTHRLEN_SHIFT))&USB_DTHRCTL_RXTHRLEN_MASK)
#define USB_DTHRCTL_RXTHREN_SHIFT       (16)
#define USB_DTHRCTL_RXTHREN_MASK        (1 << (USB_DTHRCTL_RXTHREN_SHIFT))
#define USB_DTHRCTL_RXTHREN_NO          (0 << (USB_DTHRCTL_RXTHREN_SHIFT))
#define USB_DTHRCTL_RXTHREN_YES         (1 << (USB_DTHRCTL_RXTHREN_SHIFT))
#define USB_DTHRCTL_TXTHRLEN_SHIFT      (2)
#define USB_DTHRCTL_TXTHRLEN_MASK       (0x3ff << (USB_DTHRCTL_TXTHRLEN_SHIFT))
#define USB_DTHRCTL_TXTHRLEN_VAL(x)     (((x) << (USB_DTHRCTL_TXTHRLEN_SHIFT))&USB_DTHRCTL_TXTHRLEN_MASK)
#define USB_DTHRCTL_ISOTHREN_SHIFT      (1)
#define USB_DTHRCTL_ISOTHREN_MASK       (1 << (USB_DTHRCTL_ISOTHREN_SHIFT))
#define USB_DTHRCTL_ISOTHREN_NO         (0 << (USB_DTHRCTL_ISOTHREN_SHIFT))
#define USB_DTHRCTL_ISOTHREN_YES        (1 << (USB_DTHRCTL_ISOTHREN_SHIFT))
#define USB_DTHRCTL_NONISOTHREN_SHIFT   (0)
#define USB_DTHRCTL_NONISOTHREN_MASK    (1 << (USB_DTHRCTL_NONISOTHREN_SHIFT))
#define USB_DTHRCTL_NONISOTHREN_NO      (0 << (USB_DTHRCTL_NONISOTHREN_SHIFT))
#define USB_DTHRCTL_NONISOTHREN_YES     (1 << (USB_DTHRCTL_NONISOTHREN_SHIFT))

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
#define USB_PCGCCTL_STOPCLK_SHIFT       (0)
#define USB_PCGCCTL_STOPCLK_MASK        (1 << (USB_PCGCCTL_STOPCLK_SHIFT))
#define USB_PCGCCTL_STOPCLK_NO          (0 << (USB_PCGCCTL_STOPCLK_SHIFT))
#define USB_PCGCCTL_STOPCLK_YES         (1 << (USB_PCGCCTL_STOPCLK_SHIFT))
  uint32_t              reserved_e04[0x7f];

  /* 0x1000  FIFO */
#define USB_FIFO_SIZE   0x1000
  __IO uint32_t         DFIFO[15][USB_FIFO_SIZE/4];

} stm32Dev_USB;


#define USB1_HS                 ((stm32Dev_USB *) (BUS_PERIPHERAL+0x40000))
#define USB2_FS                 ((stm32Dev_USB *) (BUS_PERIPHERAL+0x80000))


#define USB_FS_MAX_PACKET_SIZE          64
#define USB_HS_MAX_PACKET_SIZE          512
#define USB_SPEED_SUPER                 3
#define USB_SPEED_HIGH                  2
#define USB_SPEED_LOW                   1
#define USB_SPEED_FULL                  0

#define USB_MODULE_TBL                  {NULL, USB1_OTG_HS, USB2_OTG_FS}


#define USB_OTG_CORE_ID_310A            0x4F54310A
#define USB_OTG_CORE_ID_320A            0x4F54320A



/*** 0x0c OTG_GUSBCFG */
#define USB_GUSBCFG_FDMOD_SHIFT         (30)
#define USB_GUSBCFG_FDMOD_MASK          (1 << (USB_GUSBCFG_FDMOD_SHIFT))
#define USB_GUSBCFG_FDMOD_NO            (0 << (USB_GUSBCFG_FDMOD_SHIFT))
#define USB_GUSBCFG_FDMOD_YES           (1 << (USB_GUSBCFG_FDMOD_SHIFT))
#define USB_GUSBCFG_FHMOD_SHIFT         (29)
#define USB_GUSBCFG_FHMOD_MASK          (1 << (USB_GUSBCFG_FHMOD_SHIFT))
#define USB_GUSBCFG_FHMOD_NO            (0 << (USB_GUSBCFG_FHMOD_SHIFT))
#define USB_GUSBCFG_FHMOD_YES           (1 << (USB_GUSBCFG_FHMOD_SHIFT))
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
#define USB_GUSBCFG_PHYSEL_SHIFT        (6)
#define USB_GUSBCFG_PHYSEL_MASK         (1 << (USB_GUSBCFG_PHYSEL_SHIFT))
#define USB_GUSBCFG_PHYSEL_NO           (0 << (USB_GUSBCFG_PHYSEL_SHIFT))
#define USB_GUSBCFG_PHYSEL_YES          (1 << (USB_GUSBCFG_PHYSEL_SHIFT))


#endif
