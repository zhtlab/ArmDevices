#ifndef	_STM32L4_H_
#define _STM32L4_H_

#define __CM4_REV               1
#define __MPU_PRESENT           1
#define __FPU_PRESENT           1
#define __NVIC_PRIO_BITS        4
#define __Vendor_SysTickConfig  0



#define AHB1_BASE               (0x40020000)
#define AHB2_BASE               (0x48000000)
#define APB1_BASE               (0x40000000)
#define APB2_BASE               (0x40010000)



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
  TIM1_BRK_TIM15_IRQn   =    24,
  TIM1_UP_TIM15_IRQn,
  TIM1_TRG_COM_IRQn,
  TIM1_CC_IRQn,
  TIM2_IRQn               =  28,
  TIM3_IRQn,
  USART1_IRQn             =  37,
  USART2_IRQn,
  USART3_IRQn,
  
};

typedef int     IRQn_Type;
#ifndef __IO
#define __IO    volatile
#endif

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

/*************************************************************
 * USART
 */
#include        "../../STM/include/stm32Usart.h"

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


struct _stStm32l4_I2C {
  __IO uint32_t	cr1;		/* 0x00 */
#define CR1_PECEN_SHIFT		23
#define CR1_PECEN_MASK		(1 << (CR1_PECEN_SHIFT))
#define CR1_PECEN_NO		(0 << (CR1_PECEN_SHIFT))
#define CR1_PECEN_YES		(1 << (CR1_PECEN_SHIFT))
#define CR1_ALERTEN_SHIFT	22	
#define CR1_ALERTEN_MASK	(1 << (CR1_ALERTEN_SHIFT))
#define CR1_ALERTEN_NO		(0 << (CR1_ALERTEN_SHIFT))
#define CR1_ALERTEN_YES		(1 << (CR1_ALERTEN_SHIFT))
#define CR1_SMBDEN_SHIFT	21	
#define CR1_SMBDEN_MASK		(1 << (CR1_SMBDEN_SHIFT))
#define CR1_SMBDEN_NO		(0 << (CR1_SMBDEN_SHIFT))
#define CR1_SMBDEN_YES		(1 << (CR1_SMBDEN_SHIFT))
#define CR1_SMBHEN_SHIFT	20	
#define CR1_SMBHEN_MASK		(1 << (CR1_SMBHEN_SHIFT))
#define CR1_SMBHEN_NO		(0 << (CR1_SMBHEN_SHIFT))
#define CR1_SMBHEN_YES		(1 << (CR1_SMBHEN_SHIFT))
#define CR1_GCEN_SHIFT		19
#define CR1_GCEN_MASK		(1 << (CR1_GCEN_SHIFT))
#define CR1_GCEN_NO		(0 << (CR1_GCEN_SHIFT))
#define CR1_GCEN_YES		(1 << (CR1_GCEN_SHIFT))
#define CR1_WUPEN_SHIFT		18
#define CR1_WUPEN_MASK		(1 << (CR1_WUPEN_SHIFT))
#define CR1_WUPEN_NO		(0 << (CR1_WUPEN_SHIFT))
#define CR1_WUPEN_YES		(1 << (CR1_WUPEN_SHIFT))
#define CR1_NOSTRETCH_SHIFT	17	
#define CR1_NOSTRETCH_MASK	(1 << (CR1_NOSTRETCH_SHIFT))
#define CR1_NOSTRETCH_NO	(0 << (CR1_NOSTRETCH_SHIFT))
#define CR1_NOSTRETCH_YES	(1 << (CR1_NOSTRETCH_SHIFT))
#define CR1_SBC_SHIFT		16
#define CR1_SBC_MASK		(1 << (CR1_SBC_SHIFT))
#define CR1_SBC_NO		(0 << (CR1_SBC_SHIFT))
#define CR1_SBC_YES		(1 << (CR1_SBC_SHIFT))
#define CR1_RXDDMAEN_SHIFT	15	
#define CR1_RXDDMAEN_MASK      	(1 << (CR1_RXDDMAEN_SHIFT))
#define CR1_RXDDMAEN_NO		(0 << (CR1_RXDDMAEN_SHIFT))
#define CR1_RXDDMAEN_YES       	(1 << (CR1_RXDDMAEN_SHIFT))
#define CR1_TXDMAEN_SHIFT	14	
#define CR1_TXDMAEN_MASK	(1 << (CR1_TXDMAEN_SHIFT))
#define CR1_TXDMAEN_NO		(0 << (CR1_TXDMAEN_SHIFT))
#define CR1_TXDMAEN_YES		(1 << (CR1_TXDMAEN_SHIFT))
#define CR1_ANFOFF_SHIFT	12	
#define CR1_ANFOFF_MASK		(1 << (CR1_ANFOFF_SHIFT))
#define CR1_ANFOFF_NO		(0 << (CR1_ANFOFF_SHIFT))
#define CR1_ANFOFF_YES		(1 << (CR1_ANFOFF_SHIFT))
#define CR1_DNF_SHIFT		8
#define CR1_DNF_MASK		(15 << (CR1_DNF_SHIFT))
#define CR1_DNF_OFF		(0 << (CR1_DNF_SHIFT))
#define CR1_DNF_CLK1		(1 << (CR1_DNF_SHIFT))
#define CR1_DNF_CLK2		(2 << (CR1_DNF_SHIFT))
#define CR1_DNF_CLK15		(15 << (CR1_DNF_SHIFT))
#define CR1_ERRIE_SHIFT		7
#define CR1_ERRIE_MASK		(1 << (CR1_ERRIE_SHIFT))
#define CR1_ERRIE_NO		(0 << (CR1_ERRIE_SHIFT))
#define CR1_ERRIE_YES		(1 << (CR1_ERRIE_SHIFT))
#define CR1_TCIE_SHIFT		6
#define CR1_TCIE_MASK		(1 << (CR1_TCIE_SHIFT))
#define CR1_TCIE_NO		(0 << (CR1_TCIE_SHIFT))
#define CR1_TCIE_YES		(1 << (CR1_TCIE_SHIFT))
#define CR1_STOPIE_SHIFT	5	
#define CR1_STOPIE_MASK		(1 << (CR1_STOPIE_SHIFT))
#define CR1_STOPIE_NO		(0 << (CR1_STOPIE_SHIFT))
#define CR1_STOPIE_YES		(1 << (CR1_STOPIE_SHIFT))
#define CR1_NACKIE_SHIFT	4	
#define CR1_NACKIE_MASK		(1 << (CR1_NACKIE_SHIFT))
#define CR1_NACKIE_NO		(0 << (CR1_NACKIE_SHIFT))
#define CR1_NACKIE_YES		(1 << (CR1_NACKIE_SHIFT))
#define CR1_ADDRIE_SHIFT	3	
#define CR1_ADDRIE_MASK		(1 << (CR1_ADDRIE_SHIFT))
#define CR1_ADDRIE_NO		(0 << (CR1_ADDRIE_SHIFT))
#define CR1_ADDRIE_YES		(1 << (CR1_ADDRIE_SHIFT))
#define CR1_RXIE_SHIFT		2
#define CR1_RXIE_MASK		(1 << (CR1_RXIE_SHIFT))
#define CR1_RXIE_NO		(0 << (CR1_RXIE_SHIFT))
#define CR1_RXIE_YES		(1 << (CR1_RXIE_SHIFT))
#define CR1_TXIE_SHIFT		1
#define CR1_TXIE_MASK		(1 << (CR1_TXIE_SHIFT))
#define CR1_TXIE_NO		(0 << (CR1_TXIE_SHIFT))
#define CR1_TXIE_YES		(1 << (CR1_TXIE_SHIFT))
#define CR1_PE_SHIFT		0
#define CR1_PE_MASK		(1 << (CR1_PE_SHIFT))
#define CR1_PE_NO		(0 << (CR1_PE_SHIFT))
#define CR1_PE_YES		(1 << (CR1_PE_SHIFT))
  
  __IO uint32_t	cr2;		/* 0x04 */
#define CR2_PECBYTE_SHIFT	26	
#define CR2_PECBYTE_MASK	(1 << (CR2_PECBYTE_SHIFT))
#define CR2_PECBYTE_NO		(0 << (CR2_PECBYTE_SHIFT))
#define CR2_PECBYTE_YES		(1 << (CR2_PECBYTE_SHIFT))
#define CR2_AUTOEND_SHIFT	25	
#define CR2_AUTOEND_MASK	(1 << (CR2_AUTOEND_SHIFT))
#define CR2_AUTOEND_NO		(0 << (CR2_AUTOEND_SHIFT))
#define CR2_AUTOEND_YES		(1 << (CR2_AUTOEND_SHIFT))
#define CR2_RELOAD_SHIFT	24	
#define CR2_RELOAD_MASK		(1 << (CR2_RELOAD_SHIFT))
#define CR2_RELOAD_NO		(0 << (CR2_RELOAD_SHIFT))
#define CR2_RELOAD_YES		(1 << (CR2_RELOAD_SHIFT))
#define CR2_NBYTES_SHIFT	16	
#define CR2_NBYTES_MASK		(0xff << (CR2_NBYTES_SHIFT))
#define CR2_NACK_SHIFT		15
#define CR2_NACK_MASK		(1 << (CR2_NACK_SHIFT))
#define CR2_NACK_NO		(0 << (CR2_NACK_SHIFT))
#define CR2_NACK_YES		(1 << (CR2_NACK_SHIFT))
#define CR2_STOP_SHIFT		14
#define CR2_STOP_MASK		(1 << (CR2_STOP_SHIFT))
#define CR2_STOP_NO		(0 << (CR2_STOP_SHIFT))
#define CR2_STOP_YES		(1 << (CR2_STOP_SHIFT))
#define CR2_START_SHIFT		13
#define CR2_START_MASK		(1 << (CR2_START_SHIFT))
#define CR2_START_NO		(0 << (CR2_START_SHIFT))
#define CR2_START_YES		(1 << (CR2_START_SHIFT))
#define CR2_HEAD10R_SHIFT	12	
#define CR2_HEAD10R_MASK	(1 << (CR2_HEAD10R_SHIFT))
#define CR2_HEAD10R_NO		(0 << (CR2_HEAD10R_SHIFT))
#define CR2_HEAD10R_YES		(1 << (CR2_HEAD10R_SHIFT))
#define CR2_ADDR10_SHIFT	11	
#define CR2_ADDR10_MASK		(1 << (CR2_ADDR10_SHIFT))
#define CR2_ADDR10_NO		(0 << (CR2_ADDR10_SHIFT))
#define CR2_ADDR10_YES		(1 << (CR2_ADDR10_SHIFT))
#define CR2_RD_WDN_SHIFT	10	
#define CR2_RD_WDN_MASK		(1 << (CR2_RD_WDN_SHIFT))
#define CR2_RD_WDN_NO		(0 << (CR2_RD_WDN_SHIFT))
#define CR2_RD_WDN_YES		(1 << (CR2_RD_WDN_SHIFT))
#define CR2_SADD_SHIFT		0
#define CR2_SADD_MASK		(0x3ff << (CR2_SADD_SHIFT))

  __IO uint32_t	oar1;		/* 0x08 */
#define OAR1_OA1EN_SHIFT	15
#define OAR1_OA1EN_MASK		(1 << (OAR1_OA1EN_SHIFT))
#define OAR1_OA1EN_NO		(0 << (OAR1_OA1EN_SHIFT))
#define OAR1_OA1EN_YES		(1 << (OAR1_OA1EN_SHIFT))
#define OAR1_OA1MODE_SHIFT	10
#define OAR1_OA1MODE_MASK	(1 << (OAR1_OA1MODE_SHIFT))
#define OAR1_OA1MODE_NO		(0 << (OAR1_OA1MODE_SHIFT))
#define OAR1_OA1MODE_YES	(1 << (OAR1_OA1MODE_SHIFT))
#define OAR1_OA1_7BIT_SHIFT	1
#define OAR1_OA1_7BIT_MASK	(1 << (OAR1_OA1_7BIT_SHIFT))
#define OAR1_OA1_10BIT_SHIFT	0
#define OAR1_OA1_10BIT_MASK	(1 << (OAR1_OA1_10BIT_SHIFT))
  
  __IO uint32_t	oar2;		/* 0x0c */
  __IO uint32_t	timingr;	/* 0x10 */
  __IO uint32_t	timeoutr;	/* 0x14 */
  __IO uint32_t	isr;		/* 0x18 */
  /* use IISR,_XX, because ISR_XX word is already used in USART */
#define IISR_ADDCODE_SHIFT	17	
#define IISR_ADDCODE_MASK	(0x7f << (IISR_ADDCODE_SHIFT))
#define IISR_DIR_SHIFT		16
#define IISR_DIR_MASK		(1 << (IISR_DIR_SHIFT))
#define IISR_BUSY_SHIFT		15
#define IISR_BUSY_MASK		(1 << (IISR_BUSY_SHIFT))
#define IISR_ALERT_SHIFT	13
#define IISR_ALERT_MASK		(1 << (IISR_ALERT_SHIFT))
#define IISR_TIMEOUT_SHIFT	12	
#define IISR_TIMEOUT_MASK	(1 << (IISR_TIMEOUT_SHIFT))
#define IISR_PECERR_SHIFT	11	
#define IISR_PECERR_MASK	(1 << (IISR_PECERR_SHIFT))
#define IISR_OVR_SHIFT		10
#define IISR_OVR_MASK		(1 << (IISR_OVR_SHIFT))
#define IISR_ARLO_SHIFT		9
#define IISR_ARLO_MASK		(1 << (IISR_ARLO_SHIFT))
#define IISR_PE_SHIFT		8
#define IISR_BERR_MASK		(1 << (IISR_BERR_SHIFT))
#define IISR_TCR_SHIFT		7
#define IISR_TCR_MASK		(1 << (IISR_TCR_SHIFT))
#define IISR_TC_SHIFT		6
#define IISR_TC_MASK		(1 << (IISR_TC_SHIFT))
#define IISR_STOPF_SHIFT	5
#define IISR_STOPF_MASK		(1 << (IISR_STOPF_SHIFT))
#define IISR_NACKF_SHIFT	4
#define IISR_NACKF_MASK		(1 << (IISR_NACKF_SHIFT))
#define IISR_ADDR_SHIFT		3
#define IISR_ADDR_MASK		(1 << (IISR_ADDR_SHIFT))
#define IISR_RXNE_SHIFT		2
#define IISR_RXNE_MASK		(1 << (IISR_RXNE_SHIFT))
#define IISR_RXIS_SHIFT		1
#define IISR_RXIS_MASK		(1 << (IISR_RXIS_SHIFT))
#define IISR_RXIS_FLUSH		(1 << (IISR_RXIS_SHIFT))
#define IISR_TXE_SHIFT		0
#define IISR_TXE_MASK		(1 << (IISR_TXE_SHIFT))
#define IISR_TXE_FLUSH		(1 << (IISR_TXE_SHIFT))

  __IO uint32_t	icr;		/* 0x1c */
#define ICR_ALERTCF_SHIFT	13	
#define ICR_ALERTCF_CLEAR	(1 << (ICR_ALERTCF_SHIFT))
#define ICR_TIMEOUTCF_SHIFT	12	
#define ICR_TIMEOUTCF_CLEAR	(1 << (ICR_TIMEOUTCF_SHIFT))
#define ICR_PECCF_SHIFT		11
#define ICR_PECCF_CLEAR		(1 << (ICR_PECCF_SHIFT))
#define ICR_OVRCF_SHIFT		10
#define ICR_OVRCF_CLEAR		(1 << (ICR_OVRCF_SHIFT))
#define ICR_ARLOCKF_SHIFT	9	
#define ICR_ARLOCKF_CLEAR	(1 << (ICR_ARLOCKF_SHIFT))
#define ICR_BERRCF_SHIFT	8	
#define ICR_BERRCF_CLEAR	(1 << (ICR_BERRCF_SHIFT))
#define ICR_STOPCF_SHIFT	5	
#define ICR_STOPCF_CLEAR	(1 << (ICR_STOPCF_SHIFT))
#define ICR_NACKCF_SHIFT	4	
#define ICR_NACKCF_CLEAR	(1 << (ICR_NACKCF_SHIFT))
#define ICR_ADDRCF_SHIFT	3
#define ICR_ADDRCF_CLEAR	(1 << (ICR_ADDRCF_SHIFT))

  __IO uint32_t	pecr;		/* 0x20 */
  __IO uint32_t	rxdr;		/* 0x24 */
  __IO uint32_t	txdr;		/* 0x28 */
};
#define I2C_PTR		((struct _stStm32l4_I2C *) (APB1_BASE + 0x5400))
#define I2C1_PTR	((struct _stStm32l4_I2C *) (APB1_BASE + 0x5400))
#define I2C2_PTR	((struct _stStm32l4_I2C *) (APB1_BASE + 0x5800))
#define I2C3_PTR	((struct _stStm32l4_I2C *) (APB1_BASE + 0x5c00))

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
  __IO uint32_t	dhr12r1;			/* 0x08 */
  __IO uint32_t	dhr12l1;			/* 0x0c */
  __IO uint32_t	dhr8r1;			/* 0x10 */
  __IO uint32_t	dhr12r2;			/* 0x14 */
  __IO uint32_t	dhr12l2;			/* 0x18 */
  __IO uint32_t	dhr8r2;			/* 0x1c */
  __IO uint32_t	dhr12rd;			/* 0x20 */
  __IO uint32_t	dhr12ld;			/* 0x24 */
  __IO uint32_t	dhr8rd;			/* 0x28 */
  __IO uint32_t	dor1;			/* 0x2c */
  __IO uint32_t	dor2;			/* 0x30 */
  __IO uint32_t	sr;				/* 0x34 */
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


#endif
