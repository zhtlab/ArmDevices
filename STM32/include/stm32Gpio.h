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

#ifndef _STM32Gpio_H_
#define _STM32Gpio_H_


typedef struct {
  __IO uint32_t		MODER;
#define MODE_INPUT(x)		(0 << ((x)*2))
#define MODE_OUTPUT(x)		(1 << ((x)*2))
#define MODE_FUNC(x)		(2 << ((x)*2))
#define MODE_ANALOG(x)		(3 << ((x)*2))
#define MODE_MASK(x)		(3 << ((x)*2))
#define MODE_SET(x, v)		((v) << ((x)*2))
  __IO uint32_t		OTYPER;
#define OTYPE_PUSHPULL(x)	(0 << (x))
#define OTYPE_OD(x)		(1 << (x))
#define OTYPE_MASK(x)		(1 << (x))
  __IO uint32_t		OSPEEDR;
#define OSPEED_LOW(x)		(0 << ((x)*2))
#define OSPEED_MED(x)		(1 << ((x)*2))
#define OSPEED_HIGH(x)		(2 << ((x)*2))
#define OSPEED_EXHIGH(x)	(3 << ((x)*2))
#define OSPEED_MASK(x)		(3 << ((x)*2))
  __IO uint32_t		PUPDR;
#define PUPD_NONE(x)		(0 << ((x)*2))
#define PUPD_PU(x)		(1 << ((x)*2))
#define PUPD_PD(x)		(2 << ((x)*2))
#define PUPD_RESERVED(x)	(3 << ((x)*2))
#define PUPD_X(x, d)		((d) << ((x)*2))
#define PUPD_MASK(x)		(3 << ((x)*2))
  __IO uint32_t		IDR;
#define IDR_MASK(x)		(1 << (x))
  __IO uint32_t		ODR;
#define ODR_0(x)		( 0  << (x))
#define ODR_1(x)		( 1  << (x))
#define ODR_X(x, d)		((d) << (x))
#define ODR_MASK(x)		( 1  << (x))
#ifdef  STM32GPIO_BRR_28H_EXIST
  __IO uint32_t		BSRR;
#else
  __IO uint16_t		BSRR;
  __IO uint16_t		BRR;
#endif
#define BSRR_SET(x)		( 1  << (x))
#define BRR_RESET(x)		( 1  << (x))
  __IO uint32_t		LCKR;
#define LCKR_LCKK_SHIFT		(16)
#define LCKR_LCKK_0		(0 << (LCKR_LCKK_SHIFT))
#define LCKR_LCKK_1		(1 << (LCKR_LCKK_SHIFT))
#define LCKR_LCKK_MASK		(1 << (LCKR_LCKK_SHIFT))
  __IO uint32_t		AFRL;
#define AFRL_X(x, d)		(((d)&15) << (((x) & 7)*4))
#define AFRL_MASK(x)		((0xf)    << (((x) & 7)*4))
#define AFL_0		0
#define AFL_1		1
#define AFL_2		2
#define AFL_3		3
#define AFL_4		4
#define AFL_5		5
#define AFL_6		6
#define AFL_7		7

#define FUNC0_SYS	0
#define FUNC1_TIM	1
#define FUNC2_TIM	2
#define FUNC3_TIM	3
#define FUNC3_USART	3
#define FUNC4_I2C	4
#define FUNC5_SPI	5
#define FUNC6_SPI	6
#define FUNC7_USART	7
#define FUNC8_LPUART	8
#define FUNC9_CAN	9
#define FUNC9_FMC	9
#define FUNC10_USB	10
#define FUNC11_LCD	11
#define FUNC12_SD	12
#define FUNC12_FMC	12
#define FUNC12_COMP	12
#define FUNC13_SAI	13
#define FUNC14_TIM	14
#define FUNC15		15

  __IO uint32_t		AFRH;
#define AFRH_X(x, d)		(((d)&15) << (((x) & 7)*4))
#define AFRH_MASK(x)		((0xf)    << (((x) & 7)*4))
#define AFH_8		0
#define AFH_9		1
#define AFH_10		2
#define AFH_11		3
#define AFH_12		4
#define AFH_13		5
#define AFH_14		6
#define AFH_15		7

#ifdef  STM32GPIO_BRR_28H_EXIST
  __IO uint32_t		BRR;
  __IO uint32_t		gpio_reserved[0xf5];
#else
  __IO uint32_t		gpio_reserved[0xf6];
#endif
  
} stm32Dev_GPIO;



/**************************************************************
 * Port Function Register
 * [15:14] mode  in/out/func/analog
 * [13:12] pull none/up/down
 * [11:9]  reserved (if mode is in/out)
 * [8]     val      (if mode is in/out)
 * [11:8]  func     (if mode is func)
 * [7:3]   port (A -- J)
 * [2:0]   num (0 -- 7)
 */

#define	GPIO_NUM_SHIFT		(0)
#define	GPIO_NUM_MASK		(0xf << (GPIO_NUM_SHIFT))
#define	GPIO_PORT_SHIFT		(4)
#define	GPIO_PORT_MASK		(0xf << (GPIO_PORT_SHIFT))
#define	GPIO_PORTNUM_SHIFT	(0)
#define	GPIO_PORTNUM_MASK	( (GPIO_PORT_MASK) | (GPIO_NUM_MASK) )

#define	GPIO_PORT_A		(0)
#define	GPIO_PORT_B		(1)
#define	GPIO_PORT_C		(2)
#define	GPIO_PORT_D		(3)
#define	GPIO_PORT_E		(4)
#define	GPIO_PORT_F		(5)
#define	GPIO_PORT_G		(6)
#define	GPIO_PORT_H		(7)
#define	GPIO_PORT_I		(8)
#define	GPIO_PORT_J		(9)

#define GPIO_MODE_SHIFT         (14)
#define GPIO_MODE_MASK          (3 << (GPIO_MODE_SHIFT))
#define GPIO_MODE_INPUT         (0 << (GPIO_MODE_SHIFT))
#define GPIO_MODE_OUTPUT        (1 << (GPIO_MODE_SHIFT))
#define GPIO_MODE_FUNC          (2 << (GPIO_MODE_SHIFT))
#define GPIO_MODE_ANALOG        (3 << (GPIO_MODE_SHIFT))

/*
 * UP/DOWN are valid in input and function mode
 * OPENDRAIN is valid in output and function mode
 */
#define GPIO_PULL_SHIFT         (12)
#define GPIO_PULL_MASK          (3 << (GPIO_PULL_SHIFT))
#define GPIO_PULL_NONE          (0 << (GPIO_PULL_SHIFT))
#define GPIO_PULL_UP            (1 << (GPIO_PULL_SHIFT))
#define GPIO_PULL_DOWN          (2 << (GPIO_PULL_SHIFT))
#define GPIO_PULL_OD            (3 << (GPIO_PULL_SHIFT))

#define GPIO_VAL_SHIFT          (8)
#define GPIO_VAL_MASK           (1 << (GPIO_VAL_SHIFT))
#define GPIO_VAL_0              (0 << (GPIO_VAL_SHIFT))
#define GPIO_VAL_1              (1 << (GPIO_VAL_SHIFT))

#define GPIO_FUNC_SHIFT         (8)
#define GPIO_FUNC_MASK          (15 << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC(x)            (((x) << (GPIO_FUNC_SHIFT)) & GPIO_FUNC_MASK)
#define GPIO_VALFUNC_SHIFT      (8)
#define GPIO_VALFUNC_MASK       (15 << (GPIO_VALFUNC_SHIFT))

#define GPIO_PORTNUM(port, num) ((((port) << GPIO_PORT_SHIFT) & GPIO_PORT_MASK) | \
                                 (((num)  << GPIO_NUM_SHIFT)  & GPIO_NUM_MASK))

/*
 * [15: 14]: mode [0:input, 1:output, 2:func, 3:analog]
 * [13: 12]: pull [0:none,  1:up,     2:down, 3:opendrain]
 * [11: 8]:  valfunc   val set 0/1, if output mode
 *                     func15-0 is selected, if func mode
 * [ 7:0]:   portnum    port and bit number
 */
#define PORT_CTRL(portnum, mode, pull, valfunc) \
  (((portnum)  & (GPIO_PORTNUM_MASK) ) | \
   ((mode)     & (GPIO_MODE_MASK) )  | \
   ((pull)     & (GPIO_PULL_MASK) )   | \
   ((valfunc)  & (GPIO_VALFUNC_MASK)  ) )

#define PORT_CTRL_END           (0xffff)


#if 0
#define GPIO_MODULE_A	0
#define GPIO_MODULE_B	1
#define GPIO_MODULE_C	2
#define GPIO_MODULE_H	7
/*#define GPIO		(GPIO_PTR[8])*/
#endif


#endif
