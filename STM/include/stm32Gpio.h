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
  __IO uint32_t		moder;
#define MODE_INPUT(x)		(0 << ((x)*2))
#define MODE_OUTPUT(x)		(1 << ((x)*2))
#define MODE_FUNC(x)		(2 << ((x)*2))
#define MODE_ANALOG(x)		(3 << ((x)*2))
#define MODE_MASK(x)		(3 << ((x)*2))
#define MODE_SET(x, v)		((v) << ((x)*2))
  __IO uint32_t		otyper;
#define OTYPE_PUSHPULL(x)	(0 << (x))
#define OTYPE_OD(x)		(1 << (x))
#define OTYPE_MASK(x)		(1 << (x))
  __IO uint32_t		ospeedr;
#define OSPEED_LOW(x)		(0 << ((x)*2))
#define OSPEED_MED(x)		(1 << ((x)*2))
#define OSPEED_HIGH(x)		(2 << ((x)*2))
#define OSPEED_EXHIGH(x)	(3 << ((x)*2))
#define OSPEED_MASK(x)		(3 << ((x)*2))
  __IO uint32_t		pupdr;
#define PUPD_NONE(x)		(0 << ((x)*2))
#define PUPD_PU(x)		(1 << ((x)*2))
#define PUPD_PD(x)		(2 << ((x)*2))
#define PUPD_RESERVED(x)	(3 << ((x)*2))
#define PUPD_X(x, d)		((d) << ((x)*2))
#define PUPD_MASK(x)		(3 << ((x)*2))
  __IO uint32_t		idr;
#define IDR_MASK(x)		(1 << (x))
  __IO uint32_t		odr;
#define ODR_0(x)		( 0  << (x))
#define ODR_1(x)		( 1  << (x))
#define ODR_X(x, d)		((d) << (x))
#define ODR_MASK(x)		( 1  << (x))
  __IO uint32_t		bsrr;
#define BSRR_SET(x)		( 1  << (x))
#define BSRR_RESET(x)		( 1  << ((x)+16))
  __IO uint32_t		lckr;
#define LCKR_LCKK_SHIFT		(16)
#define LCKR_LCKK_0		(0 << (LCKR_LCKK_SHIFT))
#define LCKR_LCKK_1		(1 << (LCKR_LCKK_SHIFT))
#define LCKR_LCKK_MASK		(1 << (LCKR_LCKK_SHIFT))
  __IO uint32_t		afrl;
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
#define FUNC3_USART	3
#define FUNC4_I2C	4
#define FUNC5_SPI	5
#define FUNC6_SPI	6
#define FUNC7_USART	7
#define FUNC8_LPUART	8
#define FUNC9_CAN	9
#define FUNC10_USB	10
#define FUNC11_LCD	11
#define FUNC12_SD	12
#define FUNC12_COMP	12
#define FUNC13_SAI	13
#define FUNC14_TIM	14
#define FUNC15		15

  __IO uint32_t		afrh;
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
  
  __IO uint32_t		brr;
#define BRR_RESET(x)		( 1  << (x))

  __IO uint32_t		gpio_reserved[0xf5];
} stm32Dev_GPIO;
#define GPIO_MODULE_A	0
#define GPIO_MODULE_B	1
#define GPIO_MODULE_C	2
#define GPIO_MODULE_H	7


#endif
