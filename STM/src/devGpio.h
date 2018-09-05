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
#ifndef _DEVGPIO_H_
#define _DEVGPIO_H_

/**************************************************************
 * Port Function Register
 * [15:14] mode  in/out/func/analog
 * [13:12] pull none/up/down
 * [11:9]  reserved (if mode is in/out)
 * [8]     val      (if mode is in/out)
 * [11:8]  func     (if mode is func)
 * [7:4]   port (A -- J)
 * [3:0]   num (0 -- 15)
 */

#define GPIO_NUM_SHIFT          (0)
#define GPIO_NUM_MASK           (0xf << (GPIO_NUM_SHIFT))
#define GPIO_PORT_SHIFT         (4)
#define GPIO_PORT_MASK          (0xf << (GPIO_PORT_SHIFT))
#define GPIO_PORTNUM_SHIFT      (0)
#define GPIO_PORTNUM_MASK       ( (GPIO_PORT_MASK) | (GPIO_NUM_MASK) )

#define GPIO_PORT_A             (0)
#define GPIO_PORT_B             (1)
#define GPIO_PORT_C             (2)
#define GPIO_PORT_D             (3)
#define GPIO_PORT_E             (4)
#define GPIO_PORT_F             (5)
#define GPIO_PORT_G             (6)
#define GPIO_PORT_H             (7)
#define GPIO_PORT_I             (8)
#define GPIO_PORT_J             (9)


#define GPIO_MODE_SHIFT          (14)
#define GPIO_MODE_MASK           (3 << (GPIO_MODE_SHIFT))
#define GPIO_MODE_INPUT          (0 << (GPIO_MODE_SHIFT))
#define GPIO_MODE_OUTPUT         (1 << (GPIO_MODE_SHIFT))
#define GPIO_MODE_FUNC           (2 << (GPIO_MODE_SHIFT))
#define GPIO_MODE_ANALOG         (3 << (GPIO_MODE_SHIFT))

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
#define GPIO_FUNC0_SYS          ((FUNC0_SYS) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC1_TIM          ((FUNC1_TIM) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC2_TIM          ((FUNC2_TIM)) << (GPIO_FUNC_SHIFT)
#define GPIO_FUNC3_TIM          ((FUNC3_TIM)) << (GPIO_FUNC_SHIFT)
#define GPIO_FUNC3_USART        ((FUNC3_USART) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC4_I2C          ((FUNC4_I2C) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC5_SPI          ((FUNC5_SPI) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC6_SPI          ((FUNC6_SPI) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC7_USART        ((FUNC7_USART) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC8_LPUART       ((FUNC8_LPUART) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC9_CAN          ((FUNC9_CAN) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC9_FMC          ((FUNC9_FMC) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC10_USB         ((FUNC10_USB) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC11_LCD         ((FUNC11_LCD) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC12_SD          ((FUNC12_SD) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC12_COMP        ((FUNC12_COMP) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC12_FMC         ((FUNC12_FMC) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC13_SAI         ((FUNC13_SAI) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC14_TIM         ((FUNC14_TIM) << (GPIO_FUNC_SHIFT))
#define GPIO_FUNC15             ((FUNC15) << (GPIO_FUNC_SHIFT))
#define GPIO_VALFUNC_SHIFT      (8)
#define GPIO_VALFUNC_MASK       (15 << (GPIO_VALFUNC_SHIFT))

#define GPIO_PORTNUM(port, num)  ((((port) << GPIO_PORT_SHIFT) & GPIO_PORT_MASK) | \
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
#define GPIO_DRIVE_EN(portnum)  \
  (GPIO_PTR[(((portnum) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)].otype =  (1<<(((portnum) & GPIO_NUM_MASK) >> GPIO_NUM_SHIFT)))
#define GPIO_DRIVE_DIS(portnum) \
  (GPIO_PTR[(((portnum) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)].otype =  (1<<(((portnum) & GPIO_NUM_MASK) >> GPIO_NUM_SHIFT)))
#endif

#define GPIO_GET(portnum)       \
  (((GPIO_PTR[(((portnum) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)].idr) >> (((portnum) & GPIO_NUM_MASK) >> GPIO_NUM_SHIFT)) & 1)
#define GPIO_GETO(portnum)      \
  (((GPIO_PTR[(((portnum) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)].odr) >> (((portnum) & GPIO_NUM_MASK) >> GPIO_NUM_SHIFT)) & 1)
#define GPIO_SET(portnum)       \
  (GPIO_PTR[(((portnum) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)].bsrr =  (1<<(((portnum) & GPIO_NUM_MASK) >> GPIO_NUM_SHIFT)))
#define GPIO_CLEAR(portnum)     \
  (GPIO_PTR[(((portnum) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)].brr  =  (1<<(((portnum) & GPIO_NUM_MASK) >> (GPIO_NUM_SHIFT))))










void            DevGpioSet(uint16_t v);
void            DevGpioSets(const uint16_t *pGpio);
void            DevGpioInit(void);
void            DevGpioAnalogInit(int group, int bit);



//#include        "gpio.aaa.h"
//#include        "gpio.gpsdo-10m.h"


#endif
