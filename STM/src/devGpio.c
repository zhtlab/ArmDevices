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

#define _DEVGPIO_C_

#include        <stdint.h>

#include       "config.h"
#include       "system.h"

#include       "devGpio.h"


/*
 *  PULL UP/DOWN, PUSH/PULL/OPENDRAIN settings table
 *
 *         |  NONE  |  UP     | DOWN    |  OD
 *  -------+--------+---------+---------+----------
 *  input  |  FLOAT |  UP     | DOWN    |  --
 *  output |   PP   |  UP+OD  | DOWN+OD |  OD
 *  func   |   PP   |  UP+OD  | DOWN+OD |  OD
 *  analog | ANALOG | reserved| reserved| reserved
 *
 *  pp/od = (mode == {output || func}) && pull != none
 *
 */
void
DevGpioSet(uint16_t v)
{
  uint32_t      val;
  int           port, num;
  int           mode, pull, d, func;

  port = (v & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  num  = (v & GPIO_NUM_MASK)  >> GPIO_NUM_SHIFT;
  mode = (v & GPIO_MODE_MASK)  >> GPIO_MODE_SHIFT;
  pull = (v & GPIO_PULL_MASK) >> GPIO_PULL_SHIFT;

  if(mode == (GPIO_MODE_OUTPUT>>GPIO_MODE_SHIFT)) {
    if(v & GPIO_VALFUNC_MASK) {
      GPIO_PTR[port].bsrr = (1<<num);
    } else {
      GPIO_PTR[port].brr  = (1<<num);
    }

  } else if(mode == (GPIO_MODE_FUNC>>GPIO_MODE_SHIFT)) {
    func = (v & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT;
    if(num <= AFL_7) {
      val = GPIO_PTR[port].afrl;
      val &= ~AFRL_MASK(num);
      val |=  AFRL_X(num, func);
      GPIO_PTR[port].afrl = val;
    } else {
      val = GPIO_PTR[port].afrh;
      val &= ~AFRH_MASK(num-8);
      val |=  AFRH_X(num-8, func);
      GPIO_PTR[port].afrh = val;
    }
#if 0
  } else if(mode == (GPIO_MODE_ANALOG>>GPIO_MODE_SHIFT)) {
    pull = 0;
#endif
  }

  val  = GPIO_PTR[port].otyper;
  val &= ~OTYPE_MASK(num);
  if(mode == (GPIO_MODE_FUNC>>GPIO_MODE_SHIFT) ||
     mode == (GPIO_MODE_OUTPUT>>GPIO_MODE_SHIFT)) {
    if(pull == (GPIO_PULL_NONE>>GPIO_PULL_SHIFT)) {
      val |=  OTYPE_PUSHPULL(num);
    } else {
      val |=  OTYPE_OD(num);
    }
  }
  GPIO_PTR[port].otyper = val;

  val = GPIO_PTR[port].pupdr;
  val &= ~PUPD_MASK(num);
  val |=  PUPD_X(num, pull);
  GPIO_PTR[port].pupdr = val;

  val = GPIO_PTR[port].ospeedr;
  val &= ~OSPEED_MASK(num);
  val |=  OSPEED_HIGH(num);
  GPIO_PTR[port].ospeedr = val;

  val = GPIO_PTR[port].moder;
  val &= ~MODE_MASK(num);
  val |=  MODE_SET(num, mode);
  GPIO_PTR[port].moder = val;

  return;
}
void
DevGpioSets(const uint16_t *pGpio)
{

  while(*pGpio != PORT_CTRL_END) {
    DevGpioSet(*pGpio++);
  }

  return;
}


void
DevGpioInit(void)
{
  /*DevGpioSets(gpioDefaultTbl);*/
  return;
}


void
DevGpioAnalogInit(int group, int bit)
{
  uint32_t              val;

  val = GPIO_PTR[group].moder;
  val &= ~MODE_MASK(bit);
  val |=  MODE_ANALOG(bit);
  GPIO_PTR[group].moder = val;

  return;
}

#if 0
void
DevGpioSetPowerLedOff(void)
{
  GPIO_PTR[CONFIG_POWER_LED_PORTNUM].brr = (1UL<<(CONFIG_POWER_LED_BITNUM));
  return;
}
void
DevGpioSetPowerLedOn(void)
{
  GPIO_PTR[CONFIG_POWER_LED_PORTNUM].bsrr = (1UL<<(CONFIG_POWER_LED_BITNUM));
  return;
}
void
DevGpioSetUpdateLedOff(void)
{
  GPIO_PTR[CONFIG_UPDATE_LED_PORTNUM].brr = (1UL<<(CONFIG_UPDATE_LED_BITNUM));
  return;
}
void
DevGpioSetUpdateLedOn(void)
{
  GPIO_PTR[CONFIG_UPDATE_LED_PORTNUM].bsrr = (1UL<<(CONFIG_UPDATE_LED_BITNUM));
  return;
}
int
DevGpioGetPowerSw(void)
{
  return (GPIO_PTR[CONFIG_POWER_SW_PORTNUM].idr >> (CONFIG_POWER_SW_BITNUM)) & 1;
}

#endif
