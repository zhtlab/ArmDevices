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

#ifndef _STM32L0SYSCFG_H_
#define _STM32L0SYSCFG_H_

typedef struct {
  __IO uint32_t         CFGR1;             /* 0x00 */
  __IO uint32_t         CFGR2;             /* 0x04 */
  __IO uint32_t         CFGR_EXTICR1;      /* 0x08 */
  __IO uint32_t         CFGR_EXTICR2;      /* 0x0c */
  __IO uint32_t         CFGR_EXTICR3;      /* 0x10 */
  __IO uint32_t         CFGR_EXTICR4;      /* 0x14 */
  __IO uint32_t         CFGR_COMP1_CTRL;   /* 0x18 */
  __IO uint32_t         CFGR_COMP2_CTRL;   /* 0x1c */

  __IO uint32_t         CFGR3;             /* 0x20 */
#define SYSCFG_CFGR3_ENREF_HSI48_SHIFT      13
#define SYSCFG_CFGR3_ENREF_HSI48_MASK       (1 << (SYSCFG_CFGR3_ENREF_HSI48_SHIFT))
#define SYSCFG_CFGR3_ENREF_HSI48_NO         (0 << (SYSCFG_CFGR3_ENREF_HSI48_SHIFT))
#define SYSCFG_CFGR3_ENREF_HSI48_YES        (1 << (SYSCFG_CFGR3_ENREF_HSI48_SHIFT))
#define SYSCFG_CFGR3_EN_VREFINT_SHIFT      0
#define SYSCFG_CFGR3_EN_VREFINT_MASK       (1 << (SYSCFG_CFGR3_EN_VREFINT_SHIFT))
#define SYSCFG_CFGR3_EN_VREFINT_NO         (0 << (SYSCFG_CFGR3_EN_VREFINT_SHIFT))
#define SYSCFG_CFGR3_EN_VREFINT_YES        (1 << (SYSCFG_CFGR3_EN_VREFINT_SHIFT))


} stm32Dev_SYSCFG;

#endif
