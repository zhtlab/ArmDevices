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

#ifndef _STM32L0PWR_H_
#define _STM32L0PWR_H_

typedef struct {
  __IO uint32_t         CR;             /* 0x00 */
#define PWR_CR_LPRUN_SHIFT              14
#define PWR_CR_LPRUN_MASK               (1 << (PWR_CR_LPRUN_SHIFT))
#define PWR_CR_LPRUN_NO                 (0 << (PWR_CR_LPRUN_SHIFT))
#define PWR_CR_LPRUN_YES                (1 << (PWR_CR_LPRUN_SHIFT))
#define PWR_CR_VOS_SHIFT                11
#define PWR_CR_VOS_MASK                 (3 << (PWR_CR_VOS_SHIFT))
#define PWR_CR_VOS_RANGE_FORBIDDEN      (0 << (PWR_CR_VOS_SHIFT))
#define PWR_CR_VOS_RANGE1_1800MV        (1 << (PWR_CR_VOS_SHIFT))
#define PWR_CR_VOS_RANGE2_1500MV        (2 << (PWR_CR_VOS_SHIFT))
#define PWR_CR_VOS_RANGE3_1200MV        (3 << (PWR_CR_VOS_SHIFT))

  __IO uint32_t         CSR;             /* 0x04 */
#define PWR_CSR_VOSF_SHIFT              4
#define PWR_CSR_VOSF_MASK               (1 << (PWR_CSR_VOSF_SHIFT))
} stm32Dev_PWR;

#endif
