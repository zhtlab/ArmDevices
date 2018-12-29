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

#ifndef _STM32CRS_H_
#define _STM32CRS_H_

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
#define CRS_CR_ESYNCIE_SHIFT    (3)
#define CRS_CR_ESYNCIE_MASK     (1 << (CRS_CR_ESYNCIE_SHIFT))
#define CRS_CR_ESYNCIE_NO       (0 << (CRS_CR_ESYNCIE_SHIFT))
#define CRS_CR_ESYNCIE_YES      (1 << (CRS_CR_ESYNCIE_SHIFT))
#define CRS_CR_ERRIE_SHIFT      (2)
#define CRS_CR_ERRIE_MASK       (1 << (CRS_CR_ERRIE_SHIFT))
#define CRS_CR_ERRIE_NO         (0 << (CRS_CR_ERRIE_SHIFT))
#define CRS_CR_ERRIE_YES        (1 << (CRS_CR_ERRIE_SHIFT))
#define CRS_CR_SYNCWARNIE_SHIFT (1)
#define CRS_CR_SYNCWARNIE_MASK  (1 << (CRS_CR_SYNCWARNIE_SHIFT))
#define CRS_CR_SYNCWARNIE_NO    (0 << (CRS_CR_SYNCWARNIE_SHIFT))
#define CRS_CR_SYNCWARNIE_YES   (1 << (CRS_CR_SYNCWARNIE_SHIFT))
#define CRS_CR_SYNCOKIE_SHIFT   (0)
#define CRS_CR_SYNCOKIE_MASK    (1 << (CRS_CR_SYNCOKIE_SHIFT))
#define CRS_CR_SYNCOKIE_NO      (0 << (CRS_CR_SYNCOKIE_SHIFT))
#define CRS_CR_SYNCOKIE_YES     (1 << (CRS_CR_SYNCOKIE_SHIFT))


/*** 0x04  CRS CFGR */
  __IO uint32_t         CFGR;
#define CRS_CFGR_SYNCPOL_SHIFT      (31)
#define CRS_CFGR_SYNCPOL_MASK       (1 << (CRS_CFGR_SYNCPOL_SHIFT))
#define CRS_CFGR_SYNCPOL_RISING     (0 << (CRS_CFGR_SYNCPOL_SHIFT))
#define CRS_CFGR_SYNCPOL_FALLING    (1 << (CRS_CFGR_SYNCPOL_SHIFT))
#define CRS_CFGR_SYNCSRC_SHIFT      (28)
#define CRS_CFGR_SYNCSRC_MASK       (3 << (CRS_CFGR_SYNCSRC_SHIFT))
#define CRS_CFGR_SYNCSRC_GPIO       (0 << (CRS_CFGR_SYNCSRC_SHIFT))
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
#define CRS_ISR_ESYNCF_SHIFT    (3)
#define CRS_ISR_ESYNCF_MASK     (1 << (CRS_ISR_ESYNCF_SHIFT))
#define CRS_ISR_ERRF_SHIFT      (2)
#define CRS_ISR_ERRF_MASK       (1 << (CRS_ISR_ERRF_SHIFT))
#define CRS_ISR_SYNCWARNF_SHIFT (1)
#define CRS_ISR_SYNCWARNF_MASK  (1 << (CRS_ISR_SYNCWARNF_SHIFT))
#define CRS_ISR_SYNCOKF_SHIFT   (0)
#define CRS_ISR_SYNCOKF_MASK    (1 << (CRS_ISR_SYNCOKF_SHIFT))

  __IO uint32_t         ICR;
#define CRS_ICR_ESYNCC_SHIFT    (3)
#define CRS_ICR_ESYNCC_MASK     (1 << (CRS_ICR_ESYNCC_SHIFT))
#define CRS_ICR_ERRC_SHIFT      (2)
#define CRS_ICR_ERRC_MASK       (1 << (CRS_ICR_ERRC_SHIFT))
#define CRS_ICR_SYNCWARNC_SHIFT (1)
#define CRS_ICR_SYNCWARNC_MASK  (1 << (CRS_ICR_SYNCWARNC_SHIFT))
#define CRS_ICR_SYNCOKC_SHIFT   (0)
#define CRS_ICR_SYNCOKC_MASK    (1 << (CRS_ICR_SYNCOKC_SHIFT))
#define CRS_ICR_ALL_SHIFT       (0)
#define CRS_ICR_ALL_MASK        (0xf << (CRS_ICR_ALL_SHIFT))
} stm32Dev_CRS;

#endif
