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

#define _DEVCRS_C_

#include        "system.h"
#include        "devCrs.h"

int
DevCrsInit(devCrsParam_t *param)
{
  uint32_t      cfgr;

  /* configuration */
  cfgr = (CRS_CFGR_SYNCPOL_RISING |
          CRS_CFGR_SYNCDIV_DIV1 |
          CRS_CFGR_FELIM_VAL(34) |
          CRS_CFGR_RELOAD_48MHZ);
  if(param->src == CRS_CLOCK_SOURCE_USB2) {
    cfgr |= CRS_CFGR_SYNCSRC_USB2SOF;
  } else {
    cfgr |= CRS_CFGR_SYNCSRC_USB1SOF;
  }
  CRS_PTR->CFGR = cfgr;

  /* set the trim value */
  CRS_PTR->CR  = (CRS_CR_TRIM_VAL(32));
  /* interrupt enable */
  //CRS_PTR->CR |= CRS_CR_SYNCWARNIE_YES | CRS_CR_ERRIE_YES;
  /* start CRS */
  CRS_PTR->CR |= (CRS_CR_AUTOTRIMEN_YES | CRS_CR_CEN_YES);

  return 0;
}


void
DevCrsInterrupt(void)
{
  uint32_t      intr;
  intr = CRS_PTR->ISR;
  CRS_PTR->ICR = intr & CRS_ICR_ALL_MASK;

  if(intr & CRS_ISR_ESYNCF_MASK) {
  }
  if(intr & CRS_ISR_ERRF_MASK) {
  }
  if(intr & CRS_ISR_SYNCWARNF_MASK) {
  }
  if(intr & CRS_ISR_SYNCOKF_MASK) {
  }

  return;
}
