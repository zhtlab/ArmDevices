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

#define _DEV_COUNTER_C_

#include        <stdint.h>
#include        <stdio.h>
#include        <string.h>

#include        "system.h"
#include        "devCounter.h"

struct _stDevCounter      counter;

const static uint32_t sms[] = {
  /*freerun            reset               gate,                */
  TIM_SMCR_SMS_INTCLK, TIM_SMCR_SMS_RESET, TIM_SMCR_SMS_GATED,
  /* start,           restart,            enc */
  TIM_SMCR_SMS_TRIGGER, TIM_SMCR_SMS_RESET, TIM_SMCR_SMS_ENC1
};
int
DevCounterInit(int unit, devCounterParam_t *param)
{
  stm32Dev_TIM          *p;

  int                   i;
  uint16_t              shift;
  uint32_t              cr1 = 0;
  uint32_t              cr2 = 0;
  uint32_t              dier = 0;
  uint32_t              smcr = 0;
  uint32_t              val;

  if(unit == -1) {
    memset(&counter, 0, sizeof(counter));
    counter.sc[1].dev  = TIM1_PTR;
    counter.sc[2].dev  = TIM2_PTR;
#ifdef TIM3_PTR
    counter.sc[3].dev  = TIM3_PTR;
#endif
#ifdef TIM3_PTR
    counter.sc[4].dev  = TIM4_PTR;
#endif
#ifdef TIM15_PTR
    counter.sc[15].dev = TIM15_PTR;
#endif
#ifdef TIM16_PTR
    counter.sc[16].dev = TIM16_PTR;
#endif
    goto end;
  }

  p = counter.sc[unit].dev;

  /*** clock trigger settings */
  if(param->chnum & DEVCOUNTER_SETCH(DEVCOUNTER_CH_CLKTRG)) {
    if((param->clktrg.mode & DEVTIME_CLKTRG_CTG_MASK) == DEVTIME_CLKTRG_CTG_INTERNAL) {
      smcr |= TIM_SMCR_ECE_MODE2DIS | TIM_SMCR_ETPS_DIV1 | TIM_SMCR_SMS_INTCLK;
      if(param->clktrg.polneg) {
        smcr |= TIM_SMCR_ETP_FALLING;
      }
      /* set the slave mode */
      if(param->clktrg.mode & DEVTIME_CLKTRG_SLAVE_MASK) smcr |= TIM_SMCR_MSM_SYNC;
      /* set the input clock/triger mode */
      val   = param->clktrg.mode & DEVTIME_CLKTRG_MODE_MASK;
      val >>= DEVTIME_CLKTRG_MODE_SHIFT;
      smcr |= sms[val];
      /* set input trigger sel */
      val   = param->clktrg.mode & DEVTIME_CLKTRG_SEL_MASK;
      val >>= DEVTIME_CLKTRG_SEL_SHIFT;
      val <<= TIM_SMCR_TS_SHIFT;
      smcr |= val & TIM_SMCR_TS_MASK;
      /* set output trigger sel */
      val   = param->clktrg.mode & DEVTIME_CLKTRG_TRGO_MASK;
      val >>= DEVTIME_CLKTRG_TRGO_SHIFT;
      val <<= TIM_CR2_MMS_SHIFT;
      cr2 |= val & TIM_CR2_MMS_MASK;
    } else {
      if((param->clktrg.mode & DEVTIME_CLKTRG_CTG_MASK) == DEVTIME_CLKTRG_CTG_EXTERNAL2) {
        smcr |= TIM_SMCR_ECE_MODE2EN;
        smcr |= (param->clktrg.filter << TIM_SMCR_ETF_SHIFT) & TIM_SMCR_ETF_MASK;
        if(param->clktrg.polneg) {
          smcr |= TIM_SMCR_ETP_FALLING;
        }
      }
    }
    if(param->clktrg.intr) {
      dier |= TIM_DIER_UIE_YES;
    }
    if(param->clktrg.dma) {
      dier |= TIM_DIER_UDE_YES;
    }
    if(param->clktrg.down) {
      cr1 |= TIM_CR1_DIR_DOWN;  /* down counter */
    }
    if(!(smcr &TIM_SMCR_MSM_MASK)) {
      cr1 |= TIM_CR1_CEN_YES;
    }
  }

  /*** capture/compare channel settings */
  uint32_t      ccmr[2] = {0, 0};
  uint32_t      ccer = 0;
  uint32_t      bdtr = 0;
  int           pos, bit;
  shift = param->chnum;
  shift >>= 1;
  for(i = 0; i < DEVCOUNTER_CH_MAX; i++) {
    shift >>= 1;
    if(shift & 1) {
      ccer |= TIM_CCER_CC1E_YES   << ((i)*2);   /* capture/compare enable */
      if(param->ch.polneg) {
        ccer |= TIM_CCER_CC1P_YES << ((i)*2);   /* capture/compare polarity */
      }
      if(param->ch.intr) {
        dier |= TIM_DIER_CC1IE_YES << (i>>1);
      }
      if(param->ch.dma) {
        dier |= TIM_DIER_CC1DE_YES << (i>>1);
      }

      pos =  i >> 2;
      bit = ((i>>1)  & 1) << 3;
      if(param->ch.mode == DEVCOUNTER_MODE_COUNTER) {
        (&p->CCR1)[i>>1] = param->ch.val;

      } else if(param->ch.mode == DEVCOUNTER_MODE_IC) {
        ccmr[pos] |= TIM_CCMR1_CC1S_TI1 << bit;
        ccmr[pos] |= param->ch.filter << (bit + TIM_CCMR1_IC1F_SHIFT);

      } else  if(param->ch.mode == DEVCOUNTER_MODE_PWM) {
        ccmr[pos] |= TIM_CCMR1_CC1S_OUTPUT  << bit;
        ccmr[pos] |= TIM_CCMR1_OC1M_PWM1 << bit;
        (&p->CCR1)[i>>1] = param->ch.val;
        bdtr = TIM_BDTR_MOE_YES;
      }
    }
    p->CCMR1 |= ccmr[0];
    p->CCMR2 |= ccmr[1];
    p->CCER  |= ccer;
    p->BDTR   = bdtr;
    p->DIER  |= dier;
  }

  if(param->chnum & DEVCOUNTER_SETCH(DEVCOUNTER_CH_CLKTRG)) {
    p->ARR = param->clktrg.reload;
    p->PSC = param->clktrg.prescaler;
    p->RCR = 0;
    p->SMCR = smcr;

    p->CR2  = cr2;
    p->CR1 |= cr1;
  }
end:
  return 0;
}


int
DevCounterGetCounterValue(int unit, int ch, uint32_t *pVal)
{
  int                   result = -1;
  stm32Dev_TIM          *p;

  if(!pVal) goto fail;

  if(ch > 4 || ch < 0) goto fail;
  p = counter.sc[unit].dev;
  *pVal = (&p->CNT)[ch-1];

  result = 0;
fail:
  return result;
}
int
DevCounterSetPwmDutyValue(int unit, int ch, uint32_t val)
{
  int                   result = -1;
  stm32Dev_TIM          *p;

  if(ch > 4 || ch < 0) goto fail;
  p = counter.sc[unit].dev;
  (&p->CCR1)[ch-1] = val;

  result = 0;
fail:
  return result;
}
int
DevCounterGetIcValue(int unit, int ch, uint32_t *pVal)
{
  int                   result = -1;
  stm32Dev_TIM          *p;

  if(!pVal || ch > 4 || ch < 0) goto fail;

  p = counter.sc[unit].dev;
  *pVal = (&p->CCR1)[ch-1];

  result = 0;

fail:
  return result;
}


void
DevCounterDebugShowRegs(int unit)
{
  stm32Dev_TIM          *p;

  p = counter.sc[unit].dev;

  printf("---- unit: %d, dev: %4x\r\n", unit, p);
  printf("cr1:   %4x, cr2:   %4x\r\n", p->CR1, p->CR2);
  printf("smcr1: %4x, dier2: %4x, arr:  %4x\r\n", p->SMCR, p->DIER, p->ARR);
  printf("ccmr1: %4x, ccmr2: %4x, ccer: %4x\r\n", p->CCMR1, p->CCMR2, p->CCER);
  printf("ccr1:  %4x, ccr2:  %4x, ccr3: %4x, ccr4: %4x\r\n", p->CCR1, p->CCR2, p->CCR3, p->CCR4);

  return;
}
