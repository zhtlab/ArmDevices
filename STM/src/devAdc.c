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

#define _DEV_ADC_C_

#include        <string.h>

#include        "system.h"
#include        "devCounter.h"
#include        "devAdc.h"

struct _stAdc         adc;

int
DevAdcInit(int unit, int ch, devAdcParam_t *param)
{
  int                   result = -1;
  uint32_t              cr = 0;
  uint32_t              cfgr = 0, cfgr2 = 0;
  uint32_t              sampleTime = 0;
  devAdcSc_t            *psc;
  stm32Dev_ADC          *p;

  if(unit == -1) {
    memset(&adc, 0, sizeof(adc));
    adc.sc[1].dev = ADC1_PTR;
#ifdef  ADC2_PTR
    adc.sc[2].dev = ADC2_PTR;
#endif
#ifdef  ADC3_PTR
    adc.sc[3].dev = ADC3_PTR;
#endif
    goto end;
  }

  psc = &adc.sc[unit];
  p = psc->dev;

  psc->cb = param->cb;

#if 0
  if(unit == 3) {
    pCommon = ADC3_COMMON;
  } else {
    pCommon = ADC12_COMMON;
  }
#endif

  /*** AD DEEPPWD   exit power down mode */
  if(p->CR & ADC_CR_DEEPPWD_YES) {
    p->CR &= ~ADC_CR_DEEPPWD_MASK;
  }

  /*** CR ADVREGEN   turn on the voltage regulartor */
  cr |= ADC_CR_ADVREGEN_YES;
  p->CR = cr;
  SystemWaitSystemTimer(DEVCOUNTER_SYSTIMER_10U00S);

  /* CCR */
  if(!(unit & 1)
#ifndef ADC2_PTR
     || 1
#endif
     ) {
    p->CCR = (ADC_CCR_PRESC_DIV8 |
              ADC_CCR_DELAY_VAL(2) |
              ADC_CCR_DUAL_INDEPENDENT
              );
  }

  /*** CFGR */
  if(param->resolution > 14) {
    cfgr  |= ADC_CFGR_RES_16BIT;
  } else if(param->resolution > 12) {
    cfgr  |= ADC_CFGR_RES_14BIT;
  } else if(param->resolution > 8) {
    cfgr  |= ADC_CFGR_RES_12BIT;
  } else {
    cfgr  |= ADC_CFGR_RES_8BIT;
  }
#ifdef  ADC_CFGR_ALIGN_YES
  cfgr |= ADC_CFGR_ALIGN_YES;
#endif
  cfgr |= param->trgsel << ADC_CFGR_EXTSEL_SHIFT;
  cfgr |= param->trgpol << ADC_CFGR_EXTEN_SHIFT;
  switch(param->txmode) {
  case DEVADC_TXMODE_PIO:        cfgr |= ADC_CFGR_DMNGT_REGULAR; break;
  case DEVADC_TXMODE_DMAONESHOT: cfgr |= ADC_CFGR_DMNGT_DMAONESHOT; break;
  case DEVADC_TXMODE_DFSDM:      cfgr |= ADC_CFGR_DMNGT_DFSDM; break;
  case DEVADC_TXMODE_DMACIRC:    cfgr |= ADC_CFGR_DMNGT_DMACIRC; break;
  }
  if(param->continuous) cfgr |= ADC_CFGR_CONT_YES;
  p->CFGR = cfgr;

  /* CFGR2 */
  p->CFGR2 = cfgr2;

  /*** CR BOOST */
  cr |= ADC_CR_BOOST_YES;
  p->CR = cr;

  uint32_t      chbit;
  int           i, cnt;
  psc->chbit = param->chbit;
  chbit = psc->chbit;
  /*** PCSEL   channel setting */
  p->PCSEL = chbit;          /* ch9: preselection */

  p->SQR1 = 0;
  p->SQR2 = 0;
  p->SQR3 = 0;
  p->SQR4 = 0;

  i = 1;
  chbit >>= 1;
  cnt = 0;
  while(chbit) {
    if(chbit & 1) {
      cnt++;
      /* SQR */
      if(cnt <= 4) {
        p->SQR1 |= i << ( cnt     * ADC_SQR1_SQ1_SHIFT);
      } else if(cnt <= 9) {
        p->SQR2 |= i << ((cnt- 5) * ADC_SQR1_SQ1_SHIFT);
      } else if(cnt <= 14) {
        p->SQR3 |= i << ((cnt-10) * ADC_SQR1_SQ1_SHIFT);
      } else if(cnt <= 16) {
        p->SQR4 |= i << ((cnt-15) * ADC_SQR1_SQ1_SHIFT);
      }

      /*** SMPR */
      /* 64.5 ADC clk */
      sampleTime = ADC_SMPR_SMP_64_5CLKS;
      if(i <= 9) {
        p->SMPR1 |= sampleTime << (i)*3;
      } else {
        p->SMPR2 |= sampleTime << (i-10)*3;
      }

    }
    chbit >>= 1;
    i++;
  }
  psc->numCh = cnt;
  if(cnt >= 1) p->SQR1 |= cnt-1;

  /*** IER */
  //
  p->IER = ADC_ISR_OVR_MASK | ADC_ISR_EOC_MASK | ADC_ISR_EOS_MASK | ADC_ISR_EOSMP_MASK;
  p->IER = ADC_ISR_EOC_MASK;

  p->OFR1 = 0;

  /*** CR ADEN */
  p->ISR = ADC_ISR_ADRDY_YES;                     /* clear */
  do {
    p->CR |= ADC_CR_ADEN_YES;
  } while(!(p->ISR & ADC_ISR_ADRDY_MASK));

  /*** CR ADSTART */
  p->CR |= ADC_CR_ADSTART_YES;

#if 0
  printf("cfgr %x, cfgr2 %x, smpr1 %x\r\n", p->CFGR, p->CFGR2, p->SMPR1);
  printf("pcsel %x, sqr1 %x sqr2 %x sqr3 %x sqr4 %x\r\n",
         p->PCSEL, p->SQR1, p->SQR2, p->SQR3, p->SQR4);
  printf("ccr %x, cr %x\r\n", p->CCR, p->CR);
#endif

  result = 0;

end:
  return result;
}


int
DevAdcStartConvert(int unit)
{
  int                   result = -1;
  devAdcSc_t            *psc;
  stm32Dev_ADC          *p;

  uint32_t              chbit;

  psc = &adc.sc[unit];
  p = psc->dev;

  p->ISR = 0xff;

  psc->cntCh = psc->numCh;
  chbit = psc->chbit;
  int   i;
  for(i = 0; i < 32; i++) {
    if(!chbit) goto fail;
    if(chbit & 1) break;
    chbit >>= 1;
  }
  psc->nextCh = i;

  /*** CR ADSTART */
  p->CR |= ADC_CR_ADSTART_YES;

end:
  result = 0;
fail:
  return result;
}


int
DevAdcGetData(int unit, int ch, uint16_t *ptr)
{
  int                   result = -1;
  devAdcSc_t            *psc;

  if(!ptr) goto fail;

  psc = &adc.sc[unit];
  *ptr = psc->val[ch];

  result = 0;
fail:
  return result;
}

int
DevAdcGetDataPointer(int unit, int ch, uint16_t **ptr)
{
  int                   result = -1;
  stm32Dev_ADC          *p;

  p = adc.sc[unit].dev;

  *ptr = (uint16_t *)&p->DR;

  result = 0;

  return result;
}


void
DevAdcInterrupt1(void)
{
  DevAdcInterrupt(&adc.sc[1]);
  return;
}
void
DevAdcInterrupt2(void)
{
  DevAdcInterrupt(&adc.sc[2]);
  return;
}
void
DevAdcInterrupt3(void)
{
  DevAdcInterrupt(&adc.sc[3]);
  return;
}
static void
DevAdcInterrupt(devAdcSc_t *psc)
{
  stm32Dev_ADC          *p;
  uint32_t              intr;
  uint32_t              chbit;
  uint16_t              val;

  p = psc->dev;

  intr = p->ISR;
  p->ISR = intr;


#if 0
  if(intr & ADC_ISR_OVR_MASK) {
  }
#endif
  if(intr & ADC_ISR_EOC_MASK) {
    val = p->DR;
    psc->val[psc->nextCh] = val;

    if(psc->cb) psc->cb(DEVADC_CB_DONE_CH, &val, sizeof(val));

    psc->cntCh--;
    if(psc->cntCh) {

      psc->nextCh++;

      chbit = psc->chbit >> psc->nextCh;
      int   i;
      for(i = psc->nextCh; i < 32; i++) {
        if(!chbit) goto fail;
        if(chbit & 1) break;
        chbit >>= 1;
      }
      psc->nextCh = i;
    } else {
      if(psc->cb) psc->cb(DEVADC_CB_DONE_ALL, NULL, 0);
    }
  }
#if 0
  if(intr & ADC_ISR_EOS_MASK) {
  }
  if(intr & ADC_ISR_EOSMP_MASK) {
  }
#endif

fail:
  return;
}
