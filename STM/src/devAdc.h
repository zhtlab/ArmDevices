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

#ifndef _DEV_ADC_H_
#define _DEV_ADC_H_


typedef struct {
  uint16_t              mode;
  uint8_t               dma;

  uint8_t               trgpol;
#define DEVADC_TRGSEL_DISABLE   0
#define DEVADC_TRGSEL_RIGING    1
#define DEVADC_TRGSEL_FALLING   2
#define DEVADC_TRGSEL_BOTH      3
  uint8_t               trgsel; /* depend on module */
  uint8_t               resolution;
  uint8_t               txmode;
#define DEVADC_TXMODE_PIO       0
#define DEVADC_TXMODE_DMAONESHOT 1
#define DEVADC_TXMODE_DFSDM     2
#define DEVADC_TXMODE_DMACIRC   3
  uint8_t               continuous;
  uint32_t              chbit;

  void                  *cb;

} devAdcParam_t;



typedef struct {
  uint8_t               up;
  uint8_t               unit;
  stm32Dev_ADC          *dev;
  int                   (*cb)(int req, void *ptr, int size);
#define DEVADC_CB_UKNOWN        0
#define DEVADC_CB_DONE_CH       1
#define DEVADC_CB_DONE_ALL      2
  uint8_t               numCh;

  uint8_t               cntCh;
  uint8_t               nextCh;
  uint32_t              chbit;
  uint16_t              val[17];
} devAdcSc_t;

struct _stAdc {
  devAdcSc_t          sc[4];
};


int             DevAdcInit(int unit, int ch, devAdcParam_t *param);
int             DevAdcStartConvert(int unit);
int             DevAdcGetData(int unit, int ch, uint16_t *ptr);
int             DevAdcGetDataPointer(int unit, int ch, uint16_t **ptr);

void            DevAdcInterrupt1(void);


#ifdef  _DEV_ADC_C_
static void     DevAdcInterrupt(devAdcSc_t *psc);
#endif

#endif
