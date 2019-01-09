/*
 * Copyright (c) 2018,2019 zhtlab
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

#define _DEV_USART_C_

#include        "stdint.h"
#include        "string.h"

#include        "system.h"
#include        "intr.h"
#include        "fifo.h"
#include        "devUsart.h"

struct _stUsart         usart;

int
DevUsartInit(int unit, devUsartParam_t *param)
{
  int                   result = -1;
  stm32Dev_USART        *p;
  devUsartSc_t          *psc;

  systemClockFreq_t     clk;
  uint32_t              masterClk;
  int                   irq = 0;

  if(unit == -1) {
    memset(&usart, 0, sizeof(usart));
#ifdef  LPUART1_PTR
    usart.sc[1].dev = LPUART1_PTR;
#endif
#ifdef  USART1_PTR
    usart.sc[1].dev = USART1_PTR;
#endif
#ifdef  USART2_PTR
    usart.sc[2].dev = USART2_PTR;
#endif
#ifdef  USART3_PTR
    usart.sc[3].dev = USART3_PTR;
#endif
#ifdef  USART4_PTR
    usart.sc[4].dev = USART4_PTR;
#endif
#if 0
    usart.sc[4].dev = UART4;
    usart.sc[5].dev = UART5;
    usart.sc[6].dev = USART6;
    usart.sc[7].dev = UART7;
    usart.sc[8].dev = UART8;
#endif

    goto end;
  }

  if(unit > 8) goto fail;

  psc = &usart.sc[unit];
  p = psc->dev;
  psc->param = *param;

  SystemGetClockValue(&clk);

  switch(unit) {
#ifdef  USART2_PTR
  case        1:
    masterClk = clk.pclk2;      /* adhoc */
    irq = USART1_IRQn;
    break;
#endif
#ifdef  USART2_PTR
  case        2:
    masterClk = clk.pclk1;      /* adhoc */
    irq = USART2_IRQn;
    break;
#endif
#ifdef  USART3_PTR
  case        3:
    masterClk = clk.pclk2;      /* adhoc */
    irq = USART3_IRQn;
    break;
#endif
  }

  if(psc->param.mode == DEVUSART_MODE_FIFO) {
    int         d;
    if((d = FifoCreate(psc->param.szFifoTx)) < 0) {
      psc->param.mode == DEVUSART_MODE_PIO;
    } else {
      psc->dFifoTx = d;
      /* rx */
      if((d = FifoCreate(psc->param.szFifoRx)) < 0) {
        FifoDestroy(psc->dFifoTx);
        psc->param.mode == DEVUSART_MODE_PIO;
      } else {
        psc->dFifoRx = d;
      }
    }
  }
  psc->param.mode == DEVUSART_MODE_PIO;


  p->CR1  = USART_CR1_UE_YES | USART_CR1_FIFOEN_YES;  /* module enable */
  p->CR2  = 0;
  p->CR3  = 0;
  p->PRESC  = 0;

  /*** parity */
  if(param->parity & DEVUSART_PARITY_MASK) {
    if(param->parity & DEVUSART_PARITY_ODD) {
      p->CR1 |= USART_CR1_PS_ODD;
    }
    p->CR1 |= USART_CR1_PCE_YES;
  }
  /*** bit/char  default is 8bit */
  if(param->parity == DEVUSART_BIT_7) {
    p->CR1 |= USART_CR1_M_7BIT;
  } else if(param->parity == DEVUSART_BIT_9) {
    p->CR1 |= USART_CR1_M_9BIT;
  } else {
    p->CR1 &= ~USART_CR1_M_MASK;
  }
  /*** stop bit */
  if(param->parity == DEVUSART_STOP_2) {
    p->CR2 |= USART_CR2_STOP_2BIT;
  } else if(param->parity == DEVUSART_STOP_1_5) {
    p->CR2 |= USART_CR2_STOP_1_5BIT;
  } else {
    /* otherwise 1 bit */
    p->CR2 &= ~USART_CR2_STOP_MASK;
  }
  /*** baud */
  p->BRR = masterClk / param->baud;

  if(psc->param.mode == DEVUSART_MODE_FIFO) {
    p->CR3 |= USART_CR3_RXFTCFG_1_8 | USART_CR3_RXFTIE_YES;   /* threshold intr */
    p->CR1 |= USART_CR1_RXFFIE_YES;     /* full intr */
    p->CR1 |= USART_CR1_RXNEIE_YES;     /* rx intr */
  }
  /* enable error interrupt */
  p->CR3 |= USART_CR3_EIE_YES;

  /* enable */
  p->CR1 |= USART_CR1_TE_YES | USART_CR1_RE_YES;

  psc->unit = unit;
  psc->up = 1;

  if(irq) {
    IntrSetEntry(0, irq, DevUsart2Interrupt);
    NVIC_SetPriority(irq, 0);
    NVIC_EnableIRQ(irq);
  }

end:
  result = 0;

fail:
  return result;
}


int
DevUsartLoop(int unit)
{
  stm32Dev_USART        *p;
  devUsartSc_t          *psc;

  if(unit > 8) goto fail;
  psc = &usart.sc[unit];
  if(!psc->up) goto fail;

#if 0
  if(psc->param.mode == DEVUSART_MODE_FIFO) {
    DevUsartSendFifo(psc);
  }
#endif

fail:
  return 0;
}


static void
DevUsartInterrupt(int unit)
{
  stm32Dev_USART        *p;
  devUsartSc_t          *psc;
  uint32_t              flag;

  if(unit > 8) goto fail;
  psc = &usart.sc[unit];

  p = psc->dev;

  flag = p->ISR;
  p->ICR = flag;

  /*** rx with fifo */
  if(psc->param.mode == DEVUSART_MODE_FIFO) {
    uint8_t             buf[USART_RX_FIFO_SIZE+1];      /* fifo + sz(RDR) */
    uint8_t             *pBuf;
    int                 i, c;

    if(flag & (USART_ISR_RXFT_MASK | USART_ISR_RXFF_MASK | USART_ISR_RXNE_MASK)) {
      pBuf = buf;
      i = 0;
      while(p->ISR & USART_ISR_RXFT_MASK) {
        *pBuf++ = p->RDR;
        i++;
      }
      FifoWriteIn(psc->dFifoRx, buf, i);
    }
    /*** rx without fifo */
    if(flag & USART_ISR_RXNE_MASK) {
      pBuf = buf;
      i = 0;
      while(p->ISR & USART_ISR_RXNE_MASK) {
        c = p->RDR;
        *pBuf++ = c;
        i++;
      }
      if(i > 0) FifoWriteIn(psc->dFifoRx, buf, i);
    }
  }
  /*** tx */
  if(flag & USART_ISR_TXE_MASK) {
    if(psc->param.mode == DEVUSART_MODE_FIFO) {
      DevUsartSendFifo(psc);
    }
  }
#if 0
  /*** overrun */
  if(flag & USART_ISR_ORE_MASK) {
  }
#endif

fail:

  return;
}
void
DevUsart1Interrupt(void)
{
  DevUsartInterrupt(1);
}
void
DevUsart2Interrupt(void)
{
  DevUsartInterrupt(2);
}
void
DevUsart3Interrupt(void)
{
  DevUsartInterrupt(3);
  return;
}
int
DevUsartSend(int unit, uint8_t *ptr, int size)
{
  stm32Dev_USART        *p;
  devUsartSc_t          *psc;

  if(unit > 8) goto fail;
  psc = &usart.sc[unit];
  if(!psc->up) goto fail;
  p = psc->dev;

#if 0
  if(psc->param.mode == DEVUSART_MODE_FIFO) {
    FifoWriteIn(psc->dFifoTx, ptr, size);
    p->CR1 |= USART_CR1_TXEIE_YES;

  } else if(psc->param.mode == DEVUSART_MODE_PIO) {
    DevUsartSendPio(psc, ptr, size);
  }
#endif
  DevUsartSendPio(psc, ptr, size);

fail:
  return 0;
}
int
DevUsartRecv(int unit, uint8_t *ptr, int size)
{
  stm32Dev_USART        *p;
  devUsartSc_t          *psc;
  int                   sz = 0;

  if(unit > 8) goto fail;
  psc = &usart.sc[unit];
  if(!psc->up) goto fail;
  p = psc->dev;

  if(psc->param.mode == DEVUSART_MODE_FIFO) {
    sz = FifoReadOut(psc->dFifoRx, ptr, size);

  } else if(psc->param.mode == DEVUSART_MODE_PIO) {
    uint32_t    flag;
    if(p->ISR & USART_ISR_RXNE_MASK) {
      *ptr = p->RDR;
      sz = 1;
    }
  }

fail:
  return sz;
}
int
DevUsartGetDataLen(int unit)
{
  stm32Dev_USART        *p;
  devUsartSc_t          *psc;
  int                   sz = 0;

  if(unit > 8) goto fail;
  psc = &usart.sc[unit];
  if(!psc->up) goto fail;
  p = psc->dev;

  if(psc->param.mode == DEVUSART_MODE_FIFO) {
    sz = FifoGetDirtyLen(psc->dFifoRx);

  } else if(psc->param.mode == DEVUSART_MODE_PIO) {
    if(p->ISR & USART_ISR_RXFT_MASK) sz = 1;
  }

fail:
  return sz;
}




/*************************************************************:
 * internal routines
 */
static int
DevUsartSendPio(devUsartSc_t *psc, uint8_t *ptr, int size)
{
  stm32Dev_USART        *p;

  p = psc->dev;

  while(size > 0) {
    while(!(p->ISR & USART_ISR_TXE_MASK));
    p->TDR = *ptr++;
    size--;
  }

fail:
  return 0;
}
static int
DevUsartSendFifo(devUsartSc_t *psc)
{
  stm32Dev_USART        *p;
  uint8_t               *ptr;
  int                   size, cnt;

  p = psc->dev;

  FifoGetReadPointer(psc->dFifoTx, &ptr, &size);
  cnt = 0;
  while(size-- > 0) {
    if(!(p->ISR & USART_CR1_TXEIE_MASK)) break;
    p->TDR = *ptr++;
    cnt++;
  }
  FifoAddReadPointer(psc->dFifoTx, cnt);

  if(FifoGetDirtyLen(psc->dFifoTx) == 0) {
    p->CR1 &= ~USART_CR1_TXEIE_MASK;
  }

fail:
  return 0;
}
