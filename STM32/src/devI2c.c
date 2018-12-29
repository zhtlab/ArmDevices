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

#define _DEV_I2C_C_

#include        <stdio.h>
#include        <string.h>

#include        "system.h"

#include        "devDma.h"
#include        "devI2c.h"

struct _stI2c         i2c;

#define SHOW_I2C_PHASE          0

/**
  * @brief  initilize the devcie
  * @param  unit   unit number
  * @param  param  the pointer of initialize parameters
  * @retval int  0: success
  */
int
DevI2cInit(int unit, devI2cParam_t *param)
{
  int                   result = -1;
  uint32_t              cfg1, cfg2;

  stm32Dev_I2C        *p;
  devI2cSc_t          *psc;

  if(unit == -1) {
    memset(&i2c, 0, sizeof(i2c));
#ifdef  I2C1_PTR
    i2c.sc[1].dev = I2C1_PTR;
#endif
#ifdef  I2C2_PTR
    i2c.sc[2].dev = I2C2_PTR;
#endif
#ifdef  I2C3_PTR
    i2c.sc[3].dev = I2C3_PTR;
#endif
#ifdef  I2C4_PTR
    i2c.sc[4].dev = I2C4_PTR;
#endif
#ifdef  I2C5_PTR
    i2c.sc[5].dev = I2C5_PTR;
#endif
#ifdef  I2C6_PTR
    i2c.sc[6].dev = I2C6_PTR;
#endif

    goto end;
  }

  if(unit > I2C_MODULE_COUNT) goto fail;
  psc = &i2c.sc[unit];
  p = psc->dev;

  /* copy the parameters */
  psc->param = *param;

  systemClockFreq_t     clk;
  int                   val;
  SystemGetClockValue(&clk);
  val   = clk.pclk1;
  val  /= psc->param.speed;
  val >>= 2;

  /* clear first */
  p->CR1  = 0;
  p->CR2  = 0;

  /* set line filter */
  p->CR1   = I2C_CR1_ANFOFF_NO | I2C_CR1_DNF_VAL(1);
  p->TIMINGR = (I2C_TIMINGR_PRESC_VAL(1)
                | I2C_TIMINGR_SCLDEL_VAL(1) | I2C_TIMINGR_SCLDEL_VAL(1)
                | I2C_TIMINGR_SCLH_VAL(val) | I2C_TIMINGR_SCLL_VAL(val) );
  //  p->CR1  |= I2C_CR1_NOSTRETCH_YES;
  p->CR1  |= I2C_CR1_PE_YES;

  if(psc->param.intr) {
    p->CR1  |= (I2C_CR1_NACKIE_YES | I2C_CR1_TCIE_YES |
                I2C_CR1_TXIE_YES | I2C_CR1_RXIE_YES);
  }
  if(psc->param.dma) {
  }

  psc->unit = unit;
  psc->up = 1;

end:
  result = 0;

fail:
  return result;
}


/**
  * @brief  interrupt process routine
  * @param  unit   unit number
  * @retval none
  */
static void
DevI2cInterrupt(int unit)
{
  stm32Dev_I2C          *p;
  devI2cSc_t            *psc;
  uint32_t              intr;

  if(unit > I2C_MODULE_COUNT) goto fail;
  psc = &i2c.sc[unit];
  p = psc->dev;

  intr = p->ISR;
  p->ICR = intr;

  if(intr & I2C_ISR_TXIS_MASK) {
    p->TXDR = *psc->ptr++;
    psc->cnt--;

    if(psc->cnt <= 0) {
      if(psc->seq == DEVI2C_SEQ_SEND_CMD) {
        if(psc->pkt.lenSend > 0) {
          psc->ptr = psc->pkt.ptrSend;
          psc->cnt = psc->pkt.lenSend;
          psc->seq = DEVI2C_SEQ_SEND_DATA;
        } else if(psc->pkt.lenRecv > 0) {
          psc->seq = DEVI2C_SEQ_RECV_RESTART;
        }
      } else if(psc->seq == DEVI2C_SEQ_SEND_DATA) {
        if(psc->pkt.lenRecv > 0) {
          psc->seq = DEVI2C_SEQ_RECV_RESTART;
        }
      }
    }

    if(psc->seq == DEVI2C_SEQ_RECV_RESTART) {
      p->CR2  = (I2C_CR2_NBYTES_VAL(psc->pkt.lenRecv) |
                 I2C_CR2_SADD_VAL(psc->pkt.addr<<1) |
                 I2C_CR2_RDWRN_RD);
      p->CR2 |= I2C_CR2_START;

      psc->ptr = psc->pkt.ptrRecv;
      psc->cnt = psc->pkt.lenRecv;
      psc->seq = DEVI2C_SEQ_RECV_DATA;
    }
  }

  if(intr & I2C_ISR_RXNE_MASK) {
    if(psc->cnt > 0) {
      *psc->ptr++ = p->RXDR;
      psc->cnt--;
#if 0
    } else {
      psc->seq = DEVI2C_SEQ_IDLE;
      p->CR2 |= I2C_CR2_STOP;
#endif
    }
  }

  if(intr & I2C_ISR_TC_MASK) {
    psc->seq = DEVI2C_SEQ_IDLE;
    p->CR2 |= I2C_CR2_STOP;
  }
  if(intr & I2C_ISR_NACKF_MASK) {
    psc->seq |= DEVI2C_SEQ_STAT_NACK_MASK;
    p->CR2 |= I2C_CR2_STOP;
  }

 fail:

  return;
}


/**
  * @brief  interrupt entry point for unit 1
  * @param  noen
  * @retval none
  */
void
DevI2c1Interrupt(void)
{
  DevI2cInterrupt(1);
}
/**
  * @brief  interrupt entry point for unit 2
  * @param  noen
  * @retval none
  */
void
DevI2c2Interrupt(void)
{
  DevI2cInterrupt(2);
}
/**
  * @brief  interrupt entry point for unit 3
  * @param  noen
  * @retval none
  */
void
DevI2c3Interrupt(void)
{
  DevI2cInterrupt(3);
}
/**
  * @brief  interrupt entry point for unit 4
  * @param  noen
  * @retval none
  */
void
DevI2c4Interrupt(void)
{
  DevI2cInterrupt(4);
}

/**
  * @brief  send routine
  * @param  unit  unit number
  * @param  ptr   the pointer of send data
  * @param  size  send data size
  * @retval result  success/fail
  */
int
DevI2cSend(int unit, uint32_t addr, uint8_t *ptr, int size)
{
  int                   result = -1;
  devI2cSc_t            *psc;

  if(unit > I2C_MODULE_COUNT) goto fail;
  psc = &i2c.sc[unit];
  if(!psc->up) goto fail;

  if(psc->param.dma) {
    result = DevI2cSendDma(psc, addr, ptr, size);
  } else {
    result = DevI2cSendPio(psc, addr, ptr, size);
  }

fail:
  return result;
}
/**
  * @brief  recv routine
  * @param  unit  unit number
  * @param  addr  slave address
  * @param  ptr   the pointer of send data
  * @param  size  recv buffer size
  * @retval size  size of recv
  */
int
DevI2cRecv(int unit, uint32_t addr, uint8_t *ptr, int size)
{
  int                   result = -1;
  devI2cSc_t            *psc;

  if(unit > I2C_MODULE_COUNT) goto fail;
  psc = &i2c.sc[unit];
  if(!psc->up) goto fail;

  if(psc->param.dma) {
    result = DevI2cRecvDma(psc, ptr, size);
  } else {
    result = DevI2cRecvPio(psc, addr, ptr, size);
  }

fail:
  return result;
}
/**
  * @brief  send routine
  * @param  unit  unit number
  * @param  ptr   the pointer of send data
  * @param  size  send data size
  * @retval result  success/fail
  */
int
DevI2cTransmit(int unit, devI2cPkt *p)
{
  int                   result = -1;
  devI2cSc_t            *psc;

  if(unit > I2C_MODULE_COUNT) goto fail;
  psc = &i2c.sc[unit];
  if(!psc->up) goto fail;

  while((psc->dev->ISR & 0x8000) ||
        (psc->seq != DEVI2C_SEQ_IDLE &&
         !(psc->seq & DEVI2C_SEQ_STAT_NACK_MASK)));

  if(psc->param.intr) {
    psc->dev->CR1  |= (I2C_CR1_TXIE_YES | I2C_CR1_RXIE_YES);
  }

  if(psc->param.dma) {
    result = DevI2cTransmitDma(psc, p);
  } else {
    result = DevI2cTransmitPio(psc, p);
  }

fail:
  return result;
}


/**
  * @brief  control routine
  * @param  unit  unit number
  * @param  req   request number
  * @param  ptr   the pointer of send data
  * @param  size  send data size
  * @retval result  success/fail
  */
int
DevI2cCtrl(int unit, uint32_t req, void *ptr, int size)
{
  int           result = -1;

  devI2cSc_t            *psc;
  stm32Dev_I2C          *p;

  if(unit > I2C_MODULE_COUNT) goto fail;
  psc = &i2c.sc[unit];
  p = psc->dev;

  switch(req & DEVI2C_REQ_MASK) {
  case  DEVI2C_CHOPPED_ACCESS:
    if((req & DEVI2C_CHOPPED_ACCESS_START) & DEVI2C_CHOPPED_ACCESS_MASK) {
      psc->addr7 = *(uint8_t *)ptr;
#if SHOW_I2C_PHASE
      printf("# i2c START cond addr:%x\r\n", psc->addr7);
#endif

      p->ICR = -1;
      p->CR2 = (I2C_CR2_NBYTES_VAL(size) | I2C_CR2_SADD_VAL(psc->addr7<<1));
      /* send the start condition */
      p->CR2 |= I2C_CR2_START;

      /* send the slave address */
      while(1) {
        uint32_t intr = p->ISR;
#if SHOW_I2C_PHASE
        printf("# i2c START wait %x %x %x\r\n", intr, size, p->CR2);
#endif
        if( intr & I2C_ISR_STOPF_MASK  ) {
          psc->slaveStatus = SLAVE_STATUS_ACK;
          if(intr & I2C_ISR_NACKF_MASK) {
            psc->slaveStatus = SLAVE_STATUS_NACK;
          }
          psc->seq = DEVI2C_SEQ_IDLE;
          goto fail;
        }
        if( intr & (I2C_ISR_NACKF_MASK | I2C_ISR_ALLERR_MASK) ) {
          psc->slaveStatus = SLAVE_STATUS_NACK;
          psc->seq = DEVI2C_SEQ_IDLE;
          goto fail;
        }
        if( intr & I2C_ISR_TC_MASK  ) break;
        if( intr & I2C_ISR_TXIS_MASK  ) break;
      }
      psc->slaveStatus = SLAVE_STATUS_ACK;
      psc->seq = DEVI2C_SEQ_SEND_CMD;
    }

    if((req & DEVI2C_CHOPPED_ACCESS_DATA_TX) & DEVI2C_CHOPPED_ACCESS_MASK) {
      if(psc->seq == DEVI2C_SEQ_IDLE) goto fail;
#if SHOW_I2C_PHASE
      printf("# i2c DATA TX size:%d\r\n", size);
#endif

      for(int i = 0; i < size; i++) {
        while(1) {
          uint32_t intr = p->ISR;
          if( intr & (I2C_ISR_NACKF_MASK | I2C_ISR_ALLERR_MASK) ) {
            psc->slaveStatus = SLAVE_STATUS_NACK;
            goto fail;
          }
          if( (intr & I2C_ISR_TXIS_MASK)) break;
        }
        p->TXDR = *((uint8_t *)ptr + i);
      }
      while(!(p->ISR & I2C_ISR_TC_MASK));
    }

    if((req & DEVI2C_CHOPPED_ACCESS_DATA_RX) & DEVI2C_CHOPPED_ACCESS_MASK) {
      if(psc->seq == DEVI2C_SEQ_IDLE) goto fail;
#if SHOW_I2C_PHASE
      printf("# i2c DATA RX size:%d\r\n", size);
#endif
      p->CR2 = (I2C_CR2_NBYTES_VAL(size) | I2C_CR2_SADD_VAL(psc->addr7<<1) |
                I2C_CR2_RDWRN_RD);

      /* send the start condition */
      p->CR2 |= I2C_CR2_START;

      while(1) {
        uint32_t intr = p->ISR;
        if( intr & (I2C_ISR_NACKF_MASK |  I2C_ISR_ALLERR_MASK)) {
          psc->slaveStatus = SLAVE_STATUS_NACK;
          goto fail;
        }
        if( intr & I2C_ISR_RXNE_MASK  ) break;
        if( intr & I2C_ISR_STOPF_MASK  ) break;
      }
      psc->slaveStatus = SLAVE_STATUS_ACK;

      for(int i = 0; i < size; i++) {
        while(1) {
          uint32_t intr = p->ISR;
          if( (intr & I2C_ISR_RXNE_MASK)) break;
        }
        *((uint8_t *)ptr + i) = p->RXDR;
      }
      while(!(p->ISR & I2C_ISR_TC_MASK));
    }

    if(((req & DEVI2C_CHOPPED_ACCESS_STOP) & DEVI2C_CHOPPED_ACCESS_MASK) &&
       !(p->ISR & I2C_ISR_STOPF_MASK)) {
#if SHOW_I2C_PHASE
      printf("# i2c STOP cond\r\n");
#endif
      if(psc->seq == DEVI2C_SEQ_IDLE) goto fail;
      psc->seq == DEVI2C_SEQ_IDLE;

      p->CR2 |= I2C_CR2_STOP;

      while(1) {
        uint32_t intr = p->ISR;
        if( intr & (I2C_ISR_NACKF_MASK | I2C_ISR_ALLERR_MASK) ) {
          psc->slaveStatus = SLAVE_STATUS_NACK;
          goto fail;
        }
        if( intr & I2C_ISR_STOPF_MASK  ) break;
      }
      psc->slaveStatus = SLAVE_STATUS_ACK;
      if(p->ISR & I2C_ISR_NACKF_MASK) psc->slaveStatus = SLAVE_STATUS_NACK;
    }
    p->ICR = p->ISR;

    result = 0;
    break;

  case  DEVI2C_GET_SLAVE_STATUS:
    result = psc->slaveStatus;
    break;
  }

fail:
  return result;
}


/*********************************
 * pio
 */
/**
  * @brief  send routine PIO
  * @param  psc   pointer of internal descriptor
  * @param  addr  slave address
  * @param  ptr   the pointer of send data
  * @param  size  send data size
  * @retval result  sent size
  */
static int
DevI2cSendPio(devI2cSc_t *psc, uint32_t addr, uint8_t *ptr, int size)
{
  stm32Dev_I2C          *p;

  int                   sz = -1;
  uint32_t              val;

  p = psc->dev;

  p->CR2 =  I2C_CR2_NBYTES_VAL(size) | I2C_CR2_SADD_VAL(addr<<1);

  /*p->CR2 |= I2C_CR2_START | I2C_CR2_STOP | I2C_CR2_AUTOEND | I2C_CR2_RELOAD;*/
  p->CR2 |= I2C_CR2_START;

  /* send the slave address */
  while((p->ISR & I2C_ISR_TXIS_MASK));
  if(p->ISR & I2C_ISR_NACKF_MASK) goto fail;


  /* send data */
  for(int i = 0; i < size; i++) {
    while(!(p->ISR & I2C_ISR_TXIS_MASK));
    p->TXDR = *ptr++;
  }
  while(!(p->ISR & I2C_ISR_TC_MASK));

  p->CR2 |= I2C_CR2_STOP;

  sz = size;

fail:
  return sz;
}
/**
  * @brief  recv routine PIO
  * @param  psc   pointer of internal descriptor
  * @param  addr  slave address
  * @param  ptr   the pointer of send data
  * @param  size  recv buffer size
  * @retval size  size of recv
  */
static int
DevI2cRecvPio(devI2cSc_t *psc, uint32_t addr, uint8_t *ptr, int size)
{
  stm32Dev_I2C          *p;

  int                   sz = 0, cntTx = 0, cntRx = 0;
  uint32_t              val;

  p = psc->dev;


fail:
  return sz;
}
/**
  * @brief  send routine PIO
  * @param  psc   pointer of internal descriptor
  * @param  addr  slave address
  * @param  ptr   the pointer of send data
  * @param  size  send data size
  * @retval result  sent size
  */
static int
DevI2cTransmitPio(devI2cSc_t *psc, devI2cPkt *pkt)
{
  stm32Dev_I2C          *p;
  uint8_t               *ptr;

  int                   sz = -1;
  uint32_t              val;
  int                   cntCmdSend;

  p = psc->dev;

  /* copy information */
  psc->pkt = *pkt;

  cntCmdSend = pkt->lenCmd + pkt->lenSend;

  if(!psc->param.intr) {
    /* send */
    if(cntCmdSend > 0) {
      p->CR2 = (I2C_CR2_NBYTES_VAL(cntCmdSend) |
                I2C_CR2_SADD_VAL(pkt->addr<<1));

      /* send the start condition */
      p->CR2 |= I2C_CR2_START;
      psc->seq = DEVI2C_SEQ_SEND_CMD;

      /* send the slave address */
      while((p->ISR & I2C_ISR_TXIS_MASK));
      if(p->ISR & I2C_ISR_NACKF_MASK) goto fail;

      /* send data */
      ptr = pkt->ptrCmd;
      for(int i = 0; i < pkt->lenCmd; i++) {
        while(!(p->ISR & I2C_ISR_TXIS_MASK));
        p->TXDR = *ptr++;
      }
      ptr = pkt->ptrSend;
      for(int i = 0; i < pkt->lenSend; i++) {
        while(!(p->ISR & I2C_ISR_TXIS_MASK));
        p->TXDR = *ptr++;
      }

      /* wait to finish sending */
      while(!(p->ISR & I2C_ISR_TC_MASK));

      sz = pkt->lenSend;
    }

    /* receive */
    if(pkt->lenRecv > 0) {
      p->CR2  = (I2C_CR2_NBYTES_VAL(pkt->lenRecv) |
                 I2C_CR2_SADD_VAL(pkt->addr<<1) |
                 I2C_CR2_RDWRN_RD);

      /* send the start or restart condition */
      p->CR2 |= I2C_CR2_START;

      /* send the slave address */
      while((p->ISR & I2C_ISR_TXIS_MASK));
      if(p->ISR & I2C_ISR_NACKF_MASK) goto fail;

      /* recv */
      ptr = pkt->ptrRecv;
      for(int i = 0; i < pkt->lenRecv; i++) {
        while(!(p->ISR & I2C_ISR_RXNE_MASK));
        *ptr++ = p->RXDR;
      }

      sz = pkt->lenRecv;
    }
    p->CR2 |= I2C_CR2_STOP;

  } else {      /* use interrupt */
    if(cntCmdSend) {
      if(pkt->lenCmd) {
        psc->ptr = psc->pkt.ptrCmd;
        psc->cnt = psc->pkt.lenCmd;
        psc->seq = DEVI2C_SEQ_SEND_CMD;
      } else {
        psc->ptr = psc->pkt.ptrSend;
        psc->cnt = psc->pkt.lenSend;
        psc->seq = DEVI2C_SEQ_SEND_DATA;
      }
      p->CR2 = (I2C_CR2_NBYTES_VAL(cntCmdSend) |
                I2C_CR2_SADD_VAL(pkt->addr<<1));

      /* send the start condition */
      p->CR2 |= I2C_CR2_START;
    } else if(pkt->lenRecv) {
      psc->ptr = psc->pkt.ptrRecv;
      psc->cnt = psc->pkt.lenRecv;
      psc->seq = DEVI2C_SEQ_RECV_DATA;

      p->CR2 = (I2C_CR2_NBYTES_VAL(pkt->lenRecv) |
                I2C_CR2_SADD_VAL(pkt->addr<<1) |
                I2C_CR2_RDWRN_RD);

      /* send the start condition */
      p->CR2 |= I2C_CR2_START;
    }

  }


fail:
  return sz;
}

/*********************************
 * dma
 */
const static uint8_t    devI2cSendDmaModuleTbl[] = DMAMUX_MODULE_I2CTX_TBL;
const static uint8_t    devI2cSendDmaReqTbl[]    = DMAMUX_REQ_I2CTX_TBL;
/**
  * @brief  send routine DMA
  * @param  psc   pointer of internal descriptor
  * @param  ptr   the pointer of send data
  * @param  size  send data size
  * @retval result  sent size
  */
static int
DevI2cSendDma(devI2cSc_t *psc, uint32_t addr, uint8_t *ptr, int size)
{
  stm32Dev_I2C          *p;
  int                   sz = 0;
  uint32_t              val;

  p = psc->dev;

  /* enable tx dma */

  /* start dma */
  {
    devDmaParam_t       param;

    memset(&param, 0, sizeof(param));
    param.req = devI2cSendDmaReqTbl[psc->unit];
    param.a = (void *)  ptr;
    param.b = (void *) &p->TXDR;
    param.cnt = size;
    param.dirBA = 0;
    param.aInc = 1;
    param.aSize = DEVDMA_SIZE_8BITS;
    param.bSize = DEVDMA_SIZE_8BITS;

    DevDmaInit((devI2cSendDmaModuleTbl[psc->unit]? 3: 1), 6, &param);
  }

fail:
  return sz;
}
const static uint8_t    devI2cRecvDmaModuleTbl[] = DMAMUX_MODULE_I2CRX_TBL;
const static uint8_t    devI2cRecvDmaReqTbl[]    = DMAMUX_REQ_I2CRX_TBL;
/**
  * @brief  recv routine DMA
  * @param  psc   pointer of internal descriptor
  * @param  ptr   the pointer of send data
  * @param  size  recv buffer size
  * @retval size  size of recv
  */
static int
DevI2cRecvDma(devI2cSc_t *psc, uint8_t *ptr, int size)
{
  stm32Dev_I2C          *p;
  int                   sz = 0;
  uint32_t              val;

  uint32_t              cfg1 = 0, cfg2 = 0;

  p = psc->dev;

  /* start dma */
  {
    devDmaParam_t       param;

    memset(&param, 0, sizeof(param));
    param.req = devI2cRecvDmaReqTbl[psc->unit];
    param.a = (void *)  ptr;
    param.b = (void *) &p->RXDR;
    param.cnt = size;
    param.dirBA = 1;
    param.aInc = 1;
    param.aSize = DEVDMA_SIZE_8BITS;
    param.bSize = DEVDMA_SIZE_8BITS;

    DevDmaInit((devI2cRecvDmaModuleTbl[psc->unit]? 3: 1), 7, &param);
  }
  /* enable rx dma */

fail:
  return sz;
}
/**
  * @brief  send routine PIO
  * @param  psc   pointer of internal descriptor
  * @param  addr  slave address
  * @param  ptr   the pointer of send data
  * @param  size  send data size
  * @retval result  sent size
  */
static int
DevI2cTransmitDma(devI2cSc_t *psc, devI2cPkt *pkt)
{
  stm32Dev_I2C          *p;
  uint8_t               *ptr;

  int                   sz = -1;
  uint32_t              val;
  int                   cnt;

  p = psc->dev;

  /* send */
  cnt = pkt->lenCmd + pkt->lenSend;
  if(cnt > 0) {
    p->CR2 = (I2C_CR2_NBYTES_VAL(cnt) |
              I2C_CR2_SADD_VAL(pkt->addr<<1));

    /* send the start condition */
    p->CR2 |= I2C_CR2_START;

    /* send the slave address */
    while((p->ISR & I2C_ISR_TXIS_MASK));
    if(p->ISR & I2C_ISR_NACKF_MASK) goto fail;

    /* send data */
    ptr = pkt->ptrCmd;
    for(int i = 0; i < pkt->lenCmd; i++) {
      while(!(p->ISR & I2C_ISR_TXIS_MASK));
      p->TXDR = *ptr++;
    }
    ptr = pkt->ptrSend;
    for(int i = 0; i < pkt->lenSend; i++) {
      while(!(p->ISR & I2C_ISR_TXIS_MASK));
      p->TXDR = *ptr++;
    }

    /* wait to finish sending */
    while(!(p->ISR & I2C_ISR_TC_MASK));

    sz = pkt->lenSend;
  }

  /* receive */
  cnt = pkt->lenRecv;
  if(cnt) {
    p->CR1 |= I2C_CR1_RXDMAEN_YES;
    p->CR2  = (I2C_CR2_NBYTES_VAL(cnt) |
               I2C_CR2_SADD_VAL(pkt->addr<<1) |
               I2C_CR2_AUTOEND_YES |
               I2C_CR2_RDWRN_RD);

    p->CR2 |= I2C_CR2_START;
    /* start dma */
    {
      devDmaParam_t       param;
      int               ch = 7;
      memset(&param, 0, sizeof(param));
      param.req = devI2cRecvDmaReqTbl[psc->unit];
      param.a = (void *)  pkt->ptrRecv;
      param.b = (void *) &p->RXDR;
      param.cnt = cnt;
      param.dirBA = 1;
      param.aInc = 1;
      param.aSize = DEVDMA_SIZE_8BITS;
      param.bSize = DEVDMA_SIZE_8BITS;

      DevDmaInit((devI2cSendDmaModuleTbl[psc->unit]? 3: 1), ch, &param);
      while(!DevDmaIsFinished((devI2cSendDmaModuleTbl[psc->unit]? 3: 1), ch));
    }

    sz = cnt;
  }

  p->CR2 |= I2C_CR2_STOP;

fail:
  return sz;
}
