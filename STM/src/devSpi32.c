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

#define _DEV_SPI32_C_

#include        <stdio.h>
#include        <string.h>

#include        "system.h"
#include        "devDma.h"

#include        "devSpi32.h"

struct _stSpi         spi;

/**
  * @brief  initilize the devcie
  * @param  unit   unit number
  * @param  param  the pointer of initialize parameters
  * @retval int  0: success
  */
int
DevSpiInit(int unit, devSpiParam_t *param)
{
  int                   result = -1;
  uint32_t              cfg1, cfg2;

  stm32Dev_SPI        *p;
  devSpiSc_t          *psc;

  if(unit == -1) {
    memset(&spi, 0, sizeof(spi));
#ifdef  SPI1_PTR
    spi.sc[1].dev = SPI1_PTR;
#endif
#ifdef  SPI2_PTR
    spi.sc[2].dev = SPI2_PTR;
#endif
#ifdef  SPI3_PTR
    spi.sc[3].dev = SPI3_PTR;
#endif
#ifdef  SPI4_PTR
    spi.sc[4].dev = SPI4_PTR;
#endif
#ifdef  SPI5_PTR
    spi.sc[5].dev = SPI5_PTR;
#endif
#ifdef  SPI6_PTR
    spi.sc[6].dev = SPI6_PTR;
#endif

    goto end;
  }

  if(unit > SPI_MODULE_COUNT) goto fail;
  psc = &spi.sc[unit];
  p = psc->dev;

  /* copy the parameters */
  psc->param = *param;

  /* clear first */
  p->CR1  = 0;
  p->CR2  = 0;
  p->CFG1 = 0;
  p->CFG2 = 0;

  p->CR1 |= SPI_CR1_SSI_YES;
  /* data bit */
  cfg1 |= ((psc->param.bit-1) << SPI_CFG1_DSIZE_SHIFT) & SPI_CFG1_DSIZE_MASK;
  cfg1 |= ((psc->param.prescaler) << SPI_CFG1_MBR_SHIFT) & SPI_CFG1_MBR_MASK;

  /* clk mode */
  cfg2 |= ((psc->param.clkmode << SPI_CFG2_CPHA_SHIFT) &
           (SPI_CFG2_CPOL_SHIFT | SPI_CFG2_CPHA_SHIFT));

  cfg2 |= SPI_CFG2_SSOM_YES | SPI_CFG2_SSM_YES | SPI_CFG2_MASTER_YES;


  p->CFG1 = cfg1;
  p->CFG2 = cfg2;               /* CFG2 must be set at a time */

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
DevSpiInterrupt(int unit)
{
  stm32Dev_SPI        *p;
  devSpiSc_t          *psc;

  if(unit > SPI_MODULE_COUNT) goto fail;
  psc = &spi.sc[unit];

  p = psc->dev;
fail:

  return;
}
/**
  * @brief  interrupt entry point for unit 4
  * @param  noen
  * @retval none
  */
void
DevSpi4Interrupt(void)
{
  DevSpiInterrupt(4);
}
/**
  * @brief  interrupt entry point for unit 5
  * @param  noen
  * @retval none
  */
void
DevSpi5Interrupt(void)
{
  DevSpiInterrupt(5);
}

/**
  * @brief  send routine
  * @param  unit  unit number
  * @param  ptr   the pointer of send data
  * @param  size  send data size
  * @retval result  success/fail
  */
int
DevSpiSend(int unit, uint8_t *ptr, int size)
{
  int                   result = -1;
  devSpiSc_t            *psc;

  if(unit > SPI_MODULE_COUNT) goto fail;
  psc = &spi.sc[unit];
  if(!psc->up) goto fail;

  if(psc->param.dmaTx) {
    result = DevSpiSendDma(psc, ptr, size);
  } else {
    result = DevSpiSendPio(psc, ptr, size);
  }

fail:
  return result;
}
/**
  * @brief  recv routine
  * @param  unit  unit number
  * @param  ptr   the pointer of send data
  * @param  size  recv buffer size
  * @retval size  size of recv
  */
int
DevSpiRecv(int unit, uint8_t *ptr, int size)
{
  int                   result = -1;
  devSpiSc_t            *psc;

  if(unit > SPI_MODULE_COUNT) goto fail;
  psc = &spi.sc[unit];
  if(!psc->up) goto fail;

  if(psc->param.dmaRx) {
    result = DevSpiRecvDma(psc, ptr, size);
  } else {
    result = DevSpiRecvPio(psc, ptr, size);
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
  * @param  ptr   the pointer of send data
  * @param  size  send data size
  * @retval result  sent size
  */
static int
DevSpiSendPio(devSpiSc_t *psc, uint8_t *ptr, int size)
{
  stm32Dev_SPI          *p;

  int                   sz = 0;
  uint32_t              val;

  p = psc->dev;

  /* automatic tx size is set 0 */
  p->CR2 = 0;
  /* change to the simplex transmit */
  val  = p->CFG2;
  val &= ~SPI_CFG2_COMM_MASK;
  val |=  SPI_CFG2_COMM_SIMPLEX_TX;
  p->CFG2 = val;
  /* enable serial */
  p->CR1 |= SPI_CR1_SPE_YES;
  /* set the freeruning mode */
  p->CR1 |= SPI_CR1_CSTART_YES;     /* can set only MASTER==1 && SPE==1 */
  while(size > 0) {
    if(p->SR & SPI_SR_TXP_MASK) {
      *(__IO uint8_t *)&p->TXDR = (uint8_t)*ptr++;
      size--;
    }
  }
  while(!(p->SR & SPI_SR_TXC_MASK));

  p->CR1 &= ~SPI_CR1_CSTART_MASK;
  p->CR1 &= ~SPI_CR1_SPE_MASK;

fail:
  return sz;
}
/**
  * @brief  recv routine PIO
  * @param  psc   pointer of internal descriptor
  * @param  ptr   the pointer of send data
  * @param  size  recv buffer size
  * @retval size  size of recv
  */
static int
DevSpiRecvPio(devSpiSc_t *psc, uint8_t *ptr, int size)
{
  stm32Dev_SPI          *p;

  int                   sz = 0, cntTx = 0, cntRx = 0;
  uint32_t              val;

  p = psc->dev;

  p->CR1 &= ~(SPI_CR1_SPE_MASK | SPI_CR1_CSTART_MASK);
  /* change to the simplex transmit */
  val  = p->CFG2;
  val &= ~SPI_CFG2_COMM_MASK;
  val |=  SPI_CFG2_COMM_FULL_DUP;
  p->CFG2 = val;
  /* set the recv count */
  /*p->CR2 = size;*/
  p->CR2 = 0;

  /* enable serial */
  p->CR1 |= SPI_CR1_SPE_YES;
  /* clear interrupt */
  p->IFCR = SPI_IFCR_CLEAR_ALL;
  /* set the freeruning mode */
  p->CR1 |= SPI_CR1_CSTART_YES;     /* can set only MASTER==1 && SPE==1 */

  while(size > cntTx) {
    while(!(p->SR & SPI_SR_TXP_MASK));
    *(__IO uint8_t *)&p->TXDR = 0x00;
    cntTx++;

    while(p->SR & SPI_SR_RXPCNT_MASK) {
      *ptr++ = *(__IO uint8_t *)&p->RXDR;
      cntRx++;
    }
  }

  if(cntRx != size) {
    while(!(p->SR & (SPI_SR_RXPCNT_MASK | SPI_SR_RXP_MASK)));
    *ptr++ = *(__IO uint8_t *)&p->RXDR;
    cntRx++;
  }

  p->CR1 &= ~SPI_CR1_CSTART_MASK;
  p->CR1 &= ~SPI_CR1_SPE_MASK;

fail:
  return sz;
}


/*********************************
 * dma
 */
const static uint8_t    devSpiSendDmaModuleTbl[] = DMAMUX_MODULE_SPITX_TBL;
const static uint8_t    devSpiSendDmaReqTbl[]    = DMAMUX_REQ_SPITX_TBL;
/**
  * @brief  send routine DMA
  * @param  psc   pointer of internal descriptor
  * @param  ptr   the pointer of send data
  * @param  size  send data size
  * @retval result  sent size
  */
static int
DevSpiSendDma(devSpiSc_t *psc, uint8_t *ptr, int size)
{
  stm32Dev_SPI          *p;
  int                   sz = 0;
  uint32_t              val;

  p = psc->dev;

  /* enable tx dma */
  p->CFG1 |= SPI_CFG1_TXDMAEN_YES;

  /* change to the simplex transmit */
  val  = p->CFG2;
  val &= ~SPI_CFG2_COMM_MASK;
  val |=  SPI_CFG2_COMM_SIMPLEX_TX;
  p->CFG2 = val;
  /* set the send count */
  p->CR2 = size;

  /* clean d cache */
  SCB_CleanDCache_by_Addr((unsigned long *)ptr, size+32);

  /* start dma */
  {
    devDmaParam_t       param;

    memset(&param, 0, sizeof(param));
    param.req = devSpiSendDmaReqTbl[psc->unit];
    param.a = (void *)  ptr;
    param.b = (void *) &p->TXDR;
    param.cnt = size;
    param.dirBA = 0;
    param.aInc = 1;
    param.aSize = DEVDMA_SIZE_8BITS;
    param.bSize = DEVDMA_SIZE_8BITS;

    DevDmaInit((devSpiSendDmaModuleTbl[psc->unit]? 3: 1), 6, &param);
  }

  /* enable serial */
  p->CR1 |= SPI_CR1_SPE_YES;
  /* clear ALL factors */
  p->IFCR = SPI_IFCR_CLEAR_ALL;

  /* set the freeruning mode */
  p->CR1 |= SPI_CR1_CSTART_YES;     /* can set only MASTER==1 && SPE==1 */

  /* wait complete transfer */
  while(!(p->SR & SPI_SR_EOT_MASK)) {
  }

  /*DevDmaStop((devSpiSendDmaModuleTbl[psc->unit]? 3: 1), 6);*/

  printf("size    %x   %x %x %x %x %x\r\n", size, p->CR1, p->CR2, p->CFG1, p->CFG2, p->SR);

  p->CFG1 &= ~(SPI_CFG1_TXDMAEN_MASK | SPI_CFG1_RXDMAEN_MASK);
  p->CR1  |=  SPI_CR1_CSUSP_YES;
  p->CR1  &= ~(SPI_CR1_SPE_MASK | SPI_CR1_CSTART_MASK);

fail:
  return sz;
}
const static uint8_t    devSpiRecvDmaModuleTbl[] = DMAMUX_MODULE_SPIRX_TBL;
const static uint8_t    devSpiRecvDmaReqTbl[]    = DMAMUX_REQ_SPIRX_TBL;
/**
  * @brief  recv routine DMA
  * @param  psc   pointer of internal descriptor
  * @param  ptr   the pointer of send data
  * @param  size  recv buffer size
  * @retval size  size of recv
  */
static int
DevSpiRecvDma(devSpiSc_t *psc, uint8_t *ptr, int size)
{
  stm32Dev_SPI          *p;
  int                   sz = 0;
  uint32_t              val;

  uint32_t              cfg1 = 0, cfg2 = 0;

  p = psc->dev;

  /* start dma */
  {
    devDmaParam_t       param;

    memset(&param, 0, sizeof(param));
    param.req = devSpiRecvDmaReqTbl[psc->unit];
    param.a = (void *)  ptr;
    param.b = (void *) &p->RXDR;
    param.cnt = size;
    param.dirBA = 1;
    param.aInc = 1;
    param.aSize = DEVDMA_SIZE_8BITS;
    param.bSize = DEVDMA_SIZE_8BITS;

    DevDmaInit((devSpiRecvDmaModuleTbl[psc->unit]? 3: 1), 7, &param);
  }

  /* enable rx dma */
  p->CFG1 &= ~(SPI_CFG1_TXDMAEN_MASK | SPI_CFG1_RXDMAEN_MASK);
  p->CFG1 |=   SPI_CFG1_RXDMAEN_YES;

  /* change to the simplex transmit */
  val  = p->CFG2;
  val &= ~SPI_CFG2_COMM_MASK;
  val |=  SPI_CFG2_COMM_SIMPLEX_RX;
  p->CFG2 = val;

  /* set the recv count */
  p->CR2 = size;

  /* enable serial */
  p->CR1 |= SPI_CR1_SPE_YES | SPI_CR1_MASRX_YES;
  /* clear EOT and etc */
  p->IFCR = SPI_IFCR_CLEAR_ALL;

  /* set the freeruning mode */
  p->CR1 |= SPI_CR1_CSTART_YES;     /* can set only MASTER==1 && SPE==1 */

  /* wait complete transfer */
  while(!(p->SR & SPI_SR_EOT_MASK)) {
  }

  DevDmaIsFinished((devSpiRecvDmaModuleTbl[psc->unit]? 3: 1), 7);

  /* clean d cache */
  SCB_InvalidateDCache_by_Addr((unsigned long *)ptr, size+32);

  /*DevDmaStop((devSpiSendDmaModuleTbl[psc->unit]? 3: 1), 7);*/

  p->CFG1 &= ~(SPI_CFG1_TXDMAEN_MASK | SPI_CFG1_RXDMAEN_MASK);
  p->CR1  &= ~(SPI_CR1_SPE_MASK | SPI_CR1_CSTART_MASK);

fail:
  return sz;
}
