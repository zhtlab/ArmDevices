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

#ifndef _DEV_SPI16_H_
#define _DEV_SPI16_H_


typedef struct {
  uint32_t              speed;
  uint32_t              clkmode;        /* clock mode 0, 1, 2, 3 */
  uint8_t               prescaler;      /* set to MBR 0=1/2, 1=1/4, 7=1/256*/
  uint8_t               bit;
  uint8_t               dmaRx:  1;      /* use dma */
  uint8_t               dmaTx:  1;      /* use dma */

  uint8_t               mode;
#define DEVSPI_MODE_PIO       0
#define DEVSPI_MODE_FIFO      1
#define DEVSPI_MODE_INTR      2
#define DEVSPI_MODE_DMA       3

} devSpiParam_t;



typedef struct {
  uint8_t               up;
  uint8_t               unit;
  devSpiParam_t         param;
  stm32Dev_SPI          *dev;

} devSpiSc_t;

struct _stSpi {
  devSpiSc_t          sc[SPI_MODULE_COUNT+1];
};


int             DevSpiInit(int unit, devSpiParam_t *param);
int             DevSpiLoop(int unit);
int             DevSpiSend(int unit, uint8_t *ptr, int size);
int             DevSpiRecv(int unit, uint8_t *ptr, int size);
void            DevSpi3Interrupt(void);


#ifdef  _DEV_SPI16_C_
static int      DevSpiSendFifo(devSpiSc_t *psc);
static int      DevSpiSendPio(devSpiSc_t *psc, uint8_t *ptr, int size);
static int      DevSpiRecvPio(devSpiSc_t *psc, uint8_t *ptr, int size);
static int      DevSpiSendDma(devSpiSc_t *psc, uint8_t *ptr, int size);
static int      DevSpiRecvDma(devSpiSc_t *psc, uint8_t *ptr, int size);
static void     DevSpiInterrupt(int unit);
#endif

#endif
