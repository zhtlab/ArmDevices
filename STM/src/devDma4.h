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

#ifndef _DEV_DMA_H_
#define _DEV_DMA_H_


typedef struct {
  uint32_t              req;            /* DMA request signal number */
  void                  *a;             /* memory */
  void                  *b;             /* peripheral or memeory */
  uint32_t              cnt;            /* transfer count */
#define DEVDMA_SIZE_8BITS       0
#define DEVDMA_SIZE_16BITS      1
#define DEVDMA_SIZE_32BITS      2

  uint32_t              dirBA : 1;       /* 0: a -> b, 1: b -> a */
  uint32_t              mem   : 1;       /* 0: mem to/from peri, 1: mem to/from mem */
  uint32_t              circ  : 1;       /* 0: normal, 1: circular mode */

  uint32_t              aInc  : 1;
  uint32_t              aSize : 2;

  uint32_t              bInc  : 1;
  uint32_t              bSize : 2;

  uint32_t              intrTC: 1;      /* transfer complete */
  uint32_t              intrHC: 1;      /* half transfer complite */
  uint32_t              intrER: 1;      /* error */
} devDmaParam_t;



typedef struct {
  uint8_t               up;
  uint8_t               unit;
  stm32Dev_DMA          *dev;
} devDmaSc_t;

struct _stDma {
  devDmaSc_t          sc[3];
};


int             DevDmaInit(int unit, int ch, devDmaParam_t *param);
int             DevDmaStart(int unit, int ch);
int             DevDmaStop(int unit, int ch);
int             DevDmaIsFinished(int unit, int ch);
void            DevDmaDebugShowRegs(int unit, int ch);


#ifdef  _DEV_DMA_C_
#endif

#endif
