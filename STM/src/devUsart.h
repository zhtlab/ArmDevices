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

#ifndef _DEV_USART_H_
#define _DEV_USART_H_


typedef struct {
  uint32_t              baud;
  uint8_t               bit;
#define DEVUSART_BIT_7          7
#define DEVUSART_BIT_8          8
#define DEVUSART_BIT_9          9
  uint8_t               stop;
#define DEVUSART_STOP_1         1
#define DEVUSART_STOP_2         2
#define DEVUSART_STOP_1_5       3
  uint8_t               parity;
#define DEVUSART_PARITY_NONE    0
#define DEVUSART_PARITY_EVEN    2
#define DEVUSART_PARITY_ODD     3
#define DEVUSART_PARITY_MASK    2

  uint8_t               mode;
#define DEVUSART_MODE_PIO       0
#define DEVUSART_MODE_FIFO      1
#define DEVUSART_MODE_INTR      2
#define DEVUSART_MODE_DMA       3
  uint8_t               szFifoTx;       /* 2 ^ n */
  uint8_t               szFifoRx;       /* 2 ^ n */
} devUsartParam_t;



typedef struct {
  uint8_t               up;
  uint8_t               unit;
  devUsartParam_t       param;
  stm32Dev_USART        *dev;
  int                   dFifoTx;
  int                   dFifoRx;
} devUsartSc_t;

struct _stUsart {
  devUsartSc_t          sc[9];
};


int             DevUsartInit(int unit, devUsartParam_t *param);
int             DevUsartLoop(int unit);
int             DevUsartSend(int unit, uint8_t *ptr, int size);
int             DevUsartRecv(int unit, uint8_t *ptr, int size);
int             DevUsartGetDataLen(int unit);
void            DevUsart2Interrupt(void);
void            DevUsart3Interrupt(void);


#ifdef  _DEV_USART_C_
static int      DevUsartSendPio(devUsartSc_t *psc, uint8_t *ptr, int size);
static int      DevUsartSendFifo(devUsartSc_t *psc);
static void     DevUsartInterrupt(int unit);
#endif

#endif
