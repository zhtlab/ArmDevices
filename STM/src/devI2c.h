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

#ifndef _DEV_I2C_H_
#define _DEV_I2C_H_

typedef struct {
  uint32_t      addr;
  uint8_t       *ptrCmd;
  uint16_t      lenCmd;
  uint8_t       *ptrSend;
  uint16_t      lenSend;
  uint8_t       *ptrRecv;
  uint16_t      lenRecv;
} devI2cPkt;

typedef struct {

  uint32_t              speed;
#define DEVI2C_SPEED_100KHZ             100000
#define DEVI2C_SPEED_400KHZ             400000
#define DEVI2C_SPEED_1000KHZ            1000000

  uint8_t               dma:    1;      /* use dma */
  uint8_t               intr:   1;

  uint8_t               mode;   /* adhoc */
#define DEVI2C_MODE_PIO       0
  //#define DEVI2C_MODE_FIFO      1     /* non fifo */
#define DEVI2C_MODE_INTR      2
#define DEVI2C_MODE_DMA       3

} devI2cParam_t;


typedef struct {
  uint8_t               up;
  uint8_t               unit;
  uint8_t               seq;
#define DEVI2C_SEQ_IDLE         0
#define DEVI2C_SEQ_SEND_ADDR    1
#define DEVI2C_SEQ_SEND_CMD     2
#define DEVI2C_SEQ_SEND_DATA    3
#define DEVI2C_SEQ_RECV_RESTART 4
#define DEVI2C_SEQ_RECV_ADDR    5
#define DEVI2C_SEQ_RECV_DATA    6
#define DEVI2C_SEQ_SLAVE_RECV_ADDR    11
#define DEVI2C_SEQ_SLAVE_RECV_CMD     12
#define DEVI2C_SEQ_SLAVE_RECV_DATA    13
#define DEVI2C_SEQ_SLAVE_SEND_ADDR    14
#define DEVI2C_SEQ_SLAVE_SEND_DATA    15
#define DEVI2C_SEQ_STAT_NACK_MASK       0x80
  devI2cPkt             pkt;
  devI2cParam_t         param;
  stm32Dev_I2C          *dev;

  uint8_t               *ptr;
  int                   cnt;

} devI2cSc_t;

struct _stI2c {
  devI2cSc_t          sc[7];
};



int                     DevI2cInit(int unit, devI2cParam_t *param);
int                     DevI2cRecv(int unit, uint32_t addr, uint8_t *ptr, int size);
int                     DevI2cSend(int unit, uint32_t addr, uint8_t *ptr, int size);
int                     DevI2cTransmit(int unit, devI2cPkt *p);


#ifdef  _DEV_I2C_C_
static int              DevI2cSendPio(devI2cSc_t *psc, uint32_t addr, uint8_t *ptr, int size);
static int              DevI2cRecvPio(devI2cSc_t *psc, uint32_t addr, uint8_t *ptr, int size);
static int              DevI2cTransmitPio(devI2cSc_t *psc, devI2cPkt *p);
static int              DevI2cSendDma(devI2cSc_t *psc, uint32_t addr, uint8_t *ptr, int size);
static int              DevI2cRecvDma(devI2cSc_t *psc, uint8_t *ptr, int size);
static int              DevI2cTransmitDma(devI2cSc_t *psc, devI2cPkt *pkt);

#endif

#endif
