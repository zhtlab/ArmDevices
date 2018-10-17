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

#ifndef _DEVUSB16_H_
#define _DEVUSB16_H_


typedef struct {
  uint8_t               dma: 1;
  uint8_t               sof: 1;
  uint8_t               vbus: 1;

} devUsbParam_t;

typedef struct {
  uint8_t       epnum;
#define DEVUSB_EPNUM_DIR_MASK   (1<<7)
#define DEVUSB_EPNUM_DIR_OUT    (0<<7)
#define DEVUSB_EPNUM_DIR_IN     (1<<7)
#define DEVUSB_EPNUM_NUM_MASK   (0x7f<<0)
  uint8_t       *ptr;
  uint8_t       fSent;
  uint32_t      size;           /* data size */
  uint32_t      cnt;            /* tx/rx count */
  uint32_t      maxsize;        /* max packet size */
} devUsbEp_t;

typedef struct {
  uint8_t               up: 1;
  uint8_t               dma: 1;
  uint8_t               waitSetupPayload: 1;

  uint8_t               unit;
  stm32Dev_USB          *dev;

  usbifSetup_t          setup;
  uint8_t               lpmState;
#define DEVUSB_LPM_L0           0
#define DEVUSB_LPM_L1           1
  uint8_t               epMax;
  uint16_t              ep0Mps;
  uint8_t               speed;
  uint8_t               addr;

  devUsbEp_t            out[USB_MAX_EPOUT];
  devUsbEp_t            in[USB_MAX_EPIN];
} devUsbSc_t;

struct _stDevUsb {
  devUsbSc_t            sc[2];
};


int             DevUsbInit(int unit, devUsbParam_t *param);

void            DevUsbInterruptUsb1(void);
void            DevUsbInterruptUsb2(void);

int             DevUsbOpenEp(int unit, uint8_t epnum, int type, int size);
int             DevUsbCloseEp(int unit, uint8_t epnum);

int             DevUsbTransmit(int unit, uint8_t epnum, const uint8_t *ptr, int size);
int             DevUsbPrepareReceive(int unit, uint8_t epnum, const uint8_t *ptr, int size);
int             DevUsbSetStall(int unit, uint8_t epnum);
int             DevUsbSetAddress(int unit, int address);

int             DevUsbSetTRxFifo(int unit, usbdifDevFifo_t *pFifo);

#ifdef  _DEVUSB16_C_

static void     DevUsbInterrupt(devUsbSc_t *psc);
static void     DevUsbInterruptEpOut(devUsbSc_t *psc);
static void     DevUsbInterruptEpInDone(devUsbSc_t *psc);
static void     DevUsbInterruptSof(devUsbSc_t *psc);

static int      DevUsbGetBusSpeed(devUsbSc_t *psc);
static void     DevUsbFlushFifoRx(devUsbSc_t *psc, int num);
static void     DevUsbFlushFifoTx(devUsbSc_t *psc, int num);


static void     DevUsb16SetTxStatus(stm32Dev_USB *p, int num, uint16_t stat);
static void     DevUsb16SetRxStatus(stm32Dev_USB *p, int num, uint16_t stat);

static void     DevUsb16ClearRxCtr(stm32Dev_USB *p, int num);
static void     DevUsb16ClearTxCtr(stm32Dev_USB *p, int num);

static int      DevUsbWritePacket(devUsbSc_t *psc, uint8_t epnum);

#endif

#endif
