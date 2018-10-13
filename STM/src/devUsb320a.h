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

#ifndef _DEV_USB320A_H_
#define _DEV_USB320A_H_


#define STM320A_MAX_EPIN                8
#define STM320A_MAX_EPOUT               8

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
  uint8_t               epMax;
  uint16_t              ep0Mps;
  uint8_t               speed;

  devUsbEp_t            out[16];
  devUsbEp_t            in[16];
} devUsbSc_t;

struct _stDevUsb {
  devUsbSc_t            sc[3];
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


#ifdef  _DEV_USB320A_C_
static void     DevUsbInterrupt(devUsbSc_t *psc);
static int      DevUsbInterruptEnumulate(devUsbSc_t *psc);
static void     DevUsbInterruptEpOut(devUsbSc_t *psc);
static void     DevUsbInterruptEpIn(devUsbSc_t *psc);
static void     DevUsbInterruptSof(devUsbSc_t *psc);
static int      DevUsbInterruptRecvData(devUsbSc_t *psc);

static int      DevUsbResetModule(devUsbSc_t *psc);
static int      DevUsbResetPort(devUsbSc_t *psc);

static void     DevUsbSetTurnArroundTime(devUsbSc_t *psc);
static int      DevUsbGetBusSpeed(devUsbSc_t *psc);
static void     DevUsbFlushFifoRx(devUsbSc_t *psc, int num);
static void     DevUsbFlushFifoTx(devUsbSc_t *psc, int num);

static int      DevUsbEpOutEnable(devUsbSc_t *psc, uint8_t epnum, uint8_t *ptr);
static int      DevUsbSetSpeed(devUsbSc_t *psc, int speed);
static int      DevUsbStartPacketOut(devUsbSc_t *psc, uint8_t epnum);
static int      DevUsbStartPacketIn(devUsbSc_t *psc, uint8_t epnum);
static int      DevUsbWritePacket(devUsbSc_t *psc, uint8_t epnum);

static int      DevUsbDisconnect(devUsbSc_t *psc);
static int      DevUsbConnect(devUsbSc_t *psc);

#endif

void     DevUsbDebugShowGeneralReg(stm32Dev_USB *p, int num);

#endif
