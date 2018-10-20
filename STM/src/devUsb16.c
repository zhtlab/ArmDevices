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

#define _DEVUSB16_C_

#include        "config.h"

#include        "system.h"
#include        "stm32Usb16.h"

#include        "usb_def.h"
#include        "usbdif.h"

#include        "devUsb16.h"

struct _stDevUsb        devUsb;

#define sss 0

#if CONFIG_USE_RTOS
#define UsbdcoreCbBusState(unit, state) UsbdcoreCbIsr((unit), \
                     USBDCORECB_REQ_BUSSTATE, (state), USBDIF_NULL);
#define UsbdcoreCbSetup(unit, s) UsbdcoreCbIsr((unit),  \
                     USBDCORECB_REQ_SETUP, 0, (s));
#define UsbdcoreCbDataOut(unit, num, size) UsbdcoreCbIsr((unit), \
                     USBDCORECB_REQ_DATAOUT, (num), (void *)(size));
#define UsbdcoreCbDataInDone(unit, num)    UsbdcoreCbIsr((unit), \
                     USBDCORECB_REQ_DATAINDONE, (num), USBDIF_NULL)
#endif




int
DevUsbInit(int unit, devUsbParam_t *param)
{
  int           result = -1;
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;

  int                   i;
  uint32_t              cntr = 0;

  if(unit == -1) {
    memset(&devUsb, 0, sizeof(devUsb));

    devUsb.sc[1].dev = USB_FS_PTR;

    goto end;
  }

  psc = &devUsb.sc[unit];
  p   = psc->dev;
  psc->unit = unit;

  for(i = 0; i < USB_MAX_EPIN; i++) {
    psc->in[i].epnum  = 0x80 | i;
    psc->in[i].maxsize = 0x40;          /*adhoc */
  }
  for(i = 0; i < USB_MAX_EPOUT; i++) {
    psc->out[i].epnum =        i;
    psc->out[i].maxsize = 0x40;          /*adhoc */
  }

  /* enable usb module */
  p->CNTR = USB_CNTR_FRES;
  p->CNTR = 0;
  p->ISTR = 0;
  p->BTABLE = 0;
  DevUsb16SetAddress(p, 0);

  psc->cntr = USB_CNTR_CTR | USB_CNTR_RESET | USB_CNTR_WKUP | USB_CNTR_SUSP;
  //psc->cntr |= USB_CNTR_PMAOVR;
  if(param->sof) {
    psc->cntr |= USB_CNTR_SOF /*| USB_CNTR_ESOF*/;
  }
  p->CNTR   = psc->cntr;

  for(int i = 0; i < 8; i++) {
    p->EP[i].R &= USB_EP_TOGGLEBIT_MASK;
  }

  p->BCDR   = USB_BCDR_DPPU_YES;

end:
  return result;
}


int
DevUsbOpenEp(int unit, uint8_t epnum, int eptype, int size)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;
  uint32_t              type;
  uint16_t              val;

  int                   num;

  psc = &devUsb.sc[unit];
  p   = psc->dev;
  num = epnum & 0xf;
#if ((USBIF_EP_CTRL != (USB_EP_EP_TYPE_CTRL >> USB_EP_EP_TYPE_SHIFT)) || \
     (USBIF_EP_ISOC != (USB_EP_EP_TYPE_ISO  >> USB_EP_EP_TYPE_SHIFT)) || \
     (USBIF_EP_BULK != (USB_EP_EP_TYPE_BULK >> USB_EP_EP_TYPE_SHIFT)) || \
     (USBIF_EP_INTR != (USB_EP_EP_TYPE_INTR >> USB_EP_EP_TYPE_SHIFT)))
  switch(eptype) {
  case USBIF_EP_CTRL: type = USB_EP_EP_TYPE_CTRL; break;
  case USBIF_EP_ISOC: type = USB_EP_EP_TYPE_ISO;  break;
  case USBIF_EP_BULK: type = USB_EP_EP_TYPE_BULK; break;
  case USBIF_EP_INTR: type = USB_EP_EP_TYPE_INTR; break;
  }
#else
  type = eptype << USB_EPCTL_EPTYP_SHIFT;
#endif

  if(epnum & 0x80) {
    DevUsb16SetTxStatus(p, num, USB_EP_STAT_TX_DISABLED);
    p->EP[num].R = (type | USB_EP_EA_VAL(num));
    DevUsb16SetTxStatus(p, num, USB_EP_STAT_TX_NAK);
  } else {
    DevUsb16SetRxStatus(p, num, USB_EP_STAT_RX_DISABLED);
    p->EP[num].R = (type | USB_EP_EA_VAL(num));
    DevUsb16SetRxStatus(p, num, USB_EP_STAT_RX_VALID);
  }

  return 0;
}


int
DevUsbCloseEp(int unit, uint8_t epnum)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;
  uint32_t              type;

  int                   num;

  psc = &devUsb.sc[unit];
  p   = psc->dev;
  num = epnum & 0xf;

  DevUsb16SetTxStatus(p, num, USB_EP_STAT_TX_DISABLED);
  DevUsb16SetRxStatus(p, num, USB_EP_STAT_RX_DISABLED);
  p->EP[num].R = 0;

  return 0;
}


int
DevUsbTransmit(int unit, uint8_t epnum, const uint8_t *ptr, int size)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;

  int                   num;
  uint32_t              intr;

  uint16_t              *pBuf, *pSrc;
  uint16_t              stat;

  psc = &devUsb.sc[unit];
  p = psc->dev;
  num = epnum & 0x7f;

  psc->in[num].maxsize = 0x40;          /*adhoc */

  psc->in[num].epnum = epnum;
  psc->in[num].ptr = (uint8_t *)ptr;
  psc->in[num].size = size;
  psc->in[num].cnt = 0;
  psc->in[num].fSent = 0;

  DevUsbWritePacket(psc, epnum);

  return 0;
}


int
DevUsbPrepareReceive(int unit, uint8_t epnum, const uint8_t *ptr, int size)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;
  int                   num;

  psc = &devUsb.sc[unit];
  p   = psc->dev;

  num = epnum & 0x7f;

  psc->out[num].ptr  = (uint8_t *)ptr;
  psc->out[num].size = size;
  psc->out[num].cnt  = 0;

  //DevUsbStartPacketOut(psc, epnum);

  DevUsb16SetRxStatus(p, num, USB_EP_STAT_RX_VALID);

  return 0;
}


int
DevUsbSetStall(int unit, uint8_t epnum)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;


  return 0;
}


int
DevUsbSetAddress(int unit, int address)
{
  devUsbSc_t            *psc;

  psc = &devUsb.sc[unit];
  psc->addr = address;

  return 0;
}

void
DevUsbInterruptUsb1(void)
{
  DevUsbInterrupt(&devUsb.sc[1]);
  return;
}


static void
DevUsbInterrupt(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;
  uint32_t              intr;

  p = psc->dev;

  /* tx/rx interrupt */
  while((intr = p->ISTR) & USB_CNTR_CTR_MASK) {
    if(intr & USB_ISTR_DIR_MASK) {
      DevUsbInterruptEpOut(psc);
    } else {
      DevUsbInterruptEpInDone(psc);
    }
  }
  p->ISTR &= ~intr;


  /* SOF interrupt */
  if(intr & USB_CNTR_SOF_MASK) DevUsbInterruptSof(psc);


  /* resume Interrupt */
  if(intr & USB_CNTR_WKUP_MASK) {
    p->CNTR &= ~USB_CNTR_LPMODE_MASK;

    /* interrupt re-init */
    p->CNTR   = psc->cntr;

    /* Clear the Remote Wake-up Signaling */
    if(psc->lpmState == DEVUSB_LPM_L1) {
      psc->lpmState = DEVUSB_LPM_L0;
    } else {
      UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_RESUME);
      p->CNTR |= USB_CNTR_RESUME;
    }
    p->CNTR &= ~USB_CNTR_RESUME;
    p->CNTR &= ~USB_CNTR_WKUP_MASK;
  }


  /* LPM interrupt */
  if(intr & USB_CNTR_L1REQ_MASK) {
    printf("L1REQ \r\n");
    if(psc->lpmState == DEVUSB_LPM_L0) {
      psc->lpmState = DEVUSB_LPM_L1;
      /*  LPM_Callback(hpcd, LPM_L1_ACTIVE);  */  /* return immediatly */
    } else {
      UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_SUSPEND);
    }
  }


  /* suspend interrupt */
  if(intr & USB_CNTR_SUSP_MASK) {
    p->CNTR &= ~USB_CNTR_SUSP_MASK;
    p->CNTR |= USB_CNTR_FSUSP;
    p->CNTR |= USB_CNTR_LPMODE;

    UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_SUSPEND);
  }


  /* reset interrupt */
  if(intr & USB_CNTR_RESET_MASK) {
    UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_RESET);
    DevUsbInterruptEnumerate(psc);

    p->CNTR |=  USB_CNTR_SUSP;           /* enable the suspend interrupt */
  }


#if 0
  /* overflow interrupt */
  if(intr & USB_CNTR_PMAOVR_MASK) {
  }
#endif

fail:
  return;
}


static int
DevUsbInterruptEnumerate(devUsbSc_t *psc)
{
  int           unit;

  unit = psc->unit;

  psc->speed  = USB_SPEED_FULL;
  psc->ep0Mps = USB_FS_MAX_PACKET_SIZE;

  DevUsbCloseEp(unit,    0);
  DevUsbCloseEp(unit, 0x80);

  DevUsbOpenEp(unit,    0, USBIF_EP_CTRL, psc->out[0].maxsize);
  DevUsbOpenEp(unit, 0x80, USBIF_EP_CTRL, psc->in[0].maxsize);

  UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_ENUMERATED | psc->speed);
  return 0;
}
static void
DevUsbInterruptEpOut(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;

  uint32_t              epintr, epnum, epbit;
  int                   num;
  uint32_t              intr;
  __IO uint16_t         *regep;

  int                   size;
  uint8_t               *ptr;
  devUsbEp_t            *ep;

  p = psc->dev;

  intr = p->ISTR;
  num  = intr & USB_ISTR_EP_ID_MASK;
  regep= &p->EP[num].R;
  ep = &psc->out[num];
  size = USBSRAM_PTR->ep[num].rxCnt & USB_COUNT_RX_COUNT_MASK;
  ptr  = (uint8_t *)USBSRAM_PTR + USBSRAM_PTR->ep[num].rxAddr;

  DevUsb16ClearRxCtr(p, num);

  if(*regep & USB_EP_SETUP_MASK) {
    memcpy(&psc->setup, ptr, 8);
    if((psc->setup.bmRequest & USB_BMREQ_DIR_MASK)  /* IN and OUT w/o data */
       || psc->setup.wLength == 0) {
      UsbdcoreCbSetup(psc->unit, &psc->setup);
    } else {        /* OUT with data */
      DevUsbPrepareReceive(psc->unit, 0, psc->setup.buf, psc->setup.wLength);
      psc->waitSetupPayload = 1;
    }

  } else {
    if(num == 0) {
      if(psc->waitSetupPayload) {
        memcpy(&psc->setup.buf, ptr, size);
        psc->out[num].cnt += size;
        if(psc->out[num].size >= psc->out[num].cnt) {
          UsbdcoreCbSetup(psc->unit, &psc->setup);
          psc->waitSetupPayload = 0;
        }
      }
    } else {
      uint16_t  *pDst, *pSrc;
      pSrc = (uint16_t *)ptr;
      pDst = (uint16_t *)ep->ptr + ep->cnt;
      for(int i = 0; i < (size+1)/2; i++) *pDst++ = *pSrc++;
      psc->out[num].cnt += size;

      /* ep >= 1 && OUT */
      if((*regep & USB_EP_EP_TYPE_MASK) == USB_EP_EP_TYPE_ISO) {
          UsbdcoreCbDataOut(psc->unit, num, size);

      } else {
        if(ep->cnt < ep->maxsize) {
          UsbdcoreCbDataOut(psc->unit, num, size);
        } else if(ep->cnt >= ep->size) {
          UsbdcoreCbDataOut(psc->unit, num, ep->size);
        } else {
          /* continue to receive data */
          DevUsb16SetRxStatus(p, num, USB_EP_STAT_RX_VALID);
        }
      }
    }
  }

  return;
}


static void
DevUsbInterruptEpInDone(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;

  int                   num;
  uint32_t              intr;

  p = psc->dev;
  intr = p->ISTR;
  num  = intr & USB_ISTR_EP_ID_MASK;
  DevUsb16ClearTxCtr(p, num);

  if(psc->in[num].fSent) {
    UsbdcoreCbDataInDone(psc->unit, num | 0x80);
  } else {
    DevUsbWritePacket(psc, num | 0x80);
  }

  /* addres setting */
  if(psc->addr) {
    DevUsb16SetAddress(p, psc->addr);
    psc->addr = 0;

    DevUsbCloseEp(1,    0);
    DevUsbCloseEp(1, 0x80);
    DevUsbOpenEp(1,    0, USBIF_EP_CTRL, psc->out[0].maxsize);
    DevUsbOpenEp(1, 0x80, USBIF_EP_CTRL, psc->in[0].maxsize);
  }

  return;
}


static void
DevUsbInterruptSof(devUsbSc_t *psc)
{
  usbifSof_t  sof;
  static uint64_t       totalCount = 0;
  static uint32_t       cntSofPrev;
  uint32_t              val;
  int                   diff, diffSof = 0;

  val = TIM2_PTR->CCR4;
  diff = cntSofPrev - val;
  if(diff > 100000) {

    diff -= 196608;
    if(abs(diff) < 20) {
    diffSof = diff;
  }
  }
  cntSofPrev = val;

  sof.masterClk = 196608000;
  sof.diff = diffSof;
  totalCount++;
  sof.totalCount = totalCount;

  UsbifCbSof(psc->unit, &sof);

  return;
}



static int
DevUsbGetBusSpeed(devUsbSc_t *psc)
{
  int                   speed;
  stm32Dev_USB          *p;

  p = psc->dev;

  speed = USB_SPEED_FULL;

  return speed;
}


static void
DevUsbFlushFifoRx(devUsbSc_t *psc, int num)
{
  stm32Dev_USB          *p;
  p = psc->dev;

  return;
}
static void
DevUsbFlushFifoTx(devUsbSc_t *psc, int num)
{
  stm32Dev_USB          *p;
  p = psc->dev;

  return;
}
int
DevUsbSetTRxFifo(int unit, usbdifDevFifo_t *pFifo)
{
  int                   result = -1;
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;
  int                   pos = 0;
  stm32Dev_USB_SRAM_ep  *ramep;
  int                   size;

  pos = sizeof(stm32Dev_USB_SRAM_Header);

  ramep = &USBSRAM_PTR->ep[0];
  for(int i = 0; i <= USB_MAX_EPIN; i++) {
    if(pos > USBSRAM_SIZE) goto fail;
    size = pFifo->sizeTx[i];
    ramep->txAddr = pos;
    ramep->txCnt  = size;
    pos       += size;
    ramep++;
  }
  ramep = &USBSRAM_PTR->ep[0];
  for(int i = 0; i <= USB_MAX_EPOUT; i++) {
    if(pos > USBSRAM_SIZE) goto fail;
    size = pFifo->sizeRx[i];
    ramep->rxAddr = pos;
    if(size < 0x40) {
      ramep->rxCnt = USB_COUNT_RX_BLSIZE_NO  | USB_COUNT_RX_NUM_BLOCK2_VAL(size);
    } else {
      ramep->rxCnt = USB_COUNT_RX_BLSIZE_YES | USB_COUNT_RX_NUM_BLOCK32_VAL(size);
    }
    pos       += size;
    ramep++;
  }

  result = 0;

fail:
  return result;
}








#if 0
static int
DevUsbStartPacketOut(devUsbSc_t *psc, uint8_t epnum)
{
  stm32Dev_USB          *p;
  int                   size;
  int                   num;

  num = epnum & 0x7f;
  /*if(psc->in[num].fSent) goto end;*/

  p = psc->dev;

  if(size > psc->out[num].maxsize) size = psc->out[num].maxsize;



end:
  return 0;
}
#endif
#if 0
static int
DevUsbStartPacketIn(devUsbSc_t *psc, uint8_t epnum)
{
  stm32Dev_USB          *p;
  int                   num;

  num = epnum & 0x7f;
  if(psc->in[num].fSent) goto end;

  p = psc->dev;

  size = psc->in[num].size;

end:
  return 0;
}
#endif
static int
DevUsbWritePacket(devUsbSc_t *psc, uint8_t epnum)
{
  stm32Dev_USB          *p;

  int                   size;
  int                   num;
  uint32_t              intr;

  uint16_t              *pBuf, *pSrc;
  uint16_t              stat;

  p = psc->dev;
  num = epnum & 0x7f;
#if 0
  if(psc->in[num].fSent) {
    goto end;
  }
#endif
  //if(psc->in[num].fSent) goto end;
  size = psc->in[num].size;

  if(size > 0) {
    size -= psc->in[num].cnt;
    if(size > psc->in[num].maxsize) {
      size = psc->in[num].maxsize;

    } else {
      psc->in[num].fSent = 1;
    }

    if(size > 0) {
      uint8_t   *ptr;
      pBuf = (uint16_t *)USBSRAM_PTR + USBSRAM_PTR->ep[num].txAddr/2;
      pSrc = (uint16_t *)psc->in[num].ptr;
      for(int i = 0; i < (size+1)/2; i++) pBuf[i] = pSrc[i];
      stat = USB_EP_STAT_TX_VALID;

      psc->in[num].ptr += size;
      psc->in[num].cnt += size;
    }

  } else if(size == 0) {
    size = 0;
    stat = USB_EP_STAT_TX_VALID;
    psc->in[num].fSent = 1;

  } else {      /* size < 0 */
    size = 0;
    stat = USB_EP_STAT_TX_STALL;
    psc->in[num].fSent = 1;

  }

  USBSRAM_PTR->ep[num].txCnt = size;
  DevUsb16SetTxStatus(p, num, stat);

end:
  return 0;
}


static void
DevUsb16SetTxStatus(stm32Dev_USB *p, int num, uint16_t stat)
{
  uint16_t              val;

  val  = p->EP[num].R;
  val &= ~((USB_EP_DTOG_MASK) | (USB_EP_STAT_RX_VALID));
  val |= USB_EP_CTR_RX_MASK | USB_EP_CTR_TX_MASK;
  val ^= stat;
  p->EP[num].R = val;

  return;
}
static void
DevUsb16SetRxStatus(stm32Dev_USB *p, int num, uint16_t stat)
{
  uint16_t              val;

  val  = p->EP[num].R;
  val &= ~((USB_EP_DTOG_MASK) | (USB_EP_STAT_TX_VALID));
  val |= USB_EP_CTR_RX_MASK | USB_EP_CTR_TX_MASK;
  val ^= stat;
  p->EP[num].R = val;

  return;
}
static void
DevUsb16ClearTxCtr(stm32Dev_USB *p, int num)
{
  uint16_t              val;

  val  = p->EP[num].R;
  val &= ~(USB_EP_TOGGLEBIT_MASK | USB_EP_CTR_TX_MASK);
  val |= USB_EP_CTR_RX_MASK;
  p->EP[num].R = val;

  return;
}
static void
DevUsb16ClearRxCtr(stm32Dev_USB *p, int num)
{
  uint16_t              val;

  val  = p->EP[num].R;
  val &= ~(USB_EP_TOGGLEBIT_MASK | USB_EP_CTR_RX_MASK);
  val |= USB_EP_CTR_TX_MASK;
  p->EP[num].R = val;

  return;
}
static void
DevUsb16SetAddress(stm32Dev_USB *p, uint8_t addr)
{
  p->DADDR = USB_DADDR_EF_YES | USB_DADDR_ADD_VAL(addr);
  return;
}
