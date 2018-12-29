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

#define _DEV_USB320A_C_

#include        <stdio.h>

#include        "rtos.h"

#include        "config.h"
#include        "system.h"
#include        "usb_def.h"
#include        "usbdif.h"
#include        "usbdcore.h"
#include        "devGpio.h"

#include        "devUsb320a.h"

#define DEVUSB_INCOMPLETE_ISO_INTR_EN           0


struct _stDevUsb           devUsb;


#define LPM_L0          0
#define LPM_L1          1
#define USBD_HS_TRDT_VALUE      9
#if 1
/*       adhoc   it also defined in usbdcore.c */
usbdifStatus_t
UsbdevInit(int dev, usbdifInitParam_t *pUsbInit)
{
  return 0;
}
usbdifStatus_t
UsbdevStart(int dev)
{
  return 0;
}
#endif

int
DevUsbInit(int unit, devUsbParam_t *param)
{
  int           result = -1;

  devUsbSc_t            *psc;
  stm32Dev_USB          *p;

  int                   i;

  if(unit == -1) {
    memset(&devUsb, 0, sizeof(devUsb));

    devUsb.sc[1].dev = USB1_HS;
    devUsb.sc[2].dev = USB2_FS;
    devUsb.sc[0].dev = USB2_FS;

    goto end;
  }

  psc = &devUsb.sc[unit];
  p   = psc->dev;
  psc->unit = unit;

  printf("# USBOTG Product ID: %x, Core ID: %x\r\n", p->CID, p->GSNPSID);


  p->GUSBCFG |= USB_GUSBCFG_PHYSEL_YES;

  /* reset the module */
  DevUsbResetModule(psc);


  /* deactivate the power down mode */
  p->GCCFG = USB_GCCFG_PWRDWN_YES;

  if(param->dma) {
    psc->dma = 1;
    p->GAHBCFG |= USB_GAHBCFG_HBSTLEN_4WORDS;
    p->GAHBCFG |= USB_GAHBCFG_DMAEN_YES;
  }

  /* the mode is chagned to the device */
  p->GUSBCFG &= ~(USB_GUSBCFG_FHMOD_MASK | USB_GUSBCFG_FDMOD_MASK);
  p->GUSBCFG |=   USB_GUSBCFG_FDMOD_YES;


  for(i = 0; i < 15; i++) {             /* adhoc */
    psc->in[i].epnum  = 0x80 | i;       /* adhoc */
    psc->in[i].maxsize = 0x40;          /*adhoc */
    psc->out[i].epnum =        i;
    psc->out[i].maxsize = 0x40;          /*adhoc */
    p->DIEPTXF[i] = 0;
  }


  /* VBUS sensing */
  if(0) {
    p->GCCFG |= USB_GCCFG_VBDEN_YES;
  } else {
    p->GCCFG &= ~USB_GCCFG_VBDEN_MASK;
    p->GOTGCTL |= USB_GOTGCTL_BVALOEN_YES;
    p->GOTGCTL |= USB_GOTGCTL_BVALOVAL_YES;
  }

  /* phy clock restarting */
  p->PCGCCTL = 0;

  /* Device mode configuration */
  /*p->DCFG |= DCFG_FRAME_INTERVAL_80;          defined 0 */

  /* device speed */
  /*p->DCFG |= USB_SPEED_FULL;*/
  p->DCFG |= USB_DCFG_DSPD_FS;

  /* flush all fifos */
  DevUsbFlushFifoRx(psc, 0x10);
  DevUsbFlushFifoTx(psc, 0x10);

  /* disable and clear all interrupts */
  p->DIEPMSK =  0;
  p->DOEPMSK =  0;
  p->DAINT   = -1;
  p->DAINTMSK = 0;

  for (i = 0; i < USB_EPIN_COUNTS; i++) {
    if((p->in[i].CTL & USB_EPCTL_EPENA_MASK) == USB_EPCTL_EPENA_YES) {
      p->in[i].CTL = (USB_EPCTL_EPDIS_YES | USB_EPCTL_SNAK);
    } else {
      p->in[i].CTL = 0;
    }

    p->in[i].SIZ = 0;
    p->in[i].INT = -1;
  }

  for (i = 0; i < USB_EPOUT_COUNTS; i++) {
    if((p->out[i].CTL & USB_EPCTL_EPENA_MASK) == USB_EPCTL_EPENA_YES) {
      p->out[i].CTL = (USB_EPCTL_EPDIS_YES | USB_EPCTL_SNAK);
    } else {
      p->out[i].CTL = 0;
    }

    p->out[i].SIZ = 0;
    p->out[i].INT = -1;
  }

  /*p->DIEPMSK &= ~(USB_EOEPMSK_TXFURM_MASK);*/

  if(param->dma) {
    p->DTHRCTL = (USB_DTHRCTL_TXTHRLEN_VAL(0x100) |
                  USB_DTHRCTL_RXTHRLEN_VAL(0x100));
    p->DTHRCTL |= (USB_DTHRCTL_RXTHREN_YES | USB_DTHRCTL_ISOTHREN_YES |
                   USB_DTHRCTL_NONISOTHREN_YES | 0x08000000);

    p->DTHRCTL;         /* just read */
  }

  /* disable all interrupts and clear the status */
  p->GINTMSK = 0;
  p->GINTSTS = -1;

  /* enable interrupt for to manage the usb device behavior */
  p->GINTMSK = (USB_GINTSTS_USBSUSP_EN | USB_GINTSTS_USBRST_EN | \
                USB_GINTSTS_ENUMDNE_EN | USB_GINTSTS_IEPINT_EN | \
                USB_GINTSTS_OEPINT_EN  |
#if DEVUSB_INCOMPLETE_ISO_INTR_EN
                USB_GINTSTS_IISOIXFR_EN| USB_GINTSTS_PXFRM_IISOOXFR_EN |
#endif
                USB_GINTSTS_WKUINT_EN);
  /* enable receive fifo not empty interrupt */
  if(!param->dma) p->GINTMSK |= USB_GINTSTS_RXFLVL_EN;
  if(param->sof)  p->GINTMSK |= USB_GINTSTS_SOF_EN;
  if(param->vbus) p->GINTMSK |= (USB_GINTSTS_SRQINT_EN |
                                 USB_GINTSTS_OTGINT_EN);
  /* un-mask the global interrupt enable */
  p->GAHBCFG |= USB_GAHBCFG_GINTMSK_YES;

  DevUsbConnect(psc);

end:
  result = 0;
fail:
  return result;
}


int
DevUsbOpenEp(int unit, uint8_t epnum, int eptype, int size)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;

  int                   num;
  uint32_t              type;

  psc = &devUsb.sc[unit];
  p   = psc->dev;
  num = epnum & 0x7f;

#if ((USBIF_EP_CTRL != (USB_EPCTL_EPTYP_CTRL >> USB_EPCTL_EPTYP_SHIFT)) || \
     (USBIF_EP_ISOC != (USB_EPCTL_EPTYP_ISOC >> USB_EPCTL_EPTYP_SHIFT)) || \
     (USBIF_EP_BULK != (USB_EPCTL_EPTYP_BULK >> USB_EPCTL_EPTYP_SHIFT)) || \
     (USBIF_EP_INTR != (USB_EPCTL_EPTYP_INTR >> USB_EPCTL_EPTYP_SHIFT)))
  switch(eptype) {
  case USBIF_EP_CTRL: type = USB_EPCTL_EPTYP_CTRL; break;
  case USBIF_EP_ISOC: type = USB_EPCTL_EPTYP_ISOC; break;
  case USBIF_EP_BULK: type = USB_EPCTL_EPTYP_BULK; break;
  case USBIF_EP_INTR: type = USB_EPCTL_EPTYP_INTR; break;
  }
#else
  type = eptype << USB_EPCTL_EPTYP_SHIFT;
#endif

  if(epnum & DEVUSB_EPNUM_DIR_MASK) {   /* IN */
    p->in[num].INT = -1;
    p->in[num].CTL |= ((USB_EPCTL_SD0PID_SEVNFRM) |
                       type |
                       USB_EPCTL_TXFNUM_VAL(num) |
                       (USB_EPCTL_USBAEP_YES) |
                       size);
    p->DAINTMSK |= USB_DAINT_IEPINT_BIT(num);
  } else {                              /* OUT */
    p->out[num].INT = -1;
    p->out[num].CTL |= ( (USB_EPCTL_SD0PID_SEVNFRM) |
                         type |
                         (USB_EPCTL_USBAEP_YES) |
                         size);
    p->DAINTMSK |= USB_DAINT_OEPINT_BIT(num);
  }

  return 0;
}


int
DevUsbCloseEp(int unit, uint8_t epnum)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;

  int                   num;

  psc = &devUsb.sc[unit];
  p   = psc->dev;
  num = epnum & 0x7f;

  if(epnum & DEVUSB_EPNUM_DIR_MASK) {   /* IN */
    p->DAINTMSK &= ~USB_DAINT_IEPINT_BIT(num);
    p->in[num].CTL = 0;

  } else {                              /* OUT */
    p->DAINTMSK &= ~USB_DAINT_OEPINT_BIT(num);
    p->out[num].CTL = 0;
  }

  return 0;
}


int
DevUsbTransmit(int unit, uint8_t epnum, const uint8_t *ptr, int size)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;

  int                   num, len;
  uint32_t              ctl, siz;

  /* adhoc check paramters */

  psc = &devUsb.sc[unit];
  p   = psc->dev;

  num = epnum & 0xf;

  psc->in[num].maxsize = 0x40;          /*adhoc */

  psc->in[num].epnum = epnum;
  psc->in[num].ptr = (uint8_t *)ptr;
  psc->in[num].size = size;
  psc->in[num].cnt = 0;
  psc->in[num].fSent = 0;

  DevUsbStartPacketIn(psc, epnum);
  DevUsbWritePacket(psc, epnum);

  return 0;
}

int
DevUsbPrepareReceive(int unit, uint8_t epnum, const uint8_t *ptr, int size)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;

  int                   num;
  uint32_t              val;

  psc = &devUsb.sc[unit];
  p   = psc->dev;

  num = epnum & 0xf;

  psc->out[num].ptr  = (uint8_t *)ptr;
  psc->out[num].size = size;
  psc->out[num].cnt  = 0;

  DevUsbStartPacketOut(psc, epnum);

  return 0;
}


int
DevUsbSetStall(int unit, uint8_t epnum)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;

  int                   num;
  uint32_t              val;

  psc = &devUsb.sc[unit];
  p   = psc->dev;

  num = epnum & 0xf;

  p->out[num].CTL |= (1<<num);

  return 0;
}


int
DevUsbSetAddress(int unit, int address)
{
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;

  psc = &devUsb.sc[unit];
  p   = psc->dev;

  p->DCFG |= (address << USB_DCFG_DAD_SHIFT) & USB_DCFG_DAD_MASK;

  return 0;
}



void
DevUsbInterruptUsb1(void)
{
  DevUsbInterrupt(&devUsb.sc[1]);
  return;
}
void
DevUsbInterruptUsb2(void)
{
  DevUsbInterrupt(&devUsb.sc[0]);               /* adhoc the actual number is 2 */
  return;
}




static void
DevUsbInterrupt(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;

  uint32_t              i = 0, ep_intr = 0, epint = 0, epnum = 0;
  uint32_t              epbit = 0;
  uint32_t              temp = 0;

  uint32_t              intr;

  uint32_t              val;

  p    = psc->dev;

  intr  = p->GINTSTS;
  p->GINTSTS |= intr;            /* clear interrupt */
  intr &= p->GINTMSK;

  if(!intr) goto fail;

  /* ep out/in interrupt */
  if(intr & USB_GINTSTS_OEPINT_MASK) DevUsbInterruptEpOut(psc);
  if(intr & USB_GINTSTS_IEPINT_MASK) DevUsbInterruptEpIn(psc);

  /* resume Interrupt */
  if(intr & USB_GINTSTS_WKUINT_MASK) {
    /* Clear the Remote Wake-up Signaling */
    p->DCTL &= ~USB_DCTL_RWUSIG_MASK;

    if(psc->lpmState == LPM_L1) {
      psc->lpmState = LPM_L0;
      /* LPM_Callback(hpcd, LPM_L0_ACTIVE);*/  /* return immediatly */
    } else {
      p->PCGCCTL &= ~USB_PCGCCTL_STOPCLK_MASK;
      UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_RESUME);
    }
  }


  /* suspend interrupt */
  if(intr & USB_GINTSTS_USBSUSP_MASK) {
    if((p->DSTS & USB_DSTS_SUSPSTS_MASK) == USB_DSTS_SUSPSTS_YES) {

      p->PCGCCTL |= USB_PCGCCTL_STOPCLK_YES;
      UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_SUSPEND);
    }
  }


  /* SOF interrupt */
  if(intr & USB_GINTSTS_SOF_MASK) DevUsbInterruptSof(psc);

  /* Handle RxQLevel Interrupt */
  if(intr & USB_GINTSTS_RXFLVL_MASK) DevUsbInterruptRecvData(psc);

  /* enumeration interruption done Interrupt */
  if(intr & USB_GINTSTS_ENUMDNE_MASK) {
    DevUsbInterruptEnumerate(psc);
  }


  /* LPM interrupt */
  if(intr & USB_GINTSTS_LPMINT_MASK) {
    if(psc->lpmState == LPM_L0) {
      psc->lpmState = LPM_L1;
      /*LPM_Callback(hpcd, PCD_LPM_L1_ACTIVE);*/  /* return immediatly */
    } else {
      UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_SUSPEND);
    }
  }


  /* reset interrupt */
  if(intr & USB_GINTSTS_USBRST_MASK) {
    UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_RESET);

    p->DCTL &= ~USB_DCTL_RWUSIG_MASK;

    /* clear all epin fifos */
    DevUsbFlushFifoTx(psc, USB_GRSTCTL_TXFNUM_ALL >> USB_GRSTCTL_TXFNUM_SHIFT);

    for (i = 0; i < USB_EPIN_COUNTS; i++) {
      p->in[i].INT  = -1;
    }
    for (i = 0; i < USB_EPOUT_COUNTS; i++) {
      p->out[i].INT = -1;
    }
    p->DAINT = -1;
    p->DAINTMSK |=  USB_DAINT_OEPINT_BIT(0) |  USB_DAINT_IEPINT_BIT(0);

    p->DOEPMSK |= (USB_DOEPMSK_STUPM_EN | USB_DOEPMSK_XFRCM_EN | USB_DOEPMSK_EPDM_EN);
    p->DIEPMSK |= (USB_DIEPMSK_TOM_EN   | USB_DIEPMSK_XFRCM_EN | USB_DIEPMSK_EPDM_EN);

    /* Set Default Address to 0 */
    p->DCFG &= ~USB_DCFG_DAD_MASK;

#if 0
    /* setup EP0 to receive SETUP packets */
    DevUsbEpOutEnable(psc, 0, (uint8_t *)&psc->setup);
#endif
  }


  /* connect interrupt */
  if(intr & USB_GINTSTS_SRQINT_MASK) {
    UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_CONNECT);
  }


  /* disconnect interrupt */
  if(intr & USB_GINTSTS_OTGINT_MASK) {
    temp = p->GOTGINT;

    if(temp & USB_GOTGINT_SEDET_MASK) {
      UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_DISCONNECT);
    }
    p->GOTGINT |= temp;
  }

#if DEVUSB_INCOMPLETE_ISO_INTR_EN
    /* Handle Incomplete ISO IN Interrupt */
  if(intr & USB_GINTSTS_IISOIXFR_MASK) {
    printf("devusb intr incomplete isoin\r\n");
    /*xxx_IsoInIncompleteCallback(hpcd, epnum);*/
  }

  /* Handle Incomplete ISO OUT Interrupt */
  if(intr & USB_GINTSTS_PXFRM_IISOOXFR_MASK) {
    printf("devusb intr incomplete isoout\r\n");
    /*xxx_IsoOutIncompleteCallback(hpcd, epnum);*/
  }
#endif

fail:
  return;
}


int
DevUsbSetTRxFifo(int unit, usbdifDevFifo_t *pFifo)
{
  int                   result = -1;
  devUsbSc_t            *psc;
  stm32Dev_USB          *p;
  int                   size, *pSize;
  int                   offset = 0;

  psc = &devUsb.sc[unit];
  p   = psc->dev;

  /* rx fifo */
  size   = pFifo->sizeRx[0];
  size >>= 2;
  p->GRXFSIZ = size & USB_GRXSIZ_RXFD_MASK;
  offset += size;

  /* tx ep0 fifo */
  size   = pFifo->sizeTx[0];
  size >>= 2;
  p->DIEPTXF0_HNPTXFSIZ = (((size << USB_DIEPTXF_FD_SHIFT) & USB_DIEPTXF_FD_MASK) |
                           (offset & USB_DIEPTXF_SA_MASK));
  offset += size;

  pSize = &pFifo->sizeTx[1];
  for(int i = 0; i < STM320A_MAX_EPOUT-1; i++) {
    size   = *pSize++;
    size >>= 2;
    p->DIEPTXF[i] = (((size << USB_DIEPTXF_FD_SHIFT) & USB_DIEPTXF_FD_MASK) |
                     (offset & USB_DIEPTXF_SA_MASK));
    offset += size;
  }
  result = 0;

  return result;
}


static int
DevUsbInterruptEnumerate(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;

  p = psc->dev;

  /* enable setup interface */
  p->in[0].CTL &= ~USB_EPCTL_MPSIZ_MASK;
#if 0
  if((p->DSTS & USB_DSTS_ENUMSPD_MASK) == USB_DSTS_ENUMSPD_LS_PHY_6MHZ) {
    p->in[0].CTL |= 3;
  }
#endif
  p->DCTL |= USB_DCTL_CGINAK_MASK;

  p->GUSBCFG &= ~USB_GUSBCFG_TRDT_MASK;

  if(DevUsbGetBusSpeed(psc) == USB_SPEED_HIGH) {
    psc->speed  = USB_SPEED_HIGH;
    psc->ep0Mps = USB_HS_MAX_PACKET_SIZE;
    p->GUSBCFG |= (USBD_HS_TRDT_VALUE << USB_GUSBCFG_TRDT_SHIFT) & USB_GUSBCFG_TRDT_MASK;

  } else {
    psc->speed  = USB_SPEED_FULL;
    psc->ep0Mps = USB_FS_MAX_PACKET_SIZE;

    DevUsbSetTurnArroundTime(psc);
  }

  DevUsbOpenEp(psc->unit, DEVUSB_EPNUM_DIR_OUT | 0, USBIF_EP_CTRL, psc->ep0Mps);
  DevUsbOpenEp(psc->unit, DEVUSB_EPNUM_DIR_IN  | 0, USBIF_EP_CTRL, psc->ep0Mps);

#if 0
  /* disconnect */
  p->DCTL |= USB_DCTL_SDIS_DISCONNECT;
  /*p->DCTL &= ~USB_DCTL_SDIS_MASK;*/
#endif

  UsbdcoreCbBusState(psc->unit, USBDIF_BUSSTATE_ENUMERATED | psc->speed);


  return 0;
}


static void
DevUsbInterruptEpOut(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;

  uint32_t              epintr, epnum, epbit;

  p = psc->dev;

  epnum   = 0;
  epbit   = p->DAINT & USB_DAINT_OEPINT_MASK;  /* MSB 16bit is ep out */
  epbit >>= USB_DAINT_OEPINT_SHIFT;

  while(epbit) {
    if(epbit & 1) {

      epintr = p->out[epnum].INT;
      p->out[epnum].INT = epintr;       /* clear interrupt factors */

      /* data receive */
      if(epintr & USB_EPINT_XFRC_MASK) {
        int           size;
        size = (p->out[epnum].SIZ & USB_EPSIZ_XFRSIZ_MASK) >> USB_EPSIZ_XFRSIZ_SHIFT;

        if(psc->dma) {
          psc->out[epnum].size = (psc->out[epnum].maxsize - size);
          psc->out[epnum].ptr += psc->out[epnum].maxsize;

          UsbdcoreCbDataOut(psc->unit, epnum, psc->out[epnum].size);

          if(epnum == 0) {
            if(psc->out[epnum].size == 0) {
#if 0
              /* this is ZLP, so prepare EP0 for next setup */
              DevUsbEpOutEnable(psc, 0, (uint8_t *)&psc->setup);
#endif
            } else {
              if(psc->waitSetupPayload) {
                DevUsbTransmit(psc->unit, epnum, NULL, 0);
                UsbdcoreCbSetup(psc->unit, &psc->setup);
                psc->waitSetupPayload = 0;
              }
            }
          }
        } else {
          if(epnum == 0) {
            if(psc->waitSetupPayload) {
              UsbdcoreCbSetup(psc->unit, &psc->setup);
              psc->waitSetupPayload = 0;
            } else {
              /*UsbdcoreCbDataOut(psc->unit, epnum, size);*/
            }
          }
        }
      }


      /* setup patcket is received
       * skip calling upper later, if the SETUP OUT and wLength is not zero
       */
      if(epintr & USB_EPINT_STUP_MASK)  {
        if((psc->setup.bmRequest & USB_BMREQ_DIR_MASK)  /* IN and OUT w/o data */
           || psc->setup.wLength == 0) {
          UsbdcoreCbSetup(psc->unit, &psc->setup);

        } else {        /* OUT with data */
          DevUsbPrepareReceive(psc->unit, 0, psc->setup.buf, psc->setup.wLength);
          psc->waitSetupPayload = 1;
        }
      }

    }

    epnum++;
    epbit >>= 1;
  }

  return;
}


#define ENABLE_TXFE_INTERRUPT 0
static void
DevUsbInterruptEpIn(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;

  uint32_t              epintr, epnum, epbit;

  p = psc->dev;

  epnum = 0;
  epbit   = p->DAINT & USB_DAINT_IEPINT_MASK;  /* LSB 16bit is ep in */
  epbit >>= USB_DAINT_IEPINT_SHIFT;

  while(epbit) {
    if(epbit & 1) {
      uint32_t  mask, intr_orig;

      mask      = p->DIEPMSK;
#if ENABLE_TXFE_INTERRUPT
      mask     |= ((p->DIEPEMPMSK >> epnum) & 1) << USB_EPINT_TXFE_SHIFT;
#endif
      epintr    = p->in[epnum].INT;
      intr_orig = epintr;
      epintr   &= mask;
      p->in[epnum].INT = epintr;        /* clear interrupt */

      /* transfer complete interrupt */
      if(epintr & USB_EPINT_XFRC_MASK) {
        if(psc->in[epnum].fSent) {
          p->DIEPEMPMSK &= ~(1 << epnum);
          UsbdcoreCbDataInDone(psc->unit, epnum);
        } else {
          DevUsbWritePacket(psc, epnum);
        }
      }

#if 0
      /* tx fifo empty interrupt */
      /* errata: TXFE interrupt must not be used for to send the next data
       * use the XFRC interrupt for to send.
       * the data can be filled after sending completely.
       */
      if(epintr & USB_EPINT_TXFE_MASK) {
        DevUsbWritePacket(psc, epnum);
      }
#endif
    }

    epnum++;
    epbit >>= 1;
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
DevUsbInterruptRecvData(devUsbSc_t *psc)
{

  uint32_t              val;
  uint32_t              size;
  uint32_t              num;

  devUsbEp_t            *ep;
  uint32_t              *ptr;

  stm32Dev_USB          *p;

  p = psc->dev;

  p->GINTMSK &= ~USB_GINTSTS_RXFLVL_MASK;        /* mask intr */

  val = p->GRXSTSP;
  num   = (val & USB_GRXSTSP_EPNUM_MASK) >> USB_GRXSTSP_EPNUM_SHIFT;
  size  = (val & USB_GRXSTSP_BCNT_MASK)  >> USB_GRXSTSP_BCNT_SHIFT;
  ep = &psc->out[num];

  if(size > ep->size) size = ep->size;

  switch(val & USB_GRXSTSP_PKTSTS_MASK) {
  case  USB_GRXSTSP_PKTSTS_OUT_RECV:
    if(size) {
      ptr = (uint32_t *)(ep->ptr + ep->cnt);    /* adhoc   ptr */
      for(int i = 0; i < (size+3)/4; i++) {
        *ptr = p->DFIFO[0][0];
        ptr++;
      }
      ep->cnt  += size;

    }

    if((p->out[num].CTL & USB_EPCTL_EPTYP_MASK) == USB_EPCTL_EPTYP_ISOC) {
      UsbdcoreCbDataOut(psc->unit, num, size);

    } else {
      if(ep->cnt < ep->maxsize) {
        UsbdcoreCbDataOut(psc->unit, num, size);
      } else if(ep->cnt >= ep->size) {
        UsbdcoreCbDataOut(psc->unit, num, ep->size);
      } else {
        /*p->out[num].SIZ  = (USB_EPSIZ_PKTCNT_ONE | USB_EPSIZ_XFRSIZ_VAL(ep->maxsize));*/
        p->out[num].CTL  |= (USB_EPCTL_CNAK | USB_EPCTL_EPENA_YES);
      }
    }
    break;

  case  USB_GRXSTSP_PKTSTS_SETUP_RECV:
    ptr = (uint32_t *)&psc->setup;
    ptr[0] = p->DFIFO[0][0];
    ptr[1] = p->DFIFO[0][0];
    ep->size += size;
    break;
  }

  p->GINTMSK |= USB_GINTSTS_RXFLVL_EN; /* un-mask intr */

  return 0;
}


static int
DevUsbResetModule(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;

  p = psc->dev;

  /* Core Soft Reset */
  p->GRSTCTL |= USB_GRSTCTL_CSRST_MASK;

  while(p->GRSTCTL & USB_GRSTCTL_CSRST_MASK);

  return 0;
}


static int
DevUsbResetPort(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;

  uint32_t              val;

  p = psc->dev;

  val = p->HPRT;

  val &= ~(USB_HPRT_PENA_YES    | USB_HPRT_PCDET_MASK |
           USB_HPRT_PENCHNG_YES | USB_HPRT_POCCHNG_MASK );

  p->HPRT = (val | USB_HPRT_PRST_YES);
  RtosTaskSleep(100);                                /* wait 10msec at least */
  p->HPRT =  val;

  return 0;
}



static void
DevUsbSetTurnArroundTime(devUsbSc_t *psc)
{
  uint32_t                      val;
  uint32_t                      hclk;
  systemClockFreq_t             clk;

  stm32Dev_USB                  *p;

  p = psc->dev;

  /* get the core clock frequency */
  SystemGetClockValue(&clk);
  hclk = clk.core;

  if(/*(clk.core >= 14200000)&&*/ (clk.core < 15000000)) {
    val = USB_GUSBCFG_TRDT_14_2TO15_0MHZ;
  } else if(clk.core < 16000000) {
    val = USB_GUSBCFG_TRDT_15_0TO16_0MHZ;
  } else if(clk.core < 17200000) {
    val = USB_GUSBCFG_TRDT_16_0TO17_2MHZ;
  } else if(clk.core < 18500000) {
    val = USB_GUSBCFG_TRDT_17_2TO18_5MHZ;
  } else if(clk.core < 20000000) {
    val = USB_GUSBCFG_TRDT_18_5TO20_0MHZ;
  } else if(clk.core < 21800000) {
    val = USB_GUSBCFG_TRDT_20_0TO21_8MHZ;
  } else if(clk.core < 24000000) {
    val = USB_GUSBCFG_TRDT_21_8TO24_0MHZ;
  } else if(clk.core < 27700000) {
    val = USB_GUSBCFG_TRDT_24_0TO27_5MHZ;
  } else if(clk.core < 32000000) {
    val = USB_GUSBCFG_TRDT_27_5TO32_0MHZ;
  } else /* if(clk.core >= 32000000) */ {
    val = USB_GUSBCFG_TRDT_OVER32_0MHZ;
  }

  val |= p->GUSBCFG & ~USB_GUSBCFG_TRDT_MASK;
  p->GUSBCFG |= val;

  return;
}


static int
DevUsbGetBusSpeed(devUsbSc_t *psc)
{
  int                   speed;
  stm32Dev_USB          *p;

  p = psc->dev;

  switch(p->DSTS & USB_DSTS_ENUMSPD_MASK) {
  case USB_DSTS_ENUMSPD_HIGH: speed = USB_SPEED_HIGH; break;
  default:
  case USB_DSTS_ENUMSPD_FULL: speed = USB_SPEED_FULL; break;
  }

  return speed;
}


static void
DevUsbFlushFifoRx(devUsbSc_t *psc, int num)
{
  stm32Dev_USB          *p;
  p = psc->dev;

  p->GRSTCTL = USB_GRSTCTL_RXFFLSH_EN;
  while(!(p->GRSTCTL & USB_GRSTCTL_RXFFLSH_MASK));

  return;
}
static void
DevUsbFlushFifoTx(devUsbSc_t *psc, int num)
{
  stm32Dev_USB          *p;
  p = psc->dev;

  p->GRSTCTL = USB_GRSTCTL_TXFFLSH_EN | USB_GRSTCTL_TXFNUM(num);
  while(!(p->GRSTCTL & USB_GRSTCTL_TXFFLSH_MASK));

  return;
}


#if 0
static int
DevUsbEpOutEnable(devUsbSc_t *psc, uint8_t epnum, uint8_t *ptr)
{
  int                   result = -1;

  stm32Dev_USB          *p;
  uint32_t              val;
  int                   num;

  num = epnum & 0x7f;

  p = psc->dev;

  if(num == 0) {
    p->out[num].SIZ  = (USB_EPSIZ_STUPCNT_1PKTS | USB_EPSIZ_PKTCNT_ONE |
                        USB_EPSIZ_XFRSIZ_VAL(3*8));
  } else {
    /*p->out[num].SIZ  = (USB_EPSIZ_PKTCNT_ONE | USB_EPSIZ_XFRSIZ_VAL(0x40));*/
  }

  if(psc->dma) {
    p->out[num].DMA = (uint32_t)ptr;
    p->out[num].CTL = USB_EPCTL_EPENA_YES | USB_EPCTL_USBAEP_YES;
  }

  return result;
}
#endif


static int
DevUsbSetSpeed(devUsbSc_t *psc, int speed)
{
  stm32Dev_USB          *p;
  uint32_t              val;

  p = psc->dev;

  val = p->DCFG;
  val &= ~USB_DCFG_DSPD_MASK;
  switch(speed) {
  case    USBIF_SPEED_FULL:
  case    USBIF_SPEED_LOW:
    val = USB_DCFG_DSPD_FS;
    break;
  case    USBIF_SPEED_HIGH:
  case    USBIF_SPEED_SUPER:
    val = USB_DCFG_DSPD_HS;
    break;
  }

  p->DCFG = val;

  return 0;
}




#if 0
static int
DevUsbWritePacket(devUsbSc_t *psc, uint8_t epnum)
{
  stm32Dev_USB          *p;
  int                   size;
  int                   num;
  int                   i;
  uint32_t              *src;
  uint32_t              epctl, epsiz;

  num = epnum & 0x7f;
  if(psc->in[num].fSent) goto end;

  p = psc->dev;

  size = psc->in[num].size;
  epctl = USB_EPCTL_CNAK | USB_EPCTL_EPENA_YES;

  if(size > 0) {                      /* data exist */
    size -= psc->in[num].cnt;
    if(size >= psc->in[num].maxsize) {
      size = psc->in[num].maxsize;
      /* Enable the Tx FIFO Empty Interrupt for this EP */
      p->DIEPEMPMSK |= 1 << (num);
    } else {
      psc->in[num].fSent = 1;
    }
    epsiz = USB_EPSIZ_PKTCNT_ONE | USB_EPSIZ_XFRSIZ_VAL(size);

  } else if(size == 0) {
    epsiz = USB_EPSIZ_PKTCNT_ONE | USB_EPSIZ_XFRSIZ_VAL(0);
    psc->in[num].fSent = 1;

  } else {
    epsiz = 0;
    epctl |= USB_EPCTL_STALL_YES | USB_EPCTL_EPDIS_YES;
    psc->in[num].fSent = 1;
  }
  /* store the size and the control info to register */
  p->in[num].SIZ = epsiz;
  p->in[num].CTL &= ~USB_EPCTL_STALL_MASK;
  p->in[num].CTL |= epctl;

  /* send data to register and modify the data informations
   * fSent flag is set, if data is empty and the last data is not
   * max packet size
   */
  if(size > 0) {
    src = (uint32_t *)psc->in[num].ptr;

    for(i = 0; i < (size+3)/4; i++) {
      p->DFIFO[num][0] = *src++;
    }

    psc->in[num].ptr += size;
    psc->in[num].cnt += size;
  }

 end:
  return 0;
}
#else

static int
DevUsbStartPacketOut(devUsbSc_t *psc, uint8_t epnum)
{
  stm32Dev_USB          *p;
  int                   size;
  int                   num;
  uint32_t              ctl = 0, siz = 0;

  num = epnum & 0x7f;
  /*if(psc->in[num].fSent) goto end;*/

  p = psc->dev;

  if(size > psc->out[num].maxsize) size = psc->out[num].maxsize;
  ctl = USB_EPCTL_CNAK | USB_EPCTL_EPENA_YES;

  if((p->out[num].CTL & USB_EPCTL_EPTYP_MASK) == USB_EPCTL_EPTYP_ISOC) {
    if((p->DSTS & USB_DSTS_FNSOF_LSB_MASK) == 0) {
      ctl |= USB_EPCTL_SD1PID_SODDFRM;
    } else {
      ctl |= USB_EPCTL_SD0PID_SEVNFRM;
    }
  }

  siz = USB_EPSIZ_PKTCNT_ONE;
  if(num == 0) {
    siz |= USB_EPSIZ_STUPCNT_1PKTS;
  }

  p->out[num].SIZ  = siz;
  p->out[num].CTL |= ctl;

end:
  return 0;
}
static int
DevUsbStartPacketIn(devUsbSc_t *psc, uint8_t epnum)
{
  stm32Dev_USB          *p;
  int                   size;
  int                   num;
  int                   i;
  uint32_t              *src;
  uint32_t              epctl, epsiz;

  num = epnum & 0x7f;
  if(psc->in[num].fSent) goto end;

  p = psc->dev;

  size = psc->in[num].size;
  epctl = USB_EPCTL_CNAK | USB_EPCTL_EPENA_YES;

  if(size > 0) {                      /* data exist */
  } else if(size == 0) {
    epsiz = USB_EPSIZ_PKTCNT_ONE | USB_EPSIZ_XFRSIZ_VAL(0);
    psc->in[num].fSent = 1;

  } else {
    epsiz = 0;
    epctl |= USB_EPCTL_STALL_YES | USB_EPCTL_EPDIS_YES;
    psc->in[num].fSent = 1;
  }

  if(size <= 0) {
    /* store the size and the control info to register */
    p->in[num].SIZ = epsiz;
    p->in[num].CTL &= ~USB_EPCTL_STALL_MASK;
    p->in[num].CTL |= epctl;
  }

end:
  return 0;
}
static int
DevUsbWritePacket(devUsbSc_t *psc, uint8_t epnum)
{
  stm32Dev_USB          *p;
  int                   size;
  int                   num;
  int                   i;
  uint32_t              *src;
  uint32_t              siz, ctl;

  num = epnum & 0x7f;
  //if(psc->in[num].fSent) goto end;
  size = psc->in[num].size;

  if(size > 0) {                      /* data exist */
    size -= psc->in[num].cnt;
    if(size > psc->in[num].maxsize) {
      size = psc->in[num].maxsize;

    } else {
      psc->in[num].fSent = 1;
    }

    /* send data to register and modify the data informations
     * fSent flag is set, if data is empty and the last data is not
     * max packet size
     */
    if(size > 0) {
      p = psc->dev;

      siz = USB_EPSIZ_PKTCNT_ONE | USB_EPSIZ_XFRSIZ_VAL(size);
      ctl = USB_EPCTL_CNAK | USB_EPCTL_EPENA_YES;

      if((p->in[num].CTL & USB_EPCTL_EPTYP_MASK) == USB_EPCTL_EPTYP_ISOC) {
        siz |= USB_EPSIZ_MCNT_1PKTS;
        if ((p->DSTS & USB_DSTS_FNSOF_LSB_MASK) == 0) {
          ctl |= USB_EPCTL_SD1PID_SODDFRM;
        } else {
          ctl |= USB_EPCTL_SD0PID_SEVNFRM;
        }
      }

      p->in[num].SIZ  = siz;
      p->in[num].CTL |= ctl;

      src = (uint32_t *)psc->in[num].ptr;

      for(i = 0; i < (size+3)/4; i++) {
        *p->DFIFO[num] = *src++;
      }

      psc->in[num].ptr += size;
      psc->in[num].cnt += size;
    }
  }

end:
  return 0;
}
#endif


static int
DevUsbConnect(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;
  p = psc->dev;

  p->DCTL &= ~USB_DCTL_SDIS_MASK;

  return 0;
}
static int
DevUsbDisconnect(devUsbSc_t *psc)
{
  stm32Dev_USB          *p;
  p = psc->dev;

  p->DCTL &= ~USB_DCTL_SDIS_MASK;

  return 0;
}




void
DevUsbDebugShowGeneralReg(stm32Dev_USB *p, int num)
{
  printf("GOTGCTL  %x %x %x %x %x %x %x\r\n", p->GOTGCTL, p->GOTGINT, p->GAHBCFG, p->GUSBCFG, p->GRSTCTL, p->GINTSTS, p->GINTMSK);
  printf("GRXSTSR  %x %x %x\r\n", p->GRXSTSR, p->GRXFSIZ, p->DIEPTXF0_HNPTXFSIZ);
  printf("HNPTXSTS %x %x %x %x %x %x %x\r\n", p->HNPTXSTS, p->GI2CCTL, p->GCCFG, p->CID, p->GLPMCFG, p->HPTXFSIZ, p->DIEPTXF[num]);

  printf("DCFG     %x %x %x %x %x %x %x\r\n", p->DCFG, p->DCTL, p->DSTS, p->DIEPMSK, p->DOEPMSK, p->DAINT, p->DAINTMSK);
  printf("DVBUSDIS %x %x %x %x %x %x\r\n", p->DVBUSDIS, p->DVBUSPULSE, p->DTHRCTL, p->DIEPEMPMSK, p->DEACHINT, p->DEACHINTMSK);
  printf("in[]     %x %x %x %x %x\r\n", p->in[num].CTL, p->in[num].INT, p->in[num].SIZ, p->in[num].DMA, p->in[num].STS);
  printf("out[]    %x %x %x %x %x\r\n", p->out[num].CTL, p->out[num].INT, p->out[num].SIZ, p->out[num].DMA, p->out[num].STS);

  printf("in[]     %x %x %x %x %x\r\n", &p->in[num].CTL, &p->in[num].INT, &p->in[num].SIZ, &p->in[num].DMA, &p->in[num].STS);
  printf("out[]    %x %x %x %x %x\r\n", &p->out[num].CTL, &p->out[num].INT, &p->out[num].SIZ, &p->out[num].DMA, &p->out[num].STS);

  return;
}
