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

#ifndef _STM32USB16_H_
#define _STM32USB16_H_

typedef struct {
  __IO uint16_t         R;
#define USB_EP_CTR_RX_SHIFT     (15)
#define USB_EP_CTR_RX_MASK      (1 << (USB_EP_CTR_RX_SHIFT))
#define USB_EP_CTR_RX           (1 << (USB_EP_CTR_RX_SHIFT))
#define USB_EP_DTOG_RX_SHIFT    (14)
#define USB_EP_DTOG_RX_MASK     (1 << (USB_EP_DTOG_RX_SHIFT))
#define USB_EP_DTOG_RX          (1 << (USB_EP_DTOG_RX_SHIFT))
#define USB_EP_SW_BUF_TX_SHIFT  (USB_EP_DTOG_RX_SHIFT)
#define USB_EP_SW_BUF_TX_MASK   (USB_EP_DTOG_RX_MASK)
#define USB_EP_SW_BUF_TX        (USB_EP_DTOG_RX)
#define USB_EP_STAT_RX_SHIFT    (12)
#define USB_EP_STAT_RX_MASK     (3 << (USB_EP_STAT_RX_SHIFT))
#define USB_EP_STAT_RX_DISABLED (0 << (USB_EP_STAT_RX_SHIFT))
#define USB_EP_STAT_RX_STALL    (1 << (USB_EP_STAT_RX_SHIFT))
#define USB_EP_STAT_RX_NAK      (2 << (USB_EP_STAT_RX_SHIFT))
#define USB_EP_STAT_RX_VALID    (3 << (USB_EP_STAT_RX_SHIFT))
#if 0
#define USB_EP_STAT_RX_DTOG0    (1 << (USB_EP_STAT_RX_SHIFT))
#define USB_EP_STAT_RX_DTOG1    (2 << (USB_EP_STAT_RX_SHIFT))
#endif
#define USB_EP_SETUP_SHIFT      (11)
#define USB_EP_SETUP_MASK       (1 << (USB_EP_SETUP_SHIFT))
#define USB_EP_EP_TYPE_SHIFT    (9)
#define USB_EP_EP_TYPE_MASK     (3 << (USB_EP_EP_TYPE_SHIFT))
#define USB_EP_EP_TYPE_BULK     (0 << (USB_EP_EP_TYPE_SHIFT))
#define USB_EP_EP_TYPE_CTRL     (1 << (USB_EP_EP_TYPE_SHIFT))
#define USB_EP_EP_TYPE_ISO      (2 << (USB_EP_EP_TYPE_SHIFT))
#define USB_EP_EP_TYPE_INTR     (3 << (USB_EP_EP_TYPE_SHIFT))
#define USB_EP_EP_TYPE_DBL_BUF     (0 << (USB_EP_EP_TYPE_SHIFT))
#define USB_EP_EP_TYPE_STATUS_OUT  (1 << (USB_EP_EP_TYPE_SHIFT))
#define USB_EP_EP_KIND_SHIFT    (8)
#define USB_EP_EP_KIND_MASK     (1 << (USB_EP_EP_KIND_SHIFT))
#define USB_EP_EP_KIND          (1 << (USB_EP_EP_KIND_SHIFT))
#define USB_EP_CTR_TX_SHIFT     (7)
#define USB_EP_CTR_TX_MASK      (1 << (USB_EP_CTR_TX_SHIFT))
#define USB_EP_CTR_TX           (1 << (USB_EP_CTR_TX_SHIFT))
#define USB_EP_DTOG_TX_SHIFT    (6)
#define USB_EP_DTOG_TX_MASK     (1 << (USB_EP_DTOG_TX_SHIFT))
#define USB_EP_DTOG_TX          (1 << (USB_EP_DTOG_TX_SHIFT))
#define USB_EP_SW_BUF_RX_SHIFT  (USB_EP_DTOG_TX_SHIFT)
#define USB_EP_SW_BUF_RX_MASK   (USB_EP_DTOG_TX_MASK)
#define USB_EP_SW_BUF_RX        (USB_EP_DTOG_TX)
#define USB_EP_STAT_TX_SHIFT    (4)
#define USB_EP_STAT_TX_MASK     (3 << (USB_EP_STAT_TX_SHIFT))
#define USB_EP_STAT_TX_DISABLED (0 << (USB_EP_STAT_TX_SHIFT))
#define USB_EP_STAT_TX_STALL    (1 << (USB_EP_STAT_TX_SHIFT))
#define USB_EP_STAT_TX_NAK      (2 << (USB_EP_STAT_TX_SHIFT))
#define USB_EP_STAT_TX_VALID    (3 << (USB_EP_STAT_TX_SHIFT))
#if 0
#define USB_EP_STAT_TX_DTOG0       (1 << (USB_EP_STAT_TX_SHIFT))
#define USB_EP_STAT_TX_DTOG1       (2 << (USB_EP_STAT_TX_SHIFT))
#endif
#define USB_EP_EA_SHIFT         (0)
#define USB_EP_EA_MASK          (0xf << (USB_EP_EA_SHIFT))
#define USB_EP_EA_VAL(x)        (((x) << (USB_EP_EA_SHIFT)) & USB_EP_EA_MASK)

#define USB_EP_XSTAT_MASK       ((USB_EP_CTR_RX) | (USB_EP_SETUP_MASK) | \
                                 (USB_EP_EP_TYPE_MASK) | (USB_EP_EP_KIND_MASK) | \
                                 (USB_EP_CTR_TX) | (USB_EP_EA_MASK))
#define USB_EP_DTOG_MASK        ((USB_EP_DTOG_RX_MASK) | (USB_EP_DTOG_TX_MASK))
#define USB_EP_TOGGLEBIT_MASK   ((USB_EP_DTOG_RX_MASK) | (USB_EP_DTOG_TX_MASK) | \
                                 (USB_EP_STAT_RX_MASK) | (USB_EP_STAT_TX_MASK))

  uint16_t      reserved;
} stm32Usb16Ep_t;


typedef struct {
  stm32Usb16Ep_t        EP[16];
  __IO uint16_t         CNTR;
  uint16_t              reserved42;
  __IO uint16_t         ISTR;
  uint16_t              reserved46;
#define USB_CNTR_CTR_SHIFT      (15)
#define USB_CNTR_CTR_MASK       (1 << (USB_CNTR_CTR_SHIFT))
#define USB_CNTR_CTR            (1 << (USB_CNTR_CTR_SHIFT))
#define USB_CNTR_PMAOVR_SHIFT   (14)
#define USB_CNTR_PMAOVR_MASK    (1 << (USB_CNTR_PMAOVR_SHIFT))
#define USB_CNTR_PMAOVR         (1 << (USB_CNTR_PMAOVR_SHIFT))
#define USB_CNTR_ERR_SHIFT      (13)
#define USB_CNTR_ERR_MASK       (1 << (USB_CNTR_ERR_SHIFT))
#define USB_CNTR_ERR            (1 << (USB_CNTR_ERR_SHIFT))
#define USB_CNTR_WKUP_SHIFT     (12)
#define USB_CNTR_WKUP_MASK      (1 << (USB_CNTR_WKUP_SHIFT))
#define USB_CNTR_WKUP           (1 << (USB_CNTR_WKUP_SHIFT))
#define USB_CNTR_SUSP_SHIFT     (11)
#define USB_CNTR_SUSP_MASK      (1 << (USB_CNTR_SUSP_SHIFT))
#define USB_CNTR_SUSP           (1 << (USB_CNTR_SUSP_SHIFT))
#define USB_CNTR_RESET_SHIFT    (10)
#define USB_CNTR_RESET_MASK     (1 << (USB_CNTR_RESET_SHIFT))
#define USB_CNTR_RESET          (1 << (USB_CNTR_RESET_SHIFT))
#define USB_CNTR_SOF_SHIFT      (9)
#define USB_CNTR_SOF_MASK       (1 << (USB_CNTR_SOF_SHIFT))
#define USB_CNTR_SOF            (1 << (USB_CNTR_SOF_SHIFT))
#define USB_CNTR_ESOF_SHIFT     (8)
#define USB_CNTR_ESOF_MASK      (1 << (USB_CNTR_ESOF_SHIFT))
#define USB_CNTR_ESOF           (1 << (USB_CNTR_ESOF_SHIFT))
#define USB_CNTR_L1REQ_SHIFT    (7)
#define USB_CNTR_L1REQ_MASK     (1 << (USB_CNTR_L1REQ_SHIFT))
#define USB_CNTR_L1REQ          (1 << (USB_CNTR_L1REQ_SHIFT))
#define USB_CNTR_L1RESUME_SHIFT (5)
#define USB_CNTR_L1RESUME_MASK  (1 << (USB_CNTR_L1RESUME_SHIFT))
#define USB_CNTR_L1RESUME       (1 << (USB_CNTR_L1RESUME_SHIFT))
#define USB_CNTR_RESUME_SHIFT   (4)
#define USB_CNTR_RESUME_MASK    (1 << (USB_CNTR_RESUME_SHIFT))
#define USB_CNTR_RESUME         (1 << (USB_CNTR_RESUME_SHIFT))
#define USB_CNTR_FSUSP_SHIFT    (3)
#define USB_CNTR_FSUSP_MASK     (1 << (USB_CNTR_FSUSP_SHIFT))
#define USB_CNTR_FSUSP          (1 << (USB_CNTR_FSUSP_SHIFT))
#define USB_CNTR_LPMODE_SHIFT   (2)
#define USB_CNTR_LPMODE_MASK    (1 << (USB_CNTR_LPMODE_SHIFT))
#define USB_CNTR_LPMODE         (1 << (USB_CNTR_LPMODE_SHIFT))
#define USB_CNTR_PDWN_SHIFT     (1)
#define USB_CNTR_PDWN_MASK      (1 << (USB_CNTR_PDWN_SHIFT))
#define USB_CNTR_PDWN           (1 << (USB_CNTR_PDWN_SHIFT))
#define USB_CNTR_FRES_SHIFT     (0)
#define USB_CNTR_FRES_MASK      (1 << (USB_CNTR_FRES_SHIFT))
#define USB_CNTR_FRES           (1 << (USB_CNTR_FRES_SHIFT))

#define USB_ISTR_DIR_SHIFT      (4)
#define USB_ISTR_DIR_MASK       (1 << (USB_ISTR_DIR_SHIFT))
#define USB_ISTR_DIR_IN         (0 << (USB_ISTR_DIR_SHIFT))
#define USB_ISTR_DIR_OUT        (1 << (USB_ISTR_DIR_SHIFT))
#define USB_ISTR_EP_ID_SHIFT    (0)
#define USB_ISTR_EP_ID_MASK     (0xf << (USB_ISTR_EP_ID_SHIFT))

  __IO uint16_t         FNR;
  uint16_t              reserved4a;
#define USB_FNR_RXDP_SHIFT      (15)
#define USB_FNR_RXDP_MASK       (1 << (USB_FNR_RXDP_SHIFT))
#define USB_FNR_RXDM_SHIFT      (14)
#define USB_FNR_RXDM_MASK       (1 << (USB_FNR_RXDM_SHIFT))
#define USB_FNR_LCK_SHIFT       (13)
#define USB_FNR_LCK_MASK        (1 << (USB_FNR_LCK_SHIFT))
#define USB_FNR_LSOF_SHIFT      (11)
#define USB_FNR_LSOF_MASK       (3 << (USB_FNR_LSOF_SHIFT))
#define USB_FNR_NUM_SHIFT       (0)
#define USB_FNR_NUM_MASK        (0x7ff << (USB_FNR_NUM_SHIFT))

  __IO uint16_t         DADDR;
  uint16_t              reserved4e;
#define USB_DADDR_EF_SHIFT      (7)
#define USB_DADDR_EF_MASK       (1 << (USB_DADDR_EF_SHIFT))
#define USB_DADDR_EF_NO         (0 << (USB_DADDR_EF_SHIFT))
#define USB_DADDR_EF_YES        (1 << (USB_DADDR_EF_SHIFT))
#define USB_DADDR_ADD_SHIFT     (0)
#define USB_DADDR_ADD_MASK      (0x7f << (USB_DADDR_ADD_SHIFT))
#define USB_DADDR_ADD_VAL(x)    (((x) << (USB_DADDR_ADD_SHIFT)) & USB_DADDR_ADD_MASK)

  __IO uint16_t         BTABLE;
  uint16_t              reserved52;
#define USB_BTABLE_SHIFT        (0)
#define USB_BTABLE_MASK         (0xffff << (USB_BTABLE_SHIFT))
#define USB_BTABLE_8BYTES       (8 << (USB_BTABLE_SHIFT))
#define USB_BTABLE_VAL(x)       (((x) << (USB_BTABLE_SHIFT)) & USB_BTABLE_MASK)

  __IO uint16_t         LPMCSR;
  uint16_t              reserved56;
#define USB_LPMCSR_BES_SHIFT    (4)
#define USB_LPMCSR_BES_MASK     (0xf << (USB_LPMCSR_BES_SHIFT))
#define USB_LPMCSR_REMWAKE_SHIFT   (3)
#define USB_LPMCSR_REMWAKE_MASK    (1 << (USB_LPMCSR_REMWAKE_SHIFT))
#define USB_LPMCSR_LPMACK_SHIFT (1)
#define USB_LPMCSR_LPMACK_MASK  (1 << (USB_LPMCSR_LPMACK_SHIFT))
#define USB_LPMCSR_LPMEN_SHIFT  (0)
#define USB_LPMCSR_LPMEN_MASK   (1 << (USB_LPMCSR_LPMEN_SHIFT))

  __IO uint16_t         BCDR;
  uint16_t              reserved5a;
#define USB_BCDR_DPPU_SHIFT     (15)
#define USB_BCDR_DPPU_MASK      (1 << (USB_BCDR_DPPU_SHIFT))
#define USB_BCDR_DPPU_NO        (0 << (USB_BCDR_DPPU_SHIFT))
#define USB_BCDR_DPPU_YES       (1 << (USB_BCDR_DPPU_SHIFT))
#define USB_BCDR_PS2DET_SHIFT   (7)
#define USB_BCDR_PS2DET_MASK    (1 << (USB_BCDR_PS2DET_SHIFT))
#define USB_BCDR_SDET_SHIFT     (6)
#define USB_BCDR_SDET_MASK      (1 << (USB_BCDR_SDET_SHIFT))
#define USB_BCDR_PDET_SHIFT     (5)
#define USB_BCDR_PDET_MASK      (1 << (USB_BCDR_PDET_SHIFT))
#define USB_BCDR_DCDET_SHIFT    (4)
#define USB_BCDR_DCDET_MASK     (1 << (USB_BCDR_DCDET_SHIFT))
#define USB_BCDR_SDEN_SHIFT     (3)
#define USB_BCDR_SDEN_MASK      (1 << (USB_BCDR_SDEN_SHIFT))
#define USB_BCDR_SDEN_NO        (0 << (USB_BCDR_SDEN_SHIFT))
#define USB_BCDR_SDEN_YES       (1 << (USB_BCDR_DPPU_SHIFT))
#define USB_BCDR_PDEN_SHIFT     (2)
#define USB_BCDR_PDEN_MASK      (1 << (USB_BCDR_PDEN_SHIFT))
#define USB_BCDR_PDEN_NO        (0 << (USB_BCDR_PDEN_SHIFT))
#define USB_BCDR_PDEN_YES       (1 << (USB_BCDR_DPPU_SHIFT))
#define USB_BCDR_DCDEN_SHIFT    (1)
#define USB_BCDR_DCDEN_MASK     (1 << (USB_BCDR_DCDEN_SHIFT))
#define USB_BCDR_DCDEN_NO       (0 << (USB_BCDR_DCDEN_SHIFT))
#define USB_BCDR_DCDEN_YES      (1 << (USB_BCDR_DPPU_SHIFT))
#define USB_BCDR_BCDEN_SHIFT    (0)
#define USB_BCDR_BCDEN_MASK     (1 << (USB_BCDR_BCDEN_SHIFT))
#define USB_BCDR_BCDEN_NO       (0 << (USB_BCDR_BCDEN_SHIFT))
#define USB_BCDR_BCDEN_YES      (1 << (USB_BCDR_BCDEN_SHIFT))

} stm32Dev_USB;


typedef struct {
  __IO uint16_t         txAddr;
  __IO uint16_t         txCnt;
#define USB_COUNT_TX_COUNT_SHIFT        0
#define USB_COUNT_TX_COUNT_MASK         (0x3ff << (USB_COUNT_TX_COUNT_SHIFT))
  __IO uint16_t         rxAddr;
  __IO uint16_t         rxCnt;
#define USB_COUNT_RX_BLSIZE_SHIFT       15
#define USB_COUNT_RX_BLSIZE_MASK        (1 << (USB_COUNT_RX_BLSIZE_SHIFT))
#define USB_COUNT_RX_BLSIZE_NO          (0 << (USB_COUNT_RX_BLSIZE_SHIFT))
#define USB_COUNT_RX_BLSIZE_YES         (1 << (USB_COUNT_RX_BLSIZE_SHIFT))
#define USB_COUNT_RX_NUM_BLOCK_SHIFT    10
#define USB_COUNT_RX_NUM_BLOCK_MASK     (0x1f << (USB_COUNT_RX_NUM_BLOCK_SHIFT))
#define USB_COUNT_RX_NUM_BLOCK2_VAL(x)  (((((x)>>1))   << (USB_COUNT_RX_NUM_BLOCK_SHIFT)) & USB_COUNT_RX_NUM_BLOCK_MASK)
#define USB_COUNT_RX_NUM_BLOCK32_VAL(x) (((((x)>>5)-1) << (USB_COUNT_RX_NUM_BLOCK_SHIFT)) & USB_COUNT_RX_NUM_BLOCK_MASK)
#define USB_COUNT_RX_COUNT_SHIFT        0
#define USB_COUNT_RX_COUNT_MASK         (0x3ff << (USB_COUNT_RX_COUNT_SHIFT))
} stm32Dev_USB_SRAM_ep;


typedef struct {
  stm32Dev_USB_SRAM_ep  ep[8];
} stm32Dev_USB_SRAM_Header;


#define USB_FS_MAX_PACKET_SIZE          64
#define USB_SPEED_SUPER                 3
#define USB_SPEED_HIGH                  2
#define USB_SPEED_LOW                   1
#define USB_SPEED_FULL                  0



#endif
