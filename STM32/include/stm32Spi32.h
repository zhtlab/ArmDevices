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

#ifndef _STM32SPI32_H_
#define _STM32SPI32_H_

typedef struct {
  __IO uint32_t         CR1;            /* 0x00 */
#define SPI_CR1_SSI_SHIFT               (12)
#define SPI_CR1_SSI_MASK                (1 << (SPI_CR1_SSI_SHIFT))
#define SPI_CR1_SSI_NO                  (0 << (SPI_CR1_SSI_SHIFT))
#define SPI_CR1_SSI_YES                 (1 << (SPI_CR1_SSI_SHIFT))
#define SPI_CR1_HDDIR_SHIFT             (11)
#define SPI_CR1_HDDIR_MASK              (1 << (SPI_CR1_HDDIR_SHIFT))
#define SPI_CR1_HDDIR_NO                (0 << (SPI_CR1_HDDIR_SHIFT))
#define SPI_CR1_HDDIR_YES               (1 << (SPI_CR1_HDDIR_SHIFT))
#define SPI_CR1_CSUSP_SHIFT             (10)
#define SPI_CR1_CSUSP_MASK              (1 << (SPI_CR1_CSUSP_SHIFT))
#define SPI_CR1_CSUSP_NO                (0 << (SPI_CR1_CSUSP_SHIFT))
#define SPI_CR1_CSUSP_YES               (1 << (SPI_CR1_CSUSP_SHIFT))
#define SPI_CR1_CSTART_SHIFT            (9)
#define SPI_CR1_CSTART_MASK             (1 << (SPI_CR1_CSTART_SHIFT))
#define SPI_CR1_CSTART_NO               (0 << (SPI_CR1_CSTART_SHIFT))
#define SPI_CR1_CSTART_YES              (1 << (SPI_CR1_CSTART_SHIFT))
#define SPI_CR1_MASRX_SHIFT             (8)
#define SPI_CR1_MASRX_MASK              (1 << (SPI_CR1_MASRX_SHIFT))
#define SPI_CR1_MASRX_NO                (0 << (SPI_CR1_MASRX_SHIFT))
#define SPI_CR1_MASRX_YES               (1 << (SPI_CR1_MASRX_SHIFT))
#define SPI_CR1_SPE_SHIFT               (0)
#define SPI_CR1_SPE_MASK                (1 << (SPI_CR1_SPE_SHIFT))
#define SPI_CR1_SPE_NO                  (0 << (SPI_CR1_SPE_SHIFT))
#define SPI_CR1_SPE_YES                 (1 << (SPI_CR1_SPE_SHIFT))

  __IO uint32_t         CR2;            /* 0x04 */
#define SPI_CR2_TSER_SHIFT              (16)
#define SPI_CR2_TSER_MASK               (0xffff << (SPI_CR2_TSER_SHIFT))
#define SPI_CR2_TSIZE_SHIFT             (0)
#define SPI_CR2_TSIZE_MASK              (0xffff << (SPI_CR2_TSIZE_SHIFT))

  __IO uint32_t         CFG1;           /* 0x08 */
#define SPI_CFG1_MBR_SHIFT              (28)
#define SPI_CFG1_MBR_MASK               (7 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV2               (0 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV4               (1 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV8               (2 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV16              (3 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV32              (4 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV64              (5 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV128             (6 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_MBR_DIV256             (7 << (SPI_CFG1_MBR_SHIFT))
#define SPI_CFG1_TXDMAEN_SHIFT          (15)
#define SPI_CFG1_TXDMAEN_MASK           (1 << (SPI_CFG1_TXDMAEN_SHIFT))
#define SPI_CFG1_TXDMAEN_NO             (0 << (SPI_CFG1_TXDMAEN_SHIFT))
#define SPI_CFG1_TXDMAEN_YES            (1 << (SPI_CFG1_TXDMAEN_SHIFT))
#define SPI_CFG1_RXDMAEN_SHIFT          (14)
#define SPI_CFG1_RXDMAEN_MASK           (1 << (SPI_CFG1_RXDMAEN_SHIFT))
#define SPI_CFG1_RXDMAEN_NO             (0 << (SPI_CFG1_RXDMAEN_SHIFT))
#define SPI_CFG1_RXDMAEN_YES            (1 << (SPI_CFG1_RXDMAEN_SHIFT))
#define SPI_CFG1_FTHLV_SHIFT            (5)
#define SPI_CFG1_FTHLV_MASK             (15 << (SPI_CFG1_FTHLV_SHIFT))
#define SPI_CFG1_FTHLV_VAL(x)           (((x)-1)  << (SPI_CFG1_FTHLV_SHIFT))
#define SPI_CFG1_FTHLV_1BIT             (0  << (SPI_CFG1_FTHLV_SHIFT))
#define SPI_CFG1_FTHLV_8BIT             (7  << (SPI_CFG1_FTHLV_SHIFT))
#define SPI_CFG1_FTHLV_16BIT            (15  << (SPI_CFG1_FTHLV_SHIFT))
#define SPI_CFG1_DSIZE_SHIFT            (0)
#define SPI_CFG1_DSIZE_MASK             (31 << (SPI_CFG1_DSIZE_SHIFT))
#define SPI_CFG1_DSIZE_VAL(x)           (((x)-1)  << (SPI_CFG1_DSIZE_SHIFT))
#define SPI_CFG1_DSIZE_1BIT             (0  << (SPI_CFG1_DSIZE_SHIFT))
#define SPI_CFG1_DSIZE_8BIT             (7  << (SPI_CFG1_DSIZE_SHIFT))
#define SPI_CFG1_DSIZE_16BIT            (15 << (SPI_CFG1_DSIZE_SHIFT))
#define SPI_CFG1_DSIZE_32BIT            (31 << (SPI_CFG1_DSIZE_SHIFT))

  __IO uint32_t         CFG2;            /* 0x0c */
#define SPI_CFG2_AFCNTR_SHIFT           (31)
#define SPI_CFG2_AFCNTR_MASK            (1 << (SPI_CFG2_AFCNTR_SHIFT))
#define SPI_CFG2_AFCNTR_NO              (0 << (SPI_CFG2_AFCNTR_SHIFT))
#define SPI_CFG2_AFCNTR_YES             (1 << (SPI_CFG2_AFCNTR_SHIFT))
#define SPI_CFG2_SSOM_SHIFT             (30)
#define SPI_CFG2_SSOM_MASK              (1 << (SPI_CFG2_SSOM_SHIFT))
#define SPI_CFG2_SSOM_NO                (0 << (SPI_CFG2_SSOM_SHIFT))
#define SPI_CFG2_SSOM_YES               (1 << (SPI_CFG2_SSOM_SHIFT))
#define SPI_CFG2_SSM_SHIFT              (26)
#define SPI_CFG2_SSM_MASK               (1 << (SPI_CFG2_SSM_SHIFT))
#define SPI_CFG2_SSM_NO                 (0 << (SPI_CFG2_SSM_SHIFT))
#define SPI_CFG2_SSM_YES                (1 << (SPI_CFG2_SSM_SHIFT))
#define SPI_CFG2_CPOL_SHIFT             (25)
#define SPI_CFG2_CPOL_MASK              (3 << (SPI_CFG2_CPOL_SHIFT))
#define SPI_CFG2_CPOL_NO                (0 << (SPI_CFG2_CPOL_SHIFT))
#define SPI_CFG2_CPOL_YES               (1 << (SPI_CFG2_CPOL_SHIFT))
#define SPI_CFG2_CPHA_SHIFT             (24)
#define SPI_CFG2_CPHA_MASK              (1 << (SPI_CFG2_CPHA_SHIFT))
#define SPI_CFG2_CPHA_NO                (0 << (SPI_CFG2_CPHA_SHIFT))
#define SPI_CFG2_CPHA_YES               (1 << (SPI_CFG2_CPHA_SHIFT))
#define SPI_CFG2_LSBFRST_SHIFT          (23)
#define SPI_CFG2_LSBFRST_MASK           (1 << (SPI_CFG2_LSBFRST_SHIFT))
#define SPI_CFG2_LSBFRST_NO             (0 << (SPI_CFG2_LSBFRST_SHIFT))
#define SPI_CFG2_LSBFRST_YES            (1 << (SPI_CFG2_LSBFRST_SHIFT))
#define SPI_CFG2_MASTER_SHIFT           (22)
#define SPI_CFG2_MASTER_MASK            (1 << (SPI_CFG2_MASTER_SHIFT))
#define SPI_CFG2_MASTER_NO              (0 << (SPI_CFG2_MASTER_SHIFT))
#define SPI_CFG2_MASTER_YES             (1 << (SPI_CFG2_MASTER_SHIFT))
#define SPI_CFG2_SP_SHIFT               (19)
#define SPI_CFG2_SP_MASK                (1 << (SPI_CFG2_SP_SHIFT))
#define SPI_CFG2_SP_MOTOROLA            (0 << (SPI_CFG2_SP_SHIFT))
#define SPI_CFG2_SP_TI                  (1 << (SPI_CFG2_SP_SHIFT))
#define SPI_CFG2_COMM_SHIFT             (17)
#define SPI_CFG2_COMM_MASK              (3 << (SPI_CFG2_COMM_SHIFT))
#define SPI_CFG2_COMM_FULL_DUP          (0 << (SPI_CFG2_COMM_SHIFT))
#define SPI_CFG2_COMM_SIMPLEX_TX        (1 << (SPI_CFG2_COMM_SHIFT))
#define SPI_CFG2_COMM_SIMPLEX_RX        (2 << (SPI_CFG2_COMM_SHIFT))
#define SPI_CFG2_COMM_HALF_DUP          (3 << (SPI_CFG2_COMM_SHIFT))
  __IO uint32_t         IER;            /* 0x10 */
  __IO uint32_t         SR;             /* 0x14 */
#define SPI_SR_RXPLVL_SHIFT             (13)
#define SPI_SR_RXPLVL_MASK              (3 << (SPI_SR_RXPLVL_SHIFT))
#define SPI_SR_RXPCNT_SHIFT             (13)
#define SPI_SR_RXPCNT_MASK              (1 << (SPI_SR_RXPCNT_SHIFT))
#define SPI_SR_TXC_SHIFT                (12)
#define SPI_SR_TXC_MASK                 (1 << (SPI_SR_TXC_SHIFT))
#define SPI_SR_EOT_SHIFT                (3)
#define SPI_SR_EOT_MASK                 (1 << (SPI_SR_EOT_SHIFT))
#define SPI_SR_TXP_SHIFT                (1)
#define SPI_SR_TXP_MASK                 (1 << (SPI_SR_TXP_SHIFT))
#define SPI_SR_RXP_SHIFT                (0)
#define SPI_SR_RXP_MASK                 (1 << (SPI_SR_RXP_SHIFT))
  __IO uint32_t         IFCR;           /* 0x18 */
#define SPI_IFCR_CLEAR_ALL_SHIFT        (0)
#define SPI_IFCR_CLEAR_ALL              (0x0ff8 << (SPI_IFCR_CLEAR_ALL_SHIFT))
  uint32_t              reserved1c;     /* 0x1c */
  __IO uint32_t         TXDR;           /* 0x20 */
  uint32_t              reserved24[3];  /* 0x24 */
  __IO uint32_t         RXDR;           /* 0x30 */
  uint32_t              reserved34[3];  /* 0x34 */
  __IO uint32_t         CRCPOLY;        /* 0x40 */
  __IO uint32_t         TXCRC;          /* 0x44 */
  __IO uint32_t         RXCRC;          /* 0x48 */
  __IO uint32_t         UDRDR;          /* 0x4c */
  __IO uint32_t         I2CCFGR;        /* 0x50 */
} stm32Dev_SPI;


#endif
