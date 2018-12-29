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

#ifndef _STM32SPI16_H_
#define _STM32SPI16_H_


typedef struct {
  __IO uint16_t         CR1;            /* 0x00 */
#define SPI_CR1_BIDIMODE_SHIFT          (15)
#define SPI_CR1_BIDIMODE_MASK           (1 << (SPI_CR1_BIDIMODE_SHIFT))
#define SPI_CR1_BIDIMODE_YES            (0 << (SPI_CR1_BIDIMODE_SHIFT))
#define SPI_CR1_BIDIMODE_NO             (1 << (SPI_CR1_BIDIMODE_SHIFT))
#define SPI_CR1_BIDIOE_SHIFT            (14)
#define SPI_CR1_BIDIOE_MASK             (1 << (SPI_CR1_BIDIOE_SHIFT))
#define SPI_CR1_BIDIOE_NO               (0 << (SPI_CR1_BIDIOE_SHIFT))
#define SPI_CR1_BIDIOE_YES              (1 << (SPI_CR1_BIDIOE_SHIFT))
#define SPI_CR1_RXONLY_SHIFT            (10)
#define SPI_CR1_RXONLY_MASK             (1 << (SPI_CR1_RXONLY_SHIFT))
#define SPI_CR1_RXONLY_NO               (0 << (SPI_CR1_RXONLY_SHIFT))
#define SPI_CR1_RXONLY_YES              (1 << (SPI_CR1_RXONLY_SHIFT))
#define SPI_CR1_SSM_SHIFT               (9)
#define SPI_CR1_SSM_MASK                (1 << (SPI_CR1_SSM_SHIFT))
#define SPI_CR1_SSM_NO                  (0 << (SPI_CR1_SSM_SHIFT))
#define SPI_CR1_SSM_YES                 (1 << (SPI_CR1_SSM_SHIFT))
#define SPI_CR1_SSI_SHIFT               (8)
#define SPI_CR1_SSI_MASK                (1 << (SPI_CR1_SSI_SHIFT))
#define SPI_CR1_SSI_NO                  (0 << (SPI_CR1_SSI_SHIFT))
#define SPI_CR1_SSI_YES                 (1 << (SPI_CR1_SSI_SHIFT))
#define SPI_CR1_LSBFRST_SHIFT           (7)
#define SPI_CR1_LSBFRST_MASK            (1 << (SPI_CR1_LSBFRST_SHIFT))
#define SPI_CR1_LSBFRST_NO              (0 << (SPI_CR1_LSBFRST_SHIFT))
#define SPI_CR1_LSBFRST_YES             (1 << (SPI_CR1_LSBFRST_SHIFT))
#define SPI_CR1_SPE_SHIFT               (6)
#define SPI_CR1_SPE_MASK                (1 << (SPI_CR1_SPE_SHIFT))
#define SPI_CR1_SPE_NO                  (0 << (SPI_CR1_SPE_SHIFT))
#define SPI_CR1_SPE_YES                 (1 << (SPI_CR1_SPE_SHIFT))
#define SPI_CR1_BR_SHIFT                (3)
#define SPI_CR1_BR_MASK                 (7 << (SPI_CR1_BR_SHIFT))
#define SPI_CR1_BR_DIV2                 (0 << (SPI_CR1_BR_SHIFT))
#define SPI_CR1_BR_DIV4                 (1 << (SPI_CR1_BR_SHIFT))
#define SPI_CR1_BR_DIV8                 (2 << (SPI_CR1_BR_SHIFT))
#define SPI_CR1_BR_DIV16                (3 << (SPI_CR1_BR_SHIFT))
#define SPI_CR1_BR_DIV32                (4 << (SPI_CR1_BR_SHIFT))
#define SPI_CR1_BR_DIV64                (5 << (SPI_CR1_BR_SHIFT))
#define SPI_CR1_BR_DIV128               (6 << (SPI_CR1_BR_SHIFT))
#define SPI_CR1_BR_DIV256               (7 << (SPI_CR1_BR_SHIFT))
#define SPI_CR1_MSTR_SHIFT              (2)
#define SPI_CR1_MSTR_MASK               (1 << (SPI_CR1_MSTR_SHIFT))
#define SPI_CR1_MSTR_NO                 (0 << (SPI_CR1_MSTR_SHIFT))
#define SPI_CR1_MSTR_YES                (1 << (SPI_CR1_MSTR_SHIFT))
#define SPI_CR1_CPOL_SHIFT              (1)
#define SPI_CR1_CPOL_MASK               (1 << (SPI_CR1_CPOL_SHIFT))
#define SPI_CR1_CPOL_NO                 (0 << (SPI_CR1_CPOL_SHIFT))
#define SPI_CR1_CPOL_YES                (1 << (SPI_CR1_CPOL_SHIFT))
#define SPI_CR1_CPHA_SHIFT              (0)
#define SPI_CR1_CPHA_MASK               (1 << (SPI_CR1_CPHA_SHIFT))
#define SPI_CR1_CPHA_NO                 (0 << (SPI_CR1_CPHA_SHIFT))
#define SPI_CR1_CPHA_YES                (1 << (SPI_CR1_CPHA_SHIFT))
  uint16_t              reserved02;

  __IO uint16_t         CR2;            /* 0x04 */
#define SPI_CR2_FRXTH_SHIFT             (12)
#define SPI_CR2_FRXTH_MASK              (1 << (SPI_CR2_FRXTH_SHIFT))
#define SPI_CR2_FRXTH_8BIT              (0 << (SPI_CR2_FRXTH_SHIFT))
#define SPI_CR2_FRXTH_16BIT             (1 << (SPI_CR2_FRXTH_SHIFT))
#define SPI_CR2_DS_SHIFT                (8)
#define SPI_CR2_DS_MASK                 (15 << (SPI_CR2_DS_SHIFT))
#define SPI_CR2_DS_8BIT                 (7 << (SPI_CR2_DS_SHIFT))
#define SPI_CR2_DS_16BIT                (15 << (SPI_CR2_DS_SHIFT))
#define SPI_CR2_DS_VAL(x)               ((((x) - 1) << (SPI_CR2_DS_SHIFT)) & SPI_CR2_DS_MASK)
#define SPI_CR2_TXEIE_SHIFT             (7)
#define SPI_CR2_TXEIE_MASK              (1 << (SPI_CR2_TXEIE_SHIFT))
#define SPI_CR2_TXEIE_NO                (0 << (SPI_CR2_TXEIE_SHIFT))
#define SPI_CR2_TXEIE_YES               (1 << (SPI_CR2_TXEIE_SHIFT))
#define SPI_CR2_RXNEIE_SHIFT            (6)
#define SPI_CR2_RXNEIE_MASK             (1 << (SPI_CR2_RXNEIE_SHIFT))
#define SPI_CR2_RXNEIE_NO               (0 << (SPI_CR2_RXNEIE_SHIFT))
#define SPI_CR2_RXNEIE_YES              (1 << (SPI_CR2_RXNEIE_SHIFT))
#define SPI_CR2_ERRIE_SHIFT             (5)
#define SPI_CR2_ERRIE_MASK              (1 << (SPI_CR2_ERRIE_SHIFT))
#define SPI_CR2_ERRIE_NO                (0 << (SPI_CR2_ERRIE_SHIFT))
#define SPI_CR2_ERRIE_YES               (1 << (SPI_CR2_ERRIE_SHIFT))
#define SPI_CR2_FRF_SHIFT               (4)
#define SPI_CR2_FRF_MASK                (1 << (SPI_CR2_FRF_SHIFT))
#define SPI_CR2_FRF_MOTOROLA            (0 << (SPI_CR2_FRF_SHIFT))
#define SPI_CR2_FRF_TI                  (1 << (SPI_CR2_FRF_SHIFT))
#define SPI_CR2_NSSP_SHIFT              (3)
#define SPI_CR2_NSSP_MASK               (1 << (SPI_CR2_NSSP_SHIFT))
#define SPI_CR2_NSSP_NO                 (0 << (SPI_CR2_NSSP_SHIFT))
#define SPI_CR2_NSSP_YES                (1 << (SPI_CR2_NSSP_SHIFT))
#define SPI_CR2_SSOE_SHIFT              (2)
#define SPI_CR2_SSOE_MASK               (1 << (SPI_CR2_SSOE_SHIFT))
#define SPI_CR2_SSOE_NO                 (0 << (SPI_CR2_SSOE_SHIFT))
#define SPI_CR2_SSOE_YES                (1 << (SPI_CR2_SSOE_SHIFT))
#define SPI_CR2_TXDMAEN_SHIFT           (1)
#define SPI_CR2_TXDMAEN_MASK            (1 << (SPI_CR2_TXDMAEN_SHIFT))
#define SPI_CR2_TXDMAEN_NO              (0 << (SPI_CR2_TXDMAEN_SHIFT))
#define SPI_CR2_TXDMAEN_YES             (1 << (SPI_CR2_TXDMAEN_SHIFT))
#define SPI_CR2_RXDMAEN_SHIFT           (0)
#define SPI_CR2_RXDMAEN_MASK            (1 << (SPI_CR2_RXDMAEN_SHIFT))
#define SPI_CR2_RXDMAEN_NO              (0 << (SPI_CR2_RXDMAEN_SHIFT))
#define SPI_CR2_RXDMAEN_YES             (1 << (SPI_CR2_RXDMAEN_SHIFT))
  uint16_t              reserved06;

  __IO uint16_t         SR;             /* 0x08 */
#define SPI_SR_FTLVL_SHIFT              (11)
#define SPI_SR_FTLVL_MASK               (3 << (SPI_SR_FTLVL_SHIFT))
#define SPI_SR_FTLVL_EMPTY              (0 << (SPI_SR_FTLVL_SHIFT))
#define SPI_SR_FTLVL_QUATER             (1 << (SPI_SR_FTLVL_SHIFT))
#define SPI_SR_FTLVL_HALF               (2 << (SPI_SR_FTLVL_SHIFT))
#define SPI_SR_FTLVL_FULL               (3 << (SPI_SR_FTLVL_SHIFT))
#define SPI_SR_FRLVL_SHIFT              (9)
#define SPI_SR_FRLVL_MASK               (3 << (SPI_SR_FRLVL_SHIFT))
#define SPI_SR_FRLVL_EMPTY              (0 << (SPI_SR_FRLVL_SHIFT))
#define SPI_SR_FRLVL_QUATER             (1 << (SPI_SR_FRLVL_SHIFT))
#define SPI_SR_FRLVL_HALF               (2 << (SPI_SR_FRLVL_SHIFT))
#define SPI_SR_FRLVL_FULL               (3 << (SPI_SR_FRLVL_SHIFT))
#define SPI_SR_FRE_SHIFT                (8)
#define SPI_SR_FRE_MASK                 (1 << (SPI_SR_FRE_SHIFT))
#define SPI_SR_FRE_NO                   (0 << (SPI_SR_FRE_SHIFT))
#define SPI_SR_FRE_YES                  (1 << (SPI_SR_FRE_SHIFT))
#define SPI_SR_BSY_SHIFT                (7)
#define SPI_SR_BSY_MASK                 (1 << (SPI_SR_BSY_SHIFT))
#define SPI_SR_BSY_NO                   (0 << (SPI_SR_BSY_SHIFT))
#define SPI_SR_BSY_YES                  (1 << (SPI_SR_BSY_SHIFT))
#define SPI_SR_OVR_SHIFT                (6)
#define SPI_SR_OVR_MASK                 (1 << (SPI_SR_OVR_SHIFT))
#define SPI_SR_OVR_NO                   (0 << (SPI_SR_OVR_SHIFT))
#define SPI_SR_OVR_YES                  (1 << (SPI_SR_OVR_SHIFT))
#define SPI_SR_MODF_SHIFT               (5)
#define SPI_SR_MODF_MASK                (1 << (SPI_SR_MODF_SHIFT))
#define SPI_SR_MODF_NO                  (0 << (SPI_SR_MODF_SHIFT))
#define SPI_SR_MODF_YES                 (1 << (SPI_SR_MODF_SHIFT))
#define SPI_SR_CRCERR_SHIFT             (4)
#define SPI_SR_CRCERR_MASK              (1 << (SPI_SR_CRCERR_SHIFT))
#define SPI_SR_CRCERR_NO                (0 << (SPI_SR_CRCERR_SHIFT))
#define SPI_SR_CRCERR_YES               (1 << (SPI_SR_CRCERR_SHIFT))
#define SPI_SR_TXE_SHIFT                (1)
#define SPI_SR_TXE_MASK                 (1 << (SPI_SR_TXE_SHIFT))
#define SPI_SR_TXE_NO                   (0 << (SPI_SR_TXE_SHIFT))
#define SPI_SR_TXE_YES                  (1 << (SPI_SR_TXE_SHIFT))
#define SPI_SR_RXNE_SHIFT               (0)
#define SPI_SR_RXNE_MASK                (1 << (SPI_SR_RXNE_SHIFT))
#define SPI_SR_RXNE_NO                  (0 << (SPI_SR_RXNE_SHIFT))
#define SPI_SR_RXNE_YES                 (1 << (SPI_SR_RXNE_SHIFT))
  uint16_t              reserved0a;

  __IO uint16_t         DR;             /* 0x0c */
  uint16_t              reserved0e;

  __IO uint16_t         CRCPR;          /* 0x10 */
  uint16_t              reserved12;

  __IO uint16_t         RXCRCR;         /* 0x14 */
  uint16_t              reserved16;

  __IO uint16_t         TXCRCR;         /* 0x18 */

} stm32Dev_SPI;


#endif
