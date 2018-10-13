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

#ifndef _STM32I2C_H_
#define _STM32I2C_H_

typedef struct {
  __IO uint32_t         CR1;            /* 0x00 */
#define I2C_CR1_NOSTRETCH_SHIFT         17
#define I2C_CR1_NOSTRETCH_MASK          (1 << (I2C_CR1_NOSTRETCH_SHIFT))
#define I2C_CR1_NOSTRETCH_NO            (0 << (I2C_CR1_NOSTRETCH_SHIFT))
#define I2C_CR1_NOSTRETCH_YES           (1 << (I2C_CR1_NOSTRETCH_SHIFT))
#define I2C_CR1_RXDMAEN_SHIFT           15
#define I2C_CR1_RXDMAEN_MASK            (1 << (I2C_CR1_RXDMAEN_SHIFT))
#define I2C_CR1_RXDMAEN_NO              (0 << (I2C_CR1_RXDMAEN_SHIFT))
#define I2C_CR1_RXDMAEN_YES             (1 << (I2C_CR1_RXDMAEN_SHIFT))
#define I2C_CR1_TXDMAEN_SHIFT           14
#define I2C_CR1_TXDMAEN_MASK            (1 << (I2C_CR1_TXDMAEN_SHIFT))
#define I2C_CR1_TXDMAEN_NO              (0 << (I2C_CR1_TXDMAEN_SHIFT))
#define I2C_CR1_TXDMAEN_YES             (1 << (I2C_CR1_TXDMAEN_SHIFT))
#define I2C_CR1_ANFOFF_SHIFT            12
#define I2C_CR1_ANFOFF_MASK             (1 << (I2C_CR1_ANFOFF_SHIFT))
#define I2C_CR1_ANFOFF_NO               (0 << (I2C_CR1_ANFOFF_SHIFT))
#define I2C_CR1_ANFOFF_YES              (1 << (I2C_CR1_ANFOFF_SHIFT))
#define I2C_CR1_DNF_SHIFT               8
#define I2C_CR1_DNF_MASK                (0xf << (I2C_CR1_DNF_SHIFT))
#define I2C_CR1_DNF_VAL(x)              (((x) << (I2C_CR1_DNF_SHIFT)) & I2C_CR1_DNF_MASK)
#define I2C_CR1_ERRIE_SHIFT             7
#define I2C_CR1_ERRIE_MASK              (1 << (I2C_CR1_ERRIE_SHIFT))
#define I2C_CR1_ERRIE_NO                (0 << (I2C_CR1_ERRIE_SHIFT))
#define I2C_CR1_ERRIE_YES               (1 << (I2C_CR1_ERRIE_SHIFT))
#define I2C_CR1_TCIE_SHIFT              6
#define I2C_CR1_TCIE_MASK               (1 << (I2C_CR1_TCIE_SHIFT))
#define I2C_CR1_TCIE_NO                 (0 << (I2C_CR1_TCIE_SHIFT))
#define I2C_CR1_TCIE_YES                (1 << (I2C_CR1_TCIE_SHIFT))
#define I2C_CR1_STOPIE_SHIFT            5
#define I2C_CR1_STOPIE_MASK             (1 << (I2C_CR1_STOPIE_SHIFT))
#define I2C_CR1_STOPIE_NO               (0 << (I2C_CR1_STOPIE_SHIFT))
#define I2C_CR1_STOPIE_YES              (1 << (I2C_CR1_STOPIE_SHIFT))
#define I2C_CR1_NACKIE_SHIFT            4
#define I2C_CR1_NACKIE_MASK             (1 << (I2C_CR1_NACKIE_SHIFT))
#define I2C_CR1_NACKIE_NO               (0 << (I2C_CR1_NACKIE_SHIFT))
#define I2C_CR1_NACKIE_YES              (1 << (I2C_CR1_NACKIE_SHIFT))
#define I2C_CR1_ADDRIE_SHIFT            3
#define I2C_CR1_ADDRIE_MASK             (1 << (I2C_CR1_ADDRIE_SHIFT))
#define I2C_CR1_ADDRIE_NO               (0 << (I2C_CR1_ADDRIE_SHIFT))
#define I2C_CR1_ADDRIE_YES              (1 << (I2C_CR1_ADDRIE_SHIFT))
#define I2C_CR1_RXIE_SHIFT              2
#define I2C_CR1_RXIE_MASK               (1 << (I2C_CR1_RXIE_SHIFT))
#define I2C_CR1_RXIE_NO                 (0 << (I2C_CR1_RXIE_SHIFT))
#define I2C_CR1_RXIE_YES                (1 << (I2C_CR1_RXIE_SHIFT))
#define I2C_CR1_TXIE_SHIFT              1
#define I2C_CR1_TXIE_MASK               (1 << (I2C_CR1_TXIE_SHIFT))
#define I2C_CR1_TXIE_NO                 (0 << (I2C_CR1_TXIE_SHIFT))
#define I2C_CR1_TXIE_YES                (1 << (I2C_CR1_TXIE_SHIFT))
#define I2C_CR1_PE_SHIFT                0
#define I2C_CR1_PE_MASK                 (1 << (I2C_CR1_PE_SHIFT))
#define I2C_CR1_PE_NO                   (0 << (I2C_CR1_PE_SHIFT))
#define I2C_CR1_PE_YES                  (1 << (I2C_CR1_PE_SHIFT))

  __IO uint32_t         CR2;            /* 0x04 */
#define I2C_CR2_AUTOEND_SHIFT           25
#define I2C_CR2_AUTOEND_MASK            (1 << (I2C_CR2_AUTOEND_SHIFT))
#define I2C_CR2_AUTOEND_NO              (0 << (I2C_CR2_AUTOEND_SHIFT))
#define I2C_CR2_AUTOEND_YES             (1 << (I2C_CR2_AUTOEND_SHIFT))
#define I2C_CR2_RELOAD_SHIFT            24
#define I2C_CR2_RELOAD_MASK             (1 << (I2C_CR2_RELOAD_SHIFT))
#define I2C_CR2_RELOAD_NO               (0 << (I2C_CR2_RELOAD_SHIFT))
#define I2C_CR2_RELOAD_YES              (1 << (I2C_CR2_RELOAD_SHIFT))
#define I2C_CR2_NBYTES_SHIFT            16
#define I2C_CR2_NBYTES_MASK             (0xff << (I2C_CR2_NBYTES_SHIFT))
#define I2C_CR2_NBYTES_VAL(x)           (((x) << (I2C_CR2_NBYTES_SHIFT)) & I2C_CR2_NBYTES_MASK)
#define I2C_CR2_NACK_SHIFT              15
#define I2C_CR2_NACK_MASK               (1 << (I2C_CR2_NACK_SHIFT))
#define I2C_CR2_NACK                    (1 << (I2C_CR2_NACK_SHIFT))
#define I2C_CR2_STOP_SHIFT              14
#define I2C_CR2_STOP_MASK               (1 << (I2C_CR2_STOP_SHIFT))
#define I2C_CR2_STOP                    (1 << (I2C_CR2_STOP_SHIFT))
#define I2C_CR2_START_SHIFT             13
#define I2C_CR2_START_MASK              (1 << (I2C_CR2_START_SHIFT))
#define I2C_CR2_START                   (1 << (I2C_CR2_START_SHIFT))
#define I2C_CR2_RDWRN_SHIFT             10
#define I2C_CR2_RDWRN_MASK              (1 << (I2C_CR2_RDWRN_SHIFT))
#define I2C_CR2_RDWRN_WRN               (0 << (I2C_CR2_RDWRN_SHIFT))
#define I2C_CR2_RDWRN_RD                (1 << (I2C_CR2_RDWRN_SHIFT))
#define I2C_CR2_SADD_SHIFT              0
#define I2C_CR2_SADD_MASK               (0x3ff << (I2C_CR2_SADD_SHIFT))
#define I2C_CR2_SADD_VAL(x)             (((x) << (I2C_CR2_SADD_SHIFT)) & I2C_CR2_SADD_MASK)

  __IO uint32_t         OAR1;           /* 0x08 */
  __IO uint32_t         OAR2;           /* 0x0c */

  __IO uint32_t         TIMINGR;        /* 0x10 */
#define I2C_TIMINGR_PRESC_SHIFT         28
#define I2C_TIMINGR_PRESC_MASK          (0xf << (I2C_TIMINGR_PRESC_SHIFT))
#define I2C_TIMINGR_PRESC_VAL(x)        (((x) << (I2C_TIMINGR_PRESC_SHIFT)) & I2C_TIMINGR_PRESC_MASK)
#define I2C_TIMINGR_SCLDEL_SHIFT        20
#define I2C_TIMINGR_SCLDEL_MASK         (0xf << (I2C_TIMINGR_SCLDEL_SHIFT))
#define I2C_TIMINGR_SCLDEL_VAL(x)       (((x) << (I2C_TIMINGR_SCLDEL_SHIFT)) & I2C_TIMINGR_SCLDEL_MASK)
#define I2C_TIMINGR_SDADEL_SHIFT        16
#define I2C_TIMINGR_SDADEL_MASK         (0xf << (I2C_TIMINGR_SDADEL_SHIFT))
#define I2C_TIMINGR_SDADEL_VAL(x)       (((x) << (I2C_TIMINGR_SDADEL_SHIFT)) & I2C_TIMINGR_SDADEL_MASK)
#define I2C_TIMINGR_SCLH_SHIFT          8
#define I2C_TIMINGR_SCLH_MASK           (0xff << (I2C_TIMINGR_SCLH_SHIFT))
#define I2C_TIMINGR_SCLH_VAL(x)         (((x) << (I2C_TIMINGR_SCLH_SHIFT)) & I2C_TIMINGR_SCLH_MASK)
#define I2C_TIMINGR_SCLL_SHIFT          0
#define I2C_TIMINGR_SCLL_MASK           (0xff << (I2C_TIMINGR_SCLL_SHIFT))
#define I2C_TIMINGR_SCLL_VAL(x)         (((x) << (I2C_TIMINGR_SCLL_SHIFT)) & I2C_TIMINGR_SCLL_MASK)

  __IO uint32_t         TIMEOUTR;       /* 0x14 */

  __IO uint32_t         ISR;            /* 0x18 */
  __IO uint32_t         ICR;            /* 0x1c */
#define I2C_ISR_ALLERR_MASK             0x3f00
#define I2C_ISR_BUSY_SHIFT              15
#define I2C_ISR_BUSY_MASK               (1 << (I2C_ISR_BUSY_SHIFT))
#define I2C_ISR_TCR_SHIFT               7
#define I2C_ISR_TCR_MASK                (1 << (I2C_ISR_TCR_SHIFT))
#define I2C_ISR_TC_SHIFT                6
#define I2C_ISR_TC_MASK                 (1 << (I2C_ISR_TC_SHIFT))
#define I2C_ISR_STOPF_SHIFT             5
#define I2C_ISR_STOPF_MASK              (1 << (I2C_ISR_STOPF_SHIFT))
#define I2C_ISR_NACKF_SHIFT             4
#define I2C_ISR_NACKF_MASK              (1 << (I2C_ISR_NACKF_SHIFT))
#define I2C_ISR_ADDR_SHIFT              3
#define I2C_ISR_ADDR_MASK               (1 << (I2C_ISR_ADDR_SHIFT))
#define I2C_ISR_RXNE_SHIFT              2
#define I2C_ISR_RXNE_MASK               (1 << (I2C_ISR_RXNE_SHIFT))
#define I2C_ISR_TXIS_SHIFT              1
#define I2C_ISR_TXIS_MASK               (1 << (I2C_ISR_TXIS_SHIFT))
#define I2C_ISR_TXE_SHIFT               0
#define I2C_ISR_TXE_MASK                (1 << (I2C_ISR_TXE_SHIFT))

  __IO uint32_t         PECR;           /* 0x20 */
  __IO uint32_t         RXDR;           /* 0x24 */
  __IO uint32_t         TXDR;           /* 0x28 */

  uint32_t              reserved2c[245];   /* 0x2c */
} stm32Dev_I2C;

#endif
