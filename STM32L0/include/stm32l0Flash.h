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

#ifndef _STM32L0FLASH_H_
#define _STM32L0FLASH_H_

typedef struct {
  __IO uint32_t         ACR;            /* 0x00 */
#define FLASH_ACR_LATENCY_SHIFT         0
#define FLASH_ACR_LATENCY_MASK          (1 << (FLASH_ACR_LATENCY_SHIFT))
#define FLASH_ACR_LATENCY_NO            (0 << (FLASH_ACR_LATENCY_SHIFT))
#define FLASH_ACR_LATENCY_YES           (1 << (FLASH_ACR_LATENCY_SHIFT))
#define FLASH_ACR_PRFTEN_SHIFT          2
#define FLASH_ACR_PRFTEN_MASK           (1 << (FLASH_ACR_PRFTEN_SHIFT))
#define FLASH_ACR_PRFTEN_NO             (0 << (FLASH_ACR_PRFTEN_SHIFT))
#define FLASH_ACR_PRFTEN_YES            (1 << (FLASH_ACR_PRFTEN_SHIFT))
#define FLASH_ACR_SLEEP_PD_SHIFT        3
#define FLASH_ACR_SLEEP_PD_MASK         (1 << (FLASH_ACR_SLEEP_PD_SHIFT))
#define FLASH_ACR_SLEEP_PD_NO           (0 << (FLASH_ACR_SLEEP_PD_SHIFT))
#define FLASH_ACR_SLEEP_PD_YES          (1 << (FLASH_ACR_SLEEP_PD_SHIFT))
#define FLASH_ACR_RUN_PD_SHIFT          4
#define FLASH_ACR_RUN_PD_MASK           (1 << (FLASH_ACR_RUN_PD_SHIFT))
#define FLASH_ACR_RUN_PD_NO             (0 << (FLASH_ACR_RUN_PD_SHIFT))
#define FLASH_ACR_RUN_PD_YES            (1 << (FLASH_ACR_RUN_PD_SHIFT))
#define FLASH_ACR_DESAB_8BUF_SHIFT      5
#define FLASH_ACR_DESAB_8BUF_MASK       (1 << (FLASH_ACR_DESAB_8BUF_SHIFT))
#define FLASH_ACR_DESAB_8BUF_NO         (0 << (FLASH_ACR_DESAB_8BUF_SHIFT))
#define FLASH_ACR_DESAB_8BUF_YES        (1 << (FLASH_ACR_DESAB_8BUF_SHIFT))
#define FLASH_ACR_PRE_READ_SHIFT        6
#define FLASH_ACR_PRE_READ_MASK         (1 << (FLASH_ACR_PRE_READ_SHIFT))
#define FLASH_ACR_PRE_READ_NO           (0 << (FLASH_ACR_PRE_READ_SHIFT))
#define FLASH_ACR_PRE_READ_YES          (1 << (FLASH_ACR_PRE_READ_SHIFT))

  __IO uint32_t         PECR;           /* 0x04 */
#define FLASH_PECR_NZDISABLE_SHIFT      23
#define FLASH_PECR_NZDISABLE_MASK       (1 << (FLASH_PECR_NZDISABLE_SHIFT))
#define FLASH_PECR_NZDISABLE_NO         (0 << (FLASH_PECR_NZDISABLE_SHIFT))
#define FLASH_PECR_NZDISABLE_YES        (1 << (FLASH_PECR_NZDISABLE_SHIFT))
#define FLASH_PECR_OBL_LAUNCH_SHIFT     18
#define FLASH_PECR_OBL_LAUNCH_MASK      (1 << (FLASH_PECR_OBL_LAUNCH_SHIFT))
#define FLASH_PECR_OBL_LAUNCH_NO        (0 << (FLASH_PECR_OBL_LAUNCH_SHIFT))
#define FLASH_PECR_OBL_LAUNCH_YES       (1 << (FLASH_PECR_OBL_LAUNCH_SHIFT))
#define FLASH_PECR_ERRIE_SHIFT          17
#define FLASH_PECR_ERRIE_MASK           (1 << (FLASH_PECR_ERRIE_SHIFT))
#define FLASH_PECR_ERRIE_NO             (0 << (FLASH_PECR_ERRIE_SHIFT))
#define FLASH_PECR_ERRIE_YES            (1 << (FLASH_PECR_ERRIE_SHIFT))
#define FLASH_PECR_EOPIE_SHIFT          16
#define FLASH_PECR_EOPIE_MASK           (1 << (FLASH_PECR_EOPIE_SHIFT))
#define FLASH_PECR_EOPIE_NO             (0 << (FLASH_PECR_EOPIE_SHIFT))
#define FLASH_PECR_EOPIE_YES            (1 << (FLASH_PECR_EOPIE_SHIFT))
#define FLASH_PECR_PARALLELBANK_SHIFT   15
#define FLASH_PECR_PARALLELBANK_MASK    (1 << (FLASH_PECR_PARALLELBANK_SHIFT))
#define FLASH_PECR_PARALLELBANK_NO      (0 << (FLASH_PECR_PARALLELBANK_SHIFT))
#define FLASH_PECR_PARALLELBANK_YES     (1 << (FLASH_PECR_PARALLELBANK_SHIFT))
#define FLASH_PECR_FPRG_SHIFT           10
#define FLASH_PECR_FPRG_MASK            (1 << (FLASH_PECR_FPRG_SHIFT))
#define FLASH_PECR_FPRG_NO              (0 << (FLASH_PECR_FPRG_SHIFT))
#define FLASH_PECR_FPRG_YES             (1 << (FLASH_PECR_FPRG_SHIFT))
#define FLASH_PECR_ERASE_SHIFT          9
#define FLASH_PECR_ERASE_MASK           (1 << (FLASH_PECR_ERASE_SHIFT))
#define FLASH_PECR_ERASE_NO             (0 << (FLASH_PECR_ERASE_SHIFT))
#define FLASH_PECR_ERASE_YES            (1 << (FLASH_PECR_ERASE_SHIFT))
#define FLASH_PECR_FIX_SHIFT            8
#define FLASH_PECR_FIX_MASK             (1 << (FLASH_PECR_FIX_SHIFT))
#define FLASH_PECR_FIX_NO               (0 << (FLASH_PECR_FIX_SHIFT))
#define FLASH_PECR_FIX_YES              (1 << (FLASH_PECR_FIX_SHIFT))
#define FLASH_PECR_DATA_SHIFT           4
#define FLASH_PECR_DATA_MASK            (1 << (FLASH_PECR_DATA_SHIFT))
#define FLASH_PECR_DATA_NO              (0 << (FLASH_PECR_DATA_SHIFT))
#define FLASH_PECR_DATA_YES             (1 << (FLASH_PECR_DATA_SHIFT))
#define FLASH_PECR_PROG_SHIFT           3
#define FLASH_PECR_PROG_MASK            (1 << (FLASH_PECR_PROG_SHIFT))
#define FLASH_PECR_PROG_NO              (0 << (FLASH_PECR_PROG_SHIFT))
#define FLASH_PECR_PROG_YES             (1 << (FLASH_PECR_PROG_SHIFT))
#define FLASH_PECR_OPTLOCK_SHIFT        2
#define FLASH_PECR_OPTLOCK_MASK         (1 << (FLASH_PECR_OPTLOCK_SHIFT))
#define FLASH_PECR_OPTLOCK_NO           (0 << (FLASH_PECR_OPTLOCK_SHIFT))
#define FLASH_PECR_OPTLOCK_YES          (1 << (FLASH_PECR_OPTLOCK_SHIFT))
#define FLASH_PECR_PRGLOCK_SHIFT        1
#define FLASH_PECR_PRGLOCK_MASK         (1 << (FLASH_PECR_PRGLOCK_SHIFT))
#define FLASH_PECR_PRGLOCK_NO           (0 << (FLASH_PECR_PRGLOCK_SHIFT))
#define FLASH_PECR_PRGLOCK_YES          (1 << (FLASH_PECR_PRGLOCK_SHIFT))
#define FLASH_PECR_PELOCK_SHIFT         0
#define FLASH_PECR_PELOCK_MASK          (1 << (FLASH_PECR_PELOCK_SHIFT))
#define FLASH_PECR_PELOCK_NO            (0 << (FLASH_PECR_PELOCK_SHIFT))
#define FLASH_PECR_PELOCK_YES           (1 << (FLASH_PECR_PELOCK_SHIFT))

  __IO uint32_t         PDKEYR;         /* 0x08 */
#define FLASH_PDKEY1            0x04152637
#define FLASH_PDKEY2            0xfafbfcfd

  __IO uint32_t         PKEYR;          /* 0x0c */
#define FLASH_PKEY1             0x89abcdef
#define FLASH_PKEY2             0x02030405

  __IO uint32_t         PRGKEYR;        /* 0x10 */
#define FLASH_PRGKEY1           0x8c9daebf
#define FLASH_PRGKEY2           0x13141516

  __IO uint32_t         OPTKEYR;        /* 0x14 */

  __IO uint32_t         SR;             /* 0x18 */
#define FLASH_SR_FWWERR_SHIFT           17
#define FLASH_SR_FWWERR_MASK            (1 << (FLASH_SR_FWWERR_SHIFT))
#define FLASH_SR_NOTZEROERR_SHIFT       16
#define FLASH_SR_NOTZEROERR_MASK        (1 << (FLASH_SR_NOTZEROERR_SHIFT))
#define FLASH_SR_RDERR_SHIFT            13
#define FLASH_SR_RDERR_MASK             (1 << (FLASH_SR_RDERR_SHIFT))
#define FLASH_SR_OPTVERR_SHIFT          11
#define FLASH_SR_OPTVERR_MASK           (1 << (FLASH_SR_OPTVERR_SHIFT))
#define FLASH_SR_SIZERR_SHIFT           10
#define FLASH_SR_SIZERR_MASK            (1 << (FLASH_SR_SIZERR_SHIFT))
#define FLASH_SR_PGAERR_SHIFT           9
#define FLASH_SR_PGAERR_MASK            (1 << (FLASH_SR_PGAERR_SHIFT))
#define FLASH_SR_WRPERR_SHIFT           8
#define FLASH_SR_WRPERR_MASK            (1 << (FLASH_SR_WRPERR_SHIFT))
#define FLASH_SR_READY_SHIFT            3
#define FLASH_SR_READY_MASK             (1 << (FLASH_SR_READY_SHIFT))
#define FLASH_SR_ENDHV_SHIFT            2
#define FLASH_SR_ENDHV_MASK             (1 << (FLASH_SR_ENDHV_SHIFT))
#define FLASH_SR_EOP_SHIFT              1
#define FLASH_SR_EOP_MASK               (1 << (FLASH_SR_EOP_SHIFT))
#define FLASH_SR_BSY_SHIFT              0
#define FLASH_SR_BSY_MASK               (1 << (FLASH_SR_BSY_SHIFT))

  __IO uint32_t         OPTR;           /* 0x1c */
  __IO uint32_t         WRPROT1;        /* 0x20 */
  __IO uint32_t         WRPROT2;        /* 0x80 */

} stm32Dev_FLASH;


#endif
