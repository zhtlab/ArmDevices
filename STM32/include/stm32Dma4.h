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

#ifndef _STM32DMA4_H_
#define _STM32DMA4_H_

typedef struct {
  __IO uint32_t CCR;
#define DMA_CCR_MEM2MEM_SHIFT       14
#define DMA_CCR_MEM2MEM_MASK        (1 << (DMA_CCR_MEM2MEM_SHIFT))
#define DMA_CCR_MEM2MEM_NO          (0 << (DMA_CCR_MEM2MEM_SHIFT))
#define DMA_CCR_MEM2MEM_YES         (1 << (DMA_CCR_MEM2MEM_SHIFT))
#define DMA_CCR_PL_SHIFT            12
#define DMA_CCR_PL_MASK             (1 << (DMA_CCR_PL_SHIFT))
#define DMA_CCR_PL_LOW              (0 << (DMA_CCR_PL_SHIFT))
#define DMA_CCR_PL_MID              (1 << (DMA_CCR_PL_SHIFT))
#define DMA_CCR_PL_HID              (2 << (DMA_CCR_PL_SHIFT))
#define DMA_CCR_PL_VERYHIGH         (3 << (DMA_CCR_PL_SHIFT))
#define DMA_CCR_MSIZE_SHIFT         10
#define DMA_CCR_MSIZE_MASK          (1 << (DMA_CCR_MSIZE_SHIFT))
#define DMA_CCR_MSIZE_8BITS         (0 << (DMA_CCR_MSIZE_SHIFT))
#define DMA_CCR_MSIZE_16BITS        (1 << (DMA_CCR_MSIZE_SHIFT))
#define DMA_CCR_MSIZE_32BITS        (2 << (DMA_CCR_MSIZE_SHIFT))
#define DMA_CCR_MSIZE_RESERVED      (3 << (DMA_CCR_MSIZE_SHIFT))
#define DMA_CCR_PSIZE_SHIFT         8
#define DMA_CCR_PSIZE_MASK          (1 << (DMA_CCR_PSIZE_SHIFT))
#define DMA_CCR_PSIZE_8BITS         (0 << (DMA_CCR_PSIZE_SHIFT))
#define DMA_CCR_PSIZE_16BITS        (1 << (DMA_CCR_PSIZE_SHIFT))
#define DMA_CCR_PSIZE_32BITS        (2 << (DMA_CCR_PSIZE_SHIFT))
#define DMA_CCR_PSIZE_RESERVED      (3 << (DMA_CCR_PL_SHIFT))
#define DMA_CCR_MINC_SHIFT          7
#define DMA_CCR_MINC_MASK           (1 << (DMA_CCR_MINC_SHIFT))
#define DMA_CCR_MINC_NO             (0 << (DMA_CCR_MINC_SHIFT))
#define DMA_CCR_MINC_YES            (1 << (DMA_CCR_MINC_SHIFT))
#define DMA_CCR_PINC_SHIFT          6
#define DMA_CCR_PINC_MASK           (1 << (DMA_CCR_PINC_SHIFT))
#define DMA_CCR_PINC_NO             (0 << (DMA_CCR_PINC_SHIFT))
#define DMA_CCR_PINC_YES            (1 << (DMA_CCR_PINC_SHIFT))
#define DMA_CCR_CIRC_SHIFT          5
#define DMA_CCR_CIRC_MASK           (1 << (DMA_CCR_CIRC_SHIFT))
#define DMA_CCR_CIRC_NO             (0 << (DMA_CCR_CIRC_SHIFT))
#define DMA_CCR_CIRC_YES            (1 << (DMA_CCR_CIRC_SHIFT))
#define DMA_CCR_DIR_SHIFT           4
#define DMA_CCR_DIR_MASK            (1 << (DMA_CCR_DIR_SHIFT))
#define DMA_CCR_DIR_READ_PERI       (0 << (DMA_CCR_DIR_SHIFT))
#define DMA_CCR_DIR_READ_MEM        (1 << (DMA_CCR_DIR_SHIFT))
#define DMA_CCR_TEIE_SHIFT          3
#define DMA_CCR_TEIE_MASK           (1 << (DMA_CCR_TEIE_SHIFT))
#define DMA_CCR_TEIE_NO             (0 << (DMA_CCR_TEIE_SHIFT))
#define DMA_CCR_TEIE_YES            (1 << (DMA_CCR_TEIE_SHIFT))
#define DMA_CCR_HTIE_SHIFT          2
#define DMA_CCR_HTIE_MASK           (1 << (DMA_CCR_HTIE_SHIFT))
#define DMA_CCR_HTIE_NO             (0 << (DMA_CCR_HTIE_SHIFT))
#define DMA_CCR_HTIE_YES            (1 << (DMA_CCR_HTIE_SHIFT))
#define DMA_CCR_TCIE_SHIFT          1
#define DMA_CCR_TCIE_MASK           (1 << (DMA_CCR_TCIE_SHIFT))
#define DMA_CCR_TCIE_NO             (0 << (DMA_CCR_TCIE_SHIFT))
#define DMA_CCR_TCIE_YES            (1 << (DMA_CCR_TCIE_SHIFT))
#define DMA_CCR_EN_SHIFT            0
#define DMA_CCR_EN_MASK             (1 << (DMA_CCR_EN_SHIFT))
#define DMA_CCR_EN_NO               (0 << (DMA_CCR_EN_SHIFT))
#define DMA_CCR_EN_YES              (1 << (DMA_CCR_EN_SHIFT))

  __IO uint32_t CNDTR;
#define DMA_CNDTR_MASK              (0xffff)
  __IO uint32_t CPAR;
  __IO uint32_t CMAR;
  __IO uint32_t reserved;
} stm32Dev_DMACH;

typedef struct {
  __IO uint32_t ISR;
#define DMA_ISR_TEIF_MASK(d)        (1 << ((((d)-1)<<2) + 3))
#define DMA_ISR_HTIF_MASK(d)        (1 << ((((d)-1)<<2) + 2))
#define DMA_ISR_TCIF_MASK(d)        (1 << ((((d)-1)<<2) + 1))
#define DMA_ISR_GIF_MASK(d)         (1 << ((((d)-1)<<2) + 0))

  __IO uint32_t IFCR;
#define DMA_ISR_CTEIF_CLEAR(d)      (1 << ((((d)-1)<<2) + 3))
#define DMA_ISR_CHTIF_CLEAR(d)      (1 << ((((d)-1)<<2) + 2))
#define DMA_ISR_CTCIF_CLEAR(d)      (1 << ((((d)-1)<<2) + 1))
#define DMA_ISR_CGIF_CLEAR(d)       (1 << ((((d)-1)<<2) + 0))
#define DMA_ISR_ALL_CLEAR(d)        (15 << ((((d)-1)<<2) + 0))

  stm32Dev_DMACH        CH[7];
  __IO uint32_t reserved90[5];
  __IO uint32_t CSELR;
#define DMA_CSELR_DMA_MASK(d)       (0xfUL << (((d)-1)*4))
#define DMA_CSELR_DMA_SEL(d, x)     ((x)   << (((d)-1)*4))
  __IO uint32_t reservedAC[213];
} stm32Dev_DMA;

#endif
