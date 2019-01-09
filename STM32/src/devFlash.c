/*
 * Copyright (c) 2018,2019 zhtlab
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

#define _DEV_FLASH_C_


#include        "system.h"

#include        "devFlash.h"

#define FLASH_PROGRAM_SIGNLE_WORD_EN    1
#define FLASH_PROGRAM_HALF_WORD_EN      0



int
DevFlashInit(int unit)
{
  return 0;
}


int
DevFlashErase(int unit, uint32_t addr, int size)
{
  __IO uint32_t         *pAddr32;
  int                   remain;

  /* check busy and set the unlock keys */
  while(FLASH_PTR->SR & FLASH_SR_BSY_MASK);
  FLASH_PTR->SR = FLASH_PTR->SR;
  if(FLASH_PTR->PECR & FLASH_PECR_PELOCK_MASK) {
    FLASH_PTR->PKEYR = FLASH_PKEY1;
    FLASH_PTR->PKEYR = FLASH_PKEY2;
  }
  if(FLASH_PTR->PECR & FLASH_PECR_PRGLOCK_MASK) {
    FLASH_PTR->PRGKEYR = FLASH_PRGKEY1;
    FLASH_PTR->PRGKEYR = FLASH_PRGKEY2;
  }

  FLASH_PTR->PECR |= FLASH_PECR_ERASE_YES;
  FLASH_PTR->PECR |= FLASH_PECR_PROG_YES;             /* adhoc need ??? */

  /* erase */
  remain = (size + FLASH_PAGE_SIZE -1) & (FLASH_PAGE_SIZE-1);
  while(remain > 0) {
    pAddr32 = (__IO uint32_t *) addr;
    *pAddr32 = 0;
    addr += FLASH_PAGE_SIZE;
    remain -= FLASH_PAGE_SIZE;
    while(FLASH_PTR->SR & FLASH_SR_BSY_MASK);
  }

  FLASH_PTR->PECR = 0;

  return 0;
}


int
DevFlashProgram(int unit, uint32_t addr, int size, uint8_t *pBuf)
{
  uint32_t              val;
  uint8_t               *p;
  int                   cnt;
  __IO uint32_t         *pAddr32;

  while(FLASH_PTR->SR & FLASH_SR_BSY_MASK);
  FLASH_PTR->SR = FLASH_PTR->SR;
  if(FLASH_PTR->PECR & FLASH_PECR_PELOCK_MASK) {
    FLASH_PTR->PKEYR = FLASH_PKEY1;
    FLASH_PTR->PKEYR = FLASH_PKEY2;
  }
  if(FLASH_PTR->PECR & FLASH_PECR_PRGLOCK_MASK) {
    FLASH_PTR->PRGKEYR = FLASH_PRGKEY1;
    FLASH_PTR->PRGKEYR = FLASH_PRGKEY2;
  }

  pAddr32 = (__IO uint32_t *) addr;
  p = pBuf;

#if FLASH_PROGRAM_SIGNLE_WORD_EN       /* signle word program */
  FLASH_PTR->PECR = FLASH_PECR_PROG_YES;

  cnt = size >> 3;
  for(int i = 0; i < cnt; i++) {
    while(FLASH_PTR->SR & FLASH_SR_BSY_MASK);
    FLASH_PTR->SR = FLASH_PTR->SR;
    val = DevFlashAsm8toL32(p);
    p += 4;
    *pAddr32++ = val;
    val = DevFlashAsm8toL32(p);
    p += 4;
    *pAddr32++ = val;
  }
#endif
#if FLASH_PROGRAM_HALF_WORD_EN         /* half-word program */
  FLASH_PTR->PECR = FLASH_PECR_PROG_YES | FLASH_PECR_FPRG_YES;

  cnt = size >> 2;
  for(int i = 0; i < cnt; ) {
    while(FLASH_PTR->SR & FLASH_SR_BSY_MASK);
    FLASH_PTR->SR = FLASH_PTR->SR;
    /* write the first word */
    val = DevFlashAsm8toL32(p);
    p += 4;
    *pAddr32++ = val;
    i++;
    for(int j = 0; j < 0x40/4-1; j++) {
      val = DevFlashAsm8toL32(p);
      p += 4;
      *pAddr32 = val;
    }
    pAddr32 += 15;
    i += 15;
  }
#endif

  FLASH_PTR->PECR = 0;

  return 0;
}


int
DevFlashWrite32(int unit, uint32_t addr, uint32_t word)
{
  uint32_t              val;
  uint8_t               *p;
  int                   cnt;
  __IO uint32_t         *pAddr32;

  while(FLASH_PTR->SR & FLASH_SR_BSY_MASK);
  FLASH_PTR->SR = FLASH_PTR->SR;
  if(FLASH_PTR->PECR & FLASH_PECR_PELOCK_MASK) {
    FLASH_PTR->PKEYR = FLASH_PKEY1;
    FLASH_PTR->PKEYR = FLASH_PKEY2;
  }
  if(FLASH_PTR->PECR & FLASH_PECR_PRGLOCK_MASK) {
    FLASH_PTR->PRGKEYR = FLASH_PRGKEY1;
    FLASH_PTR->PRGKEYR = FLASH_PRGKEY2;
  }
  FLASH_PTR->PECR = FLASH_PECR_PROG_YES | FLASH_PECR_FPRG_YES;

  while(FLASH_PTR->SR & FLASH_SR_BSY_MASK);
  FLASH_PTR->SR = FLASH_PTR->SR;

  /* program */
  pAddr32 = (__IO uint32_t *) addr;
  *pAddr32++ = word;

  FLASH_PTR->PECR = 0;

}


int
DevFlashEraseEeprom(int unit, uint32_t addr, int size)
{
  __IO uint32_t         *pAddr32;
  int                   remain;

  /* check busy and set the unlock keys */
  while(FLASH_PTR->SR & FLASH_SR_BSY_MASK);
  FLASH_PTR->SR = FLASH_PTR->SR;
  if(FLASH_PTR->PECR & FLASH_PECR_PELOCK_MASK) {
    FLASH_PTR->PKEYR = FLASH_PKEY1;
    FLASH_PTR->PKEYR = FLASH_PKEY2;
  }
#if 0
  if(FLASH_PTR->PECR & FLASH_PECR_PRGLOCK_MASK) {
    FLASH_PTR->PRGKEYR = FLASH_PRGKEY1;
    FLASH_PTR->PRGKEYR = FLASH_PRGKEY2;
  }
#endif

  FLASH_PTR->PECR |= FLASH_PECR_DATA_YES;
  FLASH_PTR->PECR |= FLASH_PECR_ERASE_YES;

  /* erase */
#if 0
  remain = (size + FLASH_PAGE_SIZE -1) & (FLASH_PAGE_SIZE-1);
  while(remain > 0) {
    pAddr32 = (__IO uint32_t *) addr;
    *pAddr32 = 0;
    addr += FLASH_PAGE_SIZE;
    remain -= FLASH_PAGE_SIZE;
    while(FLASH_PTR->SR & FLASH_SR_BSY_MASK);
  }
#endif

  printf("xxxxx erase %x %x\n", FLASH_PTR->PECR, FLASH_PTR->SR);

  pAddr32 = (__IO uint32_t *) addr;
  *pAddr32 = 0;
  while(FLASH_PTR->SR & FLASH_SR_BSY_MASK);

  FLASH_PTR->PECR = 0;

  return 0;
}




static uint32_t
DevFlashAsm8toL32(uint8_t *p)
{
  uint32_t	val;
  val  = (*p++ <<  0);
  val |= (*p++ <<  8);
  val |= (*p++ << 16);
  val |= (*p   << 24);
  return val;
}
static uint32_t
DevFlashAsm8toB32(uint8_t *p)
{
  uint32_t	val;
  val  = (*p++ << 24);
  val |= (*p++ << 16);
  val |= (*p++ <<  8);
  val |= (*p   <<  0);
  return val;
}
static void
DevFlashAsm32toB8(uint8_t *p, uint32_t val)
{
  p[0] = val >> 24;
  p[1] = val >> 16;
  p[2] = val >>  8;
  p[3] = val >>  0;
  return;
}
#if 0
static void
DevFlashAsm32toL8(uint8_t *p, uint32_t val)
{
  p[0] = val >>  0;
  p[1] = val >>  8;
  p[2] = val >> 16;
  p[3] = val >> 24;
  return;
}
#endif

