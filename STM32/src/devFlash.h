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

#ifndef _DEV_FLASH_H_
#define _DEV_FLASH_H_


int             DevFlashInit(int unit);
int             DevFlashErase(int unit, uint32_t addr, int size);
int             DevFlashProgram(int unit, uint32_t addr, int size, uint8_t *pBuf);
int             DevFlashWrite32(int unit, uint32_t addr, uint32_t word);

int             DevFlashEraseEeprom(int unit, uint32_t addr, int size);




#ifdef  _DEV_FLASH_C_

/* prototypes */
static uint32_t DevFlashAsm8toL32(uint8_t *p);
static uint32_t DevFlashAsm8toB32(uint8_t *p);
static void     DevFlashAsm32toB8(uint8_t *p, uint32_t val);
static void     DevFlashAsm32toL8(uint8_t *p, uint32_t val);

#endif

#endif
