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

#ifndef _STM32L4FLASH_H_
#define _STM32L4FLASH_H_

#define FLASH_ERASED_VALUE              0xff
#define FLASH_ERASED_VALUE32            0xffffffffUL

typedef struct {
  __IO uint32_t		ACR;
#define FLASH_LATENCY_SHIFT	(0)
#define FLASH_LATENCY_NONE (0 << (FLASH_LATENCY_SHIFT))
#define FLASH_LATENCY_1CLK	(1 << (FLASH_LATENCY_SHIFT))
#define FLASH_LATENCY_2CLK	(2 << (FLASH_LATENCY_SHIFT))
#define FLASH_LATENCY_3CLK	(3 << (FLASH_LATENCY_SHIFT))
#define FLASH_LATENCY_4CLK	(4 << (FLASH_LATENCY_SHIFT))

  __IO uint32_t		PDKEYR;
#define PDKEYR_KEY1	0x04152637
#define PDKEYR_KEY2	0xfafbfcfd
  __IO uint32_t		KEYR;
#define KEYR_KEY1	0x45670123
#define KEYR_KEY2	0xcdef89ab
  __IO uint32_t		OPTKEYR;
#define OPTKEYR_KEY1	0x08192a3b
#define OPTKEYR_KEY2	0x4c5d6e7f

  __IO uint32_t		SR;
#define SR_PEMPTY_SHIFT		17
#define SR_PEMPTY_MASK		(1 << (SR_PEMPTY_SHIFT))
#define SR_PEMPTY_NO		(0 << (SR_PEMPTY_SHIFT))
#define SR_PEMPTY_YES		(1 << (SR_PEMPTY_SHIFT))
#define SR_BSY_SHIFT		16
#define SR_BSY_MASK		(1 << (SR_BSY_SHIFT))
#define SR_BSY_NO		(0 << (SR_BSY_SHIFT))
#define SR_BSY_YES		(1 << (SR_BSY_SHIFT))
#define SR_OPTVERR_SHIFT	15	
#define SR_OPTVERR_MASK		(1 << (SR_OPTVERR_SHIFT))
#define SR_OPTVERR_NO		(0 << (SR_OPTVERR_SHIFT))
#define SR_OPTVERR_YES		(1 << (SR_OPTVERR_SHIFT))
#define SR_RDERR_SHIFT		14
#define SR_RDERR_MASK		(1 << (SR_RDERR_SHIFT))
#define SR_RDERR_NO		(0 << (SR_RDERR_SHIFT))
#define SR_RDERR_YES		(1 << (SR_RDERR_SHIFT))
#define SR_FASTERR_SHIFT	9	
#define SR_FASTERR_MASK		(1 << (SR_FASTERR_SHIFT))
#define SR_FASTERR_NO		(0 << (SR_FASTERR_SHIFT))
#define SR_FASTERR_YES		(1 << (SR_FASTERR_SHIFT))
#define SR_MISERR_SHIFT		8
#define SR_MISERR_MASK		(1 << (SR_MISERR_SHIFT))
#define SR_MISERR_NO		(0 << (SR_MISERR_SHIFT))
#define SR_MISERR_YES		(1 << (SR_MISERR_SHIFT))
#define SR_PGSERR_SHIFT		7
#define SR_PGSERR_MASK		(1 << (SR_PGSERR_SHIFT))
#define SR_PGSERR_NO		(0 << (SR_PGSERR_SHIFT))
#define SR_PGSERR_YES		(1 << (SR_PGSERR_SHIFT))
#define SR_SIZERR_SHIFT		6
#define SR_SIZERR_MASK		(1 << (SR_SIZERR_SHIFT))
#define SR_SIZERR_NO		(0 << (SR_SIZERR_SHIFT))
#define SR_SIZERR_YES		(1 << (SR_SIZERR_SHIFT))
#define SR_PGAERR_SHIFT		5
#define SR_PGAERR_MASK		(1 << (SR_PGAERR_SHIFT))
#define SR_PGAERR_NO		(0 << (SR_PGAERR_SHIFT))
#define SR_PGAERR_YES		(1 << (SR_PGAERR_SHIFT))
#define SR_WRPERR_SHIFT		4
#define SR_WRPERR_MASK		(1 << (SR_WRPERR_SHIFT))
#define SR_WRPERR_NO		(0 << (SR_WRPERR_SHIFT))
#define SR_WRPERR_YES		(1 << (SR_WRPERR_SHIFT))
#define SR_PROGERR_SHIFT	3
#define SR_PROGERR_MASK		(1 << (SR_PROGERR_SHIFT))
#define SR_PROGERR_NO		(0 << (SR_PROGERR_SHIFT))
#define SR_PROGERR_YES		(1 << (SR_PROGERR_SHIFT))
#define SR_OPERR_SHIFT		1
#define SR_OPERR_MASK		(1 << (SR_OPERR_SHIFT))
#define SR_OPERR_NO		(0 << (SR_OPERR_SHIFT))
#define SR_OPERR_YES		(1 << (SR_OPERR_SHIFT))
#define SR_EOP_SHIFT		0
#define SR_EOP_MASK		(1 << (SR_EOP_SHIFT))
#define SR_EOP_NO		(0 << (SR_EOP_SHIFT))
#define SR_EOP_YES		(1 << (SR_EOP_SHIFT))

  __IO uint32_t		CR;
#define CR_LOCK_SHIFT		31
#define CR_LOCK_MASK		(1UL << (CR_LOCK_SHIFT))
#define CR_LOCK_NO		(0UL << (CR_LOCK_SHIFT))
#define CR_LOCK_YES		(1UL << (CR_LOCK_SHIFT))
#define CR_OPTLOCK_SHIFT	30
#define CR_OPTLOCK_MASK		(1 << (CR_OPTLOCK_SHIFT))
#define CR_OPTLOCK_NO		(0 << (CR_OPTLOCK_SHIFT))
#define CR_OPTLOCK_YES		(1 << (CR_OPTLOCK_SHIFT))
#define CR_OBL_LAUNCH_SHIFT	27
#define CR_OBL_LAUNCH_MASK	(1 << (CR_OBL_LAUNCH_SHIFT))
#define CR_OBL_LAUNCH_NO	(0 << (CR_OBL_LAUNCH_SHIFT))
#define CR_OBL_LAUNCH_YES	(1 << (CR_OBL_LAUNCH_SHIFT))
#define CR_RDERRIE_SHIFT	26
#define CR_RDERRIE_MASK		(1 << (CR_RDERRIE_SHIFT))
#define CR_RDERRIE_NO		(0 << (CR_RDERRIE_SHIFT))
#define CR_RDERRIE_YES		(1 << (CR_RDERRIE_SHIFT))
#define CR_ERRIE_SHIFT		25
#define CR_ERRIE_MASK		(1 << (CR_ERRIE_SHIFT))
#define CR_ERRIE_NO		(0 << (CR_ERRIE_SHIFT))
#define CR_ERRIE_YES		(1 << (CR_ERRIE_SHIFT))
#define CR_EOPIE_SHIFT		24
#define CR_EOPIE_MASK		(1 << (CR_EOPIE_SHIFT))
#define CR_EOPIE_NO		(0 << (CR_EOPIE_SHIFT))
#define CR_EOPIE_YES		(1 << (CR_EOPIE_SHIFT))
#define CR_FSTPG_SHIFT		18
#define CR_FSTPG_MASK		(1 << (CR_FSTPG_SHIFT))
#define CR_FSTPG_NO		(0 << (CR_FSTPG_SHIFT))
#define CR_FSTPG_YES		(1 << (CR_FSTPG_SHIFT))
#define CR_OPTSTRT_SHIFT	17
#define CR_OPTSTRT_MASK		(1 << (CR_OPTSTRT_SHIFT))
#define CR_OPTSTRT_NO		(0 << (CR_OPTSTRT_SHIFT))
#define CR_OPTSTRT_YES		(1 << (CR_OPTSTRT_SHIFT))
#define CR_STRT_SHIFT		16
#define CR_STRT_MASK		(1 << (CR_STRT_SHIFT))
#define CR_STRT_NO		(0 << (CR_STRT_SHIFT))
#define CR_STRT_YES		(1 << (CR_STRT_SHIFT))
#define CR_PNB_SHIFT		3
#define CR_PNB_MASK		(0xff << (CR_PNB_SHIFT))
#define CR_PNB_SECT(x)		((x) << (CR_PNB_SHIFT))
#define CR_MER1_SHIFT		2
#define CR_MER1_MASK		(1 << (CR_MER1_SHIFT))
#define CR_MER1_NO		(0 << (CR_MER1_SHIFT))
#define CR_MER1_YES		(1 << (CR_MER1_SHIFT))
#define CR_PER_SHIFT		1
#define CR_PER_MASK		(1 << (CR_PER_SHIFT))
#define CR_PER_NO		(0 << (CR_PER_SHIFT))
#define CR_PER_YES		(1 << (CR_PER_SHIFT))
#define CR_PG_SHIFT		0
#define CR_PG_MASK		(1 << (CR_PG_SHIFT))
#define CR_PG_NO		(0 << (CR_PG_SHIFT))
#define CR_PG_YES		(1 << (CR_PG_SHIFT))

  __IO uint32_t		ECCR;
  __IO uint32_t		OPTR;
  __IO uint32_t		PCROP1ST;
  __IO uint32_t		PCROP1ER;
  __IO uint32_t		WRP1AR;
  __IO uint32_t		WRP1BR;
} stm32Dev_FLASH;

#endif
