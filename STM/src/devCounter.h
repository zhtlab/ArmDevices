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

#ifndef _DEV_COUNTER_H_
#define _DEV_COUNTER_H_

#define DEVCOUNTER_SYSTIMER_1U000S      (1)
#define DEVCOUNTER_SYSTIMER_10U00S      (10)
#define DEVCOUNTER_SYSTIMER_100U0S      (100)
#define DEVCOUNTER_SYSTIMER_1M000S      (1000)
#define DEVCOUNTER_SYSTIMER_10M00S      (1000*10)
#define DEVCOUNTER_SYSTIMER_100M0S      (1000*100)
#define DEVCOUNTER_SYSTIMER_1S000       (1000*1000)
#define DEVCOUNTER_SYSTIMER_10S00       (1000*1000*10)
#define DEVCOUNTER_SYSTIMER_100S0       (1000*1000*100)
#define DEVCOUNTER_SYSTIMER_1000S       (1000*1000*1000)


typedef struct {
  /* bit0 = ch0, bit1 = ch1, .. bit14=ch14, bit15=clkrtg */
  /* STM32 do not have ch0 */
  uint16_t              chnum;
#define DEVCOUNTER_SETCH(num)     (1<<(num))
#define DEVCOUNTER_CH_0           0
#define DEVCOUNTER_CH_0N          1
#define DEVCOUNTER_CH_1           2
#define DEVCOUNTER_CH_1N          3
#define DEVCOUNTER_CH_2           4
#define DEVCOUNTER_CH_2N          5
#define DEVCOUNTER_CH_3           6
#define DEVCOUNTER_CH_3N          7
#define DEVCOUNTER_CH_4           8
#define DEVCOUNTER_CH_4N          9
#define DEVCOUNTER_CH_MAX         9
#define DEVCOUNTER_CH_CLKTRG      15
  struct {
    uint16_t            mode;           /* [15:12] mode
                                         *            0: Freerun
                                         *            1: Reset
                                         *            2: Gate
                                         *            3: Start
                                         *            4: Restart
                                         *            5: Encoder
                                         * [11: 8] input category
                                         *            0: Internal
                                         *            1: TRGI (TS[4:0])
                                         *            2: ETF  (ETF[3:0])
                                         *   [7:0] input sel  TS[4:0], ETF[3:0]
                                         */
#define DEVTIME_CLKTRG_MODE_SHIFT       (12)
#define DEVTIME_CLKTRG_MODE_MASK        (0xf << (DEVTIME_CLKTRG_MODE_SHIFT))
#define DEVTIME_CLKTRG_MODE_FREERUN     (0 << (DEVTIME_CLKTRG_MODE_SHIFT))
#define DEVTIME_CLKTRG_MODE_RESET       (1 << (DEVTIME_CLKTRG_MODE_SHIFT))
#define DEVTIME_CLKTRG_MODE_GATE        (2 << (DEVTIME_CLKTRG_MODE_SHIFT))
#define DEVTIME_CLKTRG_MODE_START       (3 << (DEVTIME_CLKTRG_MODE_SHIFT))
#define DEVTIME_CLKTRG_MODE_RESTART     (4 << (DEVTIME_CLKTRG_MODE_SHIFT))
#define DEVTIME_CLKTRG_MODE_ENCODER     (5 << (DEVTIME_CLKTRG_MODE_SHIFT))
#define DEVTIME_CLKTRG_CTG_SHIFT        (8)
#define DEVTIME_CLKTRG_CTG_MASK         (0xf << (DEVTIME_CLKTRG_CTG_SHIFT))
#define DEVTIME_CLKTRG_CTG_INTERNAL     (0 << (DEVTIME_CLKTRG_CTG_SHIFT))
#define DEVTIME_CLKTRG_CTG_EXTERNAL1    (1 << (DEVTIME_CLKTRG_CTG_SHIFT))
#define DEVTIME_CLKTRG_CTG_EXTERNAL2    (2 << (DEVTIME_CLKTRG_CTG_SHIFT))
#define DEVTIME_CLKTRG_SEL_SHIFT        (0)
#define DEVTIME_CLKTRG_SEL_MASK         (0xff << (DEVTIME_CLKTRG_SEL_SHIFT))
#define DEVTIME_CLKTRG_SEL(x)           ((x) << (DEVTIME_CLKTRG_SEL_SHIFT))

    uint16_t            intr : 1;       /* 1 interrupt enable */
    uint16_t            dma : 1;        /* 1 dma request enable */
    uint16_t            polneg : 1;      /* 0: rising, 1: falling */
    uint16_t            down : 1;       /* 0 up counter, 1 down counter */
    uint16_t            oneshot : 1;    /* 0 normal operation, 1 one shot */
    uint16_t            reserved : 3;
    uint16_t            filter : 8;     /* filter param, depend on modules */
    uint16_t            prescaler;      /* clock prescaler, depend on modules */
    uint32_t            reload;         /* reload value */
  } clktrg;
  struct {
    uint16_t              mode : 8;
#define DEVCOUNTER_MODE_DISABLE   0
#define DEVCOUNTER_MODE_COUNTER   1
#define DEVCOUNTER_MODE_IC        2
#define DEVCOUNTER_MODE_PWM       3
    uint16_t              intr : 1;     /* 1: interrupt enable */
    uint16_t              dma : 1;      /* 1: dma request enable */
    uint16_t              polneg : 1;   /* 0: rising, 1: falling */
    uint16_t              reserved : 5;
    uint16_t              filter : 8;   /* filter param, depend on modules */
    uint32_t              modeOpt;      /* set option, if !0.  depend on modules */
    uint32_t              val;          /* compare value */
  } ch;
} devCounterParam_t;


typedef struct {
  uint8_t               up;
  uint8_t               unit;

  stm32Dev_TIM          *dev;

} devCounterSc_t;
struct _stDevCounter {
  devCounterSc_t          sc[17];
};

int             DevCounterInit(int unit, devCounterParam_t *param);

int             DevCounterSetPwmDutyValue(int unit, int ch, uint32_t val);
int             DevCounterGetIcValue(int unit, int ch, uint32_t *pVal);

void            DevCounterDebugShowRegs(int unit);

uint32_t        DevCounterGetSystemTimer(void);
void            DevCounterWaitSystemTimer(uint32_t tout);

#endif
