/******************************************************************
 *
 *  @file   : timers.c
 *  @brief  : This source file contains the LETIMER0 functions
 *
 *  @date   : September 04, 2022
 *  @author : Sanish Kharade
 *
 *  @note   : Please see timers.h for function explanations
 *
 *****************************************************************/

#include "em_letimer.h"
#include "timers.h"
#include "gpio.h"
#include "app.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#define ACTUAL_CLK_FREQ     ( CLK_FREQ / PRESCALAR )

#define LETIMER_LOAD_VALUE  (LETIMER_PERIOD_MS * ACTUAL_CLK_FREQ) / 1000
#define LETIMER_COMP1_VALUE (LETIMER_LOAD_VALUE * LETIMER_ON_TIME_MS) / LETIMER_PERIOD_MS

#define LETIMER0_TOTAL_LOAD_COUNTS  (65536)
#define MAX_TIMER_WAIT_US   (LETIMER_PERIOD_MS * 1000)
#define MIN_TIMER_WAIT_US   (1000000/ACTUAL_CLK_FREQ)


void letimer0_init(void)
{
  // Initialize the letimer init parameter
  LETIMER_Init_TypeDef init_letimer0 = {
      false,
      false,
      true,
      false,
      0,
      0,
      letimerUFOANone,
      letimerUFOANone,
      letimerRepeatFree,
      0
  };

  // Initialize the LETIMER0
  LETIMER_Init(LETIMER0, &init_letimer0);

  // Set the required interrupt bits
  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF);

  // Set the compare value in the COM0 register
  LETIMER_CompareSet(LETIMER0, 0, LETIMER_LOAD_VALUE);


  LETIMER0->CTRL |= LETIMER_CTRL_COMP0TOP;

  // Clear pending IRQs and enable the required ones
  NVIC_ClearPendingIRQ(LETIMER0_IRQn);
  NVIC_EnableIRQ(LETIMER0_IRQn);

  // Start the timer
  LETIMER_Enable(LETIMER0, true);

}
/**
 * @brief   : Function to convert microseconds to ticks depending on
 *            the clock used
 *
 * @param   : none
 *
 * @return  : void
 */
static uint32_t usToTicks(uint64_t time_us)
{
  uint32_t ticks = 0;

  ticks = (time_us * ACTUAL_CLK_FREQ) / 1000000;

  return (uint32_t)ticks;

}
void timerWaitUs_polled(uint32_t us_wait)
{
  // Range check if provided wait is possible
  if(us_wait < MIN_TIMER_WAIT_US || us_wait > MAX_TIMER_WAIT_US)
  {
      LOG_ERROR("Inaccurate wait period for LETIMER0");
      return;
  }

  uint32_t start_tick = LETIMER_CounterGet(LETIMER0);

  // Calculate end_count
  uint32_t tick_count = usToTicks(us_wait);

  uint32_t end_tick = 0;

  if(tick_count > start_tick) {

      end_tick = LETIMER_LOAD_VALUE - (tick_count - start_tick) + 1;
  }
  else {

      end_tick = start_tick - tick_count;
  }

  while (LETIMER_CounterGet(LETIMER0) != end_tick);

}
void timerWaitUs_irq(uint32_t us_wait)
{
  // Range check if provided wait is possible
  if(us_wait < MIN_TIMER_WAIT_US || us_wait > MAX_TIMER_WAIT_US)
  {
      LOG_ERROR("Inaccurate wait period for LETIMER0");
      return;
  }

  uint32_t start_tick = LETIMER_CounterGet(LETIMER0);

  // Calculate end_count
  uint32_t tick_count = usToTicks(us_wait);

  uint32_t end_tick = 0;

  if(tick_count > start_tick) {

      end_tick = LETIMER_LOAD_VALUE - (tick_count - start_tick) + 1;
  }
  else {

      end_tick = start_tick - tick_count;
  }

  LETIMER_CompareSet(LETIMER0, 1, end_tick);

  LETIMER_IntClear(LETIMER0, LETIMER_IEN_COMP1);
  // First time we do it an interrupt is triggered immediately. Hence we need
  // to do the same thing again
  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);
  LETIMER0->IEN |= LETIMER_IF_COMP1;

}
