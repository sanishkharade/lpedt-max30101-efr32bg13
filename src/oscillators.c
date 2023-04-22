/******************************************************************
 *
 *  @file   : oscillators.c
 *  @brief  : This source file contains the oscillator functions
 *
 *  @date   : September 04, 2022
 *  @author : Sanish Kharade
 *
 *  @note   : Please see oscillators.h for function explanations
 *
 *****************************************************************/

#include <em_cmu.h>

#include "oscillators.h"
#include "app.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

void oscillator_init(void)
{

#if (LOWEST_ENERGY_MODE == 3)
  CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
#else
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
#endif

  CMU_ClockDivSet(cmuClock_LETIMER0, PRESCALAR);
  CMU_ClockEnable(cmuClock_LETIMER0, true);

#if DEBUG
  LOG_INFO("clk cmuClock_LFA = %d\n\r", CMU_ClockSelectGet(cmuClock_LFA));
  LOG_INFO("clkfreq cmuClock_LETIMER0 = %ld\n\r", CMU_ClockFreqGet(cmuClock_LETIMER0));
  LOG_INFO("clkfreq cmuClock_LFA = %ld\n\r", CMU_ClockFreqGet(cmuClock_LFA));
#endif

}
