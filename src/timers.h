/*******************************************************
 *
 *  @file   : timers.h
 *
 *  @brief  : This header file provides abstraction of the
 *            LETIMER0 functions
 *
 *  @date   : September 04, 2022
 *  @author : Sanish Kharade
 *
 ******************************************************/
#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

/**
 * @brief   : Function to initialize LETIMER0
 *
 * @param   : none
 *
 * @return  : void
 */
void letimer0_init(void);

/**
 * @brief   : Function to generate a delay using polling
 *
 * @param   : us_wait - required delay in microseconds
 *
 * @return  : void
 */
void timerWaitUs_polled(uint32_t us_wait);


/**
 * @brief   : Function to generate a delay using interrupts
 *
 * @param   : us_wait - required delay in microseconds
 *
 * @return  : void
 */
void timerWaitUs_irq(uint32_t us_wait);


#endif /* SRC_TIMERS_H_ */
