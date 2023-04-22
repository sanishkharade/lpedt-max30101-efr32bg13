/*******************************************************
 *
 *  @file   : scheduler.h
 *
 *  @brief  : This header file provides abstraction of the
 *            scheduler functions
 *
 *  @date   : September 09, 2022
 *  @author : Sanish Kharade
 *
 ******************************************************/

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_


/*
 * There will be only one scheduler for multiple state machines
 * The events will be generic and not specific to a state machine
 * Multiple state machines can have state transitions on the same event
 * */
typedef enum {
  EVENT_MAX_PRIORITY,
  LETIMER0_UF,
  LETIMER0_COMP1,
  I2C0_TRANSFER_COMPLETE,
  EVENT_MIN_PRIORITY
} scheduler_event_t;

/**
 * @brief   : Function to set the underflow event of LETIMER0
 *
 * @param   : none
 *
 * @return  : void
 */
void schedulerSetEvent_LETIMER0_UF(void);

/**
 * @brief   : Function to set the COMP1 event of LETIMER0
 *
 * @param   : none
 *
 * @return  : void
 */
void schedulerSetEvent_LETIMER0_COMP1(void);

/**
 * @brief   : Function to set the I2C0 event when transfer is complete
 *
 * @param   : none
 *
 * @return  : void
 */
void schedulerSetEvent_I2C0_TRANSFER_COMPLETE(void);

/**
 * @brief   : Function to get the highest priority event that has occurred since
 *            the last time this function was called
 *
 * @param   : none
 *
 * @return  : void
 */
scheduler_event_t schedulerGetEvent(void);

/**
 * @brief   : State Machine for the Si7021 temperature sensor
 *
 * @param   : none
 *
 * @return  : void
 */
void temperature_state_machine(scheduler_event_t event);

void handle_gesture();

#endif /* SRC_SCHEDULER_H_ */
