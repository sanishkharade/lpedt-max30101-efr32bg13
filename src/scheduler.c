/******************************************************************
 *
 *  @file   : scheduler.c
 *  @brief  : This source file contains the scheduler functions
 *
 *  @date   : September 09, 2022
 *  @author : Sanish Kharade
 *
 *  @note   : Please see scheduler.h for function explanations
 *
 *  @note   : Explanation of Critical Sections -->
 *            Core macros have been used in all the schedulerSetEventXXXX()
 *            functions to protect the critical sections
 *
 *            CORE_DECLARE_IRQ_STATE declares a variable irqState of type
 *            CORE_irqState_t. irqState stores the current interrupt status of
 *            the system which is a value returned by the macro
 *            CORE_ENTER_CRITICAL(). This irqState value is then passed to the
 *            CORE_EXIT_CRITICAL() macro for exiting the critical section.
 *
 *****************************************************************/
#include <em_core.h>
#include "scheduler.h"
#include "i2c.h"
#include "timers.h"
#include "sl_power_manager_debug.h"


#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


#include "SparkFun_APDS9960.h"

#define LETIMER0_UF_EVENT_SHIFT                 (1)
#define LETIMER0_UF_EVENT_MASK                  (0x1UL << LETIMER0_UF_EVENT_SHIFT)


#define LETIMER0_COMP1_EVENT_SHIFT              (2)
#define LETIMER0_COMP1_EVENT_MASK               (0x1UL << LETIMER0_COMP1_EVENT_SHIFT)

#define I2C0_TRANSFER_COMPLETE_EVENT_SHIFT      (3)
#define I2C0_TRANSFER_COMPLETE_EVENT_MASK       (0x1UL << I2C0_TRANSFER_COMPLETE_EVENT_SHIFT)


typedef enum {

  stateIDLE,
  statePOWERUP,
  stateSENDCOMMAND,
  stateCONVERSION,
  stateREADTEMP,
  numStates
} sm_temp_state_t;


uint32_t events = 0;
void schedulerSetEvent_LETIMER0_UF(void)
{

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  // Critical Section
  events |= LETIMER0_UF_EVENT_MASK;

  CORE_EXIT_CRITICAL();
}

void schedulerSetEvent_LETIMER0_COMP1(void)
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  // Critical Section
  events |= LETIMER0_COMP1_EVENT_MASK;

  CORE_EXIT_CRITICAL();
}

void schedulerSetEvent_I2C0_TRANSFER_COMPLETE(void)
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  // Critical Section
  events |= I2C0_TRANSFER_COMPLETE_EVENT_MASK;

  CORE_EXIT_CRITICAL();
}

scheduler_event_t schedulerGetEvent(void)
{

  scheduler_event_t event = EVENT_MAX_PRIORITY;
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  // Critical Section

  for (event = EVENT_MAX_PRIORITY; event < EVENT_MIN_PRIORITY; event++)
  {
      if (events & (0x1UL << event))
      {
          events &= ~(0x1UL << event);
          break;
      }
  }

  CORE_EXIT_CRITICAL();
  return event;
}

void temperature_state_machine(scheduler_event_t event)
{
  sm_temp_state_t currentState;

  static sm_temp_state_t nextState = stateIDLE;

  currentState = nextState;
  switch(currentState)
  {
    case stateIDLE:
      nextState = stateIDLE;  // default
      if (event == LETIMER0_UF) {

          // Turn Power ON
          Si7021_power(true);

          // Configure timer
          timerWaitUs_irq(SI7021_POWERUP_TIME_US);

          // Update next state
          nextState = statePOWERUP;
      }
      break;

    case statePOWERUP:
      nextState = statePOWERUP;  // default
      if (event == LETIMER0_COMP1) {

          // Add power requirement here, just before starting I2C
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);

          // Send I2C read command
          sendReadCommandToSi7021();

          // Update next state
          nextState = stateSENDCOMMAND;
      }
      break;

    case stateSENDCOMMAND:
      nextState = stateSENDCOMMAND;  // default
      if (event == I2C0_TRANSFER_COMPLETE) {

          //Disable NVIC for I2C0
          NVIC_DisableIRQ(I2C0_IRQn);

          // Remove power requirement here, just after ending I2C transfer
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);

          // Configure timer for data to be ready
          timerWaitUs_irq(SI7021_CONVERSION_TIME_US);

          // Update next state
          nextState = stateCONVERSION;
      }
      break;

    case stateCONVERSION:
      nextState = stateCONVERSION;  // default
      if (event == LETIMER0_COMP1) {

          // Add power requirement here, just before starting I2C
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);

          // Read temperature from the temperature sensor
          readTemperature();

          // Update next state
          nextState = stateREADTEMP;
      }
      break;

    case stateREADTEMP:
      nextState = stateREADTEMP;  // default
      if (event == I2C0_TRANSFER_COMPLETE) {

          // Disable NVIC for I2C0
          NVIC_DisableIRQ(I2C0_IRQn);

          // Print the temperature
          printTemperature();
          // Remove power requirement here, just after ending I2C transfer
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);

          // Turn Power OFF
          Si7021_power(false);

          // Update next state
          nextState = stateIDLE;
      }
      break;

    default:
      break;

  }
}

void handle_gesture() {

  if ( isGestureAvailable() ) {

      int c = readGesture();
      switch (c) {

        case DIR_UP:
          LOG_INFO("UP\n\r");
          break;

        case DIR_DOWN:
          LOG_INFO("DOWN\n\r");
          break;

        case DIR_LEFT:
          LOG_INFO("LEFT\n\r");
          break;

        case DIR_RIGHT:
          LOG_INFO("RIGHT\n\r");
          break;

        case DIR_NEAR:
          LOG_INFO("NEAR\n\r");
          break;

        case DIR_FAR:
          LOG_INFO("FAR\n\r");
          break;

        default:
          LOG_INFO("NONE\n\r");
          break;
      }
  }
}
