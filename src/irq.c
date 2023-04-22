/******************************************************************
 *
 *  @file   : irq.c
 *  @brief  : This file contains the interrupt service routine
 *
 *  @date   : September 04, 2022
 *  @author : Sanish Kharade
 *
 *
 *****************************************************************/

#include "em_letimer.h"
#include "gpio.h"
#include "scheduler.h"

#include "em_i2c.h"
#include "em_gpio.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


#define GPIO_INTPB0_MASK (1UL << PB0_PIN)
#define GPIO_INTPB1_MASK (1UL << PB1_PIN)

uint32_t milliseconds = 0;
uint32_t read_ges = 0;


bool pb0_pressed = false;
bool pb1_pressed = false;
/**
 * @brief   : Interrupt Handler for LETIMER0
 *
 * @param   : none
 *
 * @return  : void
 */
void LETIMER0_IRQHandler(void)
{
  // Determine the source of the IRQ
  uint32_t flags = LETIMER_IntGetEnabled(LETIMER0);

  // Clear the interrupts
  LETIMER_IntClear(LETIMER0, flags);

  if (flags & LETIMER_IEN_UF) {

      milliseconds += 1;
      schedulerSetEvent_LETIMER0_UF();

  }

  if (flags & LETIMER_IEN_COMP1) {

      LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
      schedulerSetEvent_LETIMER0_COMP1();

  }

}
/**
 * @brief   : Interrupt Handler for I2C0
 *
 * @param   : none
 *
 * @return  : void
 */
void I2C0_IRQHandler(void)
{

  I2C_TransferReturn_TypeDef transferStatus;

  transferStatus = I2C_Transfer(I2C0);
  if (transferStatus == i2cTransferDone) {
      //schedulerSetEvent_I2C0_TRANSFER_COMPLETE();
  }

  if (transferStatus < 0) {
      LOG_ERROR("I2C Transfer: I2C failed with error = %d\n\r", transferStatus);
  }
}

/**
 * @brief   : Interrupt Handler for EVEN GPIO
 *
 * @param   : none
 *
 * @return  : void
 */
void GPIO_EVEN_IRQHandler(void)
{

  // Determine the source of the IRQ
  uint32_t flags = GPIO_IntGetEnabled();

  //pb0_state = GPIO_PinInGet(PB0_PORT, PB0_PIN);

  GPIO_IntClear(flags);
  if (flags & GPIO_INTPB0_MASK) {

      pb0_pressed = true;
      //schedulerSetEvent_GPIO_PB0();

  }

}

/**
 * @brief   : Interrupt Handler for ODD GPIO
 *
 * @param   : none
 *
 * @return  : void
 */
void GPIO_ODD_IRQHandler(void)
{

  // Determine the source of the IRQ
  uint32_t flags = GPIO_IntGetEnabled();

  GPIO_IntClear(flags);
  if (flags == 2048) {

      read_ges = 1;

  }
  if (flags & GPIO_INTPB1_MASK) {

      pb1_pressed = true;
      //schedulerSetEvent_GPIO_PB0();

  }

}

uint32_t get_read_ges()
{
  return read_ges;
}
void clear_read_ges()
{
  read_ges = 0;
}


bool is_pb0_pressed()
{
  return pb0_pressed;
}
void clear_pb0_press()
{
  pb0_pressed = false;
}

bool is_pb1_pressed()
{
  return pb1_pressed;
}
void clear_pb1_press()
{
  pb1_pressed = false;
}

uint32_t letimerMilliseconds()
{
  return milliseconds;
}
