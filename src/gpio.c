/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.

   Modified by: Sanish Kharade, sanish.kharade@colorado.edu
   Change     : Added LED toggle functions
   Date       : 09-04-2022

 */


// *****************************************************************************
// Students:
// We will be creating additional functions that configure and manipulate GPIOs.
// For any new GPIO function you create, place that function in this file.
// *****************************************************************************

#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>

#include "gpio.h"


// Student Edit: Define these, 0's are placeholder values.
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.

#define LED0_port  5 // change to correct ports and pins
#define LED0_pin   4
#define LED1_port  5
#define LED1_pin   5

#define APDS9960_INT_PORT   gpioPortD
#define APDS9960_INT_PIN    11


// Set GPIO drive strengths and modes of operation
void gpioInit()
{

	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);

	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

	// Initialize the SENSOR_ENABLE PIN for temperature sensor Si7021
  GPIO_DriveStrengthSet(SI7021_SENSOR_ENABLE_PORT, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(SI7021_SENSOR_ENABLE_PORT, SI7021_SENSOR_ENABLE_PIN, gpioModePushPull, false);

  GPIO_PinModeSet(APDS9960_INT_PORT, APDS9960_INT_PIN, gpioModeInputPullFilter, true);
  GPIO_ExtIntConfig(APDS9960_INT_PORT, APDS9960_INT_PIN, APDS9960_INT_PIN, false, true, true);


  /* Initialize the PB0 pin as input
   * Setting DOUT as true means that the filter is enabled
   * */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, true);
  //GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInputPullFilter, true);

  /* Enable both rising and falling edge interrupts */
  GPIO_ExtIntConfig(PB0_PORT, PB0_PIN, PB0_PIN, false, true, true);


  /* Initialize the PB1 pin as input
   * Setting DOUT as true means that the filter is enabled
   * */
  GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInput, true);
  //GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInputPullFilter, true);

  /* Enable both rising and falling edge interrupts */ //A9
  GPIO_ExtIntConfig(PB1_PORT, PB1_PIN, PB1_PIN, false, true, true);


  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
} // gpioInit()


void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}


void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}

void gpioLed0Toggle()
{
  GPIO_PinOutToggle(LED0_port, LED0_pin);
}


void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}


void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}

void gpioLed1Toggle()
{
  GPIO_PinOutToggle(LED1_port, LED1_pin);
}






