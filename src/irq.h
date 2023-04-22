/*******************************************************
 *
 *  @file   : irq.h
 *
 *  @brief  : This is the header file for irq.c
 *
 *  @date   : September 04, 2022
 *  @author : Sanish Kharade
 *
 ******************************************************/

#ifndef SRC_IRQ_H_
#define SRC_IRQ_H_

/**
 * @brief   : Function to get milliseconds passed since startup
 *
 * @param   : none
 *
 * @return  : uint32_t - milliseconds passed since startup
 */
uint32_t letimerMilliseconds();

uint32_t get_read_ges();

void clear_read_ges();

bool is_pb0_pressed();
void clear_pb0_press();

bool is_pb1_pressed();
void clear_pb1_press();

#endif /* SRC_IRQ_H_ */
