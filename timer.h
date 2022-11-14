/*
 * timer.c
 *
 *  Created on: Feb 13, 2017
 *      Author: srazak
 *  Modified on: Sept 2, 2019
 *      Author: efeoflus
 */

#ifndef TIMER_H_
#define TIMER_H_

// Initialize Systick
void SystickInit();

// busy-waiting for 10 milliseconds
void SysTick_Wait10ms(uint32_t delay);

// busy-waiting for 1 millisecond
void SysTick_Wait1ms(uint32_t delay);

// busy-waiting for 100 microseconds
void SysTick_Wait100microsec(uint32_t delay);

#endif /* TIMER_H_ */
