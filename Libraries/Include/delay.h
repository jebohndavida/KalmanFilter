#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f4xx.h"
#include "misc.h"

#define SYSTICK_FREQ 1000000

extern uint32_t tick_count;
extern uint32_t usec_count;
extern uint32_t msec_count;
extern uint32_t sec_count;

#define gettc()		tick_count
#define rtime()		sec_count

void SysTick_Init(uint32_t microSeconds);
void TimeTick_Decrement(void);
void delay_nus(uint32_t n);
void delay_1ms(void);
void delay_nms(uint32_t n);
uint32_t delay_start_measure(void);
uint32_t delay_measure(uint32_t start);

#endif
