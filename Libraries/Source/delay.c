#include "delay.h"

static __IO uint32_t sysTickCounter;
static __IO uint32_t microSecs;

uint32_t tick_count;
uint32_t usec_count;
uint32_t msec_count;
uint32_t sec_count;

/****************************************
 *SystemFrequency/1000      1ms         *
 *SystemFrequency/100000    10us        *
 *SystemFrequency/1000000   1us         *
 ****************************************/
void SysTick_Init(uint32_t microSeconds) {
	NVIC_InitTypeDef NVIC_InitStructure;
//	uint32_t divider = SYSTICK_FREQ / microSeconds;

	microSecs = microSeconds;
	while (SysTick_Config(SystemCoreClock / (1000 / microSeconds)) != 0) {
	}

	// One SysTick interrupt now equals microSeconds

	/* set SysTick interrupt priority to high */
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), !0, 0));
}

/**
 * This method needs to be called in the SysTick_Handler
 */
void TimeTick_Decrement(void) {
	if (sysTickCounter != 0x00) {
		sysTickCounter--;
	}
}

void delay_nus(uint32_t n) {
	sysTickCounter = n / microSecs;
	while (sysTickCounter != 0) {
	}
}

void delay_1ms(void) {
	sysTickCounter = 1000 / microSecs;
	while (sysTickCounter != 0) {
	}
}

void delay_nms(uint32_t n) {
	while (n--) {
		delay_1ms();
	}
}

uint32_t delay_start_measure(void) {
	return tick_count;
}

uint32_t delay_measure(uint32_t start) {
	int32_t difference = tick_count - start;
	if (difference < 0) {
		difference = difference + INT32_MAX;
	}
	return difference * microSecs;
}

void SysTick_Handler(void) {
	TimeTick_Decrement();
	tick_count++;
	usec_count += microSecs;
	if (usec_count > 1000) {
		msec_count++;
		usec_count -= 1000;
		if (msec_count > 1000) {
			sec_count++;
			msec_count -= 1000;
		}
	}
}
