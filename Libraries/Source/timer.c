#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "timer.h"

/* TimeBase management functions: stm32f4xx_tim.c l.160*/

void timerInit(){
	TIM_TimeBaseInitTypeDef TIM_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable TIM clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	//between 0x0000 and 0xFFFF
	TIM_InitStruct.TIM_Period = 499;
	//between 0x0000 and 0xFFFF. fCK_PSC / (PSC[15:0] + 1)
	TIM_InitStruct.TIM_Prescaler = 3359;//SystemCoreClock / (SystemCoreClock / 10000) - 1;
	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_InitStruct);
	/* Enable update interrupt */
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	/* TIM3 enable counter */
	TIM_Cmd(TIM1, ENABLE);

	/* Enable the TIM3_ global Interrupt */
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void timerStart(){}

void timerStop(){}


