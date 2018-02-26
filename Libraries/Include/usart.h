#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "arm_math.h"

#ifndef _usart_H_
#define _usart_H_

void init_USART6();
void USART_send_word8(uint8_t);
void USART_sendString(uint8_t*);
void USART_sendMany_word16(uint16_t*, uint8_t);
void USART_sendFloat_word8(float32_t*);
void USART_sendManyFloat_word8(float32_t*, uint8_t);

#endif
