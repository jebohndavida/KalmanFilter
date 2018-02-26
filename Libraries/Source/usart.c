#include "usart.h"
#include "arm_math.h"

/*
U(S)ARTX     |TX     RX      |TX     RX      |TX     RX
USART1       |PA9    PA10    |PB6    PB7     |-      -
USART2       |PA2    PA3     |PD5    PD6     |-      -
USART3       |PB10   PB11    |PC10   PC11    |PD8    PD9
UART4        |PA0    PA1     |PC10   PC11    |-      -
UART5        |PC12   PD2     |-      -       |-      -
USART6       |PC6    PC7     |PG14   PG9     |-      -
UART7        |PE8    PE7     |PF7    PF6     |-      -
UART8        |PE1    PE0     |-      -       |-      -
*/

void init_USART6(){
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	uint32_t baudrate = 115200;
	// enable APB2 peripheral clock for USART6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	// enable clock for Rx and Tx pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOC, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() f
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); //
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	USART_InitStruct.USART_BaudRate = baudrate;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART6, &USART_InitStruct);
	USART_Cmd(USART6, ENABLE);
}

// Sends 1 byte
void USART_send_word8(uint8_t data){
	// wait until data register is empty
	while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
	// uint16_t cast puts data into LSB
	USART_SendData(USART6, (uint16_t)data);
}

// Sends string
void USART_sendString(uint8_t *data){
	uint8_t i=0;
	//for(uint8_t i=0; i<nBytes; i++){
	while((*(data+i)) != '*'){
		// wait until data register is empty
		while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
		USART_SendData(USART6, (uint16_t)(*(data+i)));
		i++;
	}
}

// Sends many bytes
void USART_sendMany_word16(uint16_t *data, uint8_t nBytes){
	for(uint8_t i=0; i<nBytes; i++){
		// wait until data register is empty
//		while( !(USART6->SR & 0x00000040) );
		while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
		USART_SendData(USART6, *(data+i));
	}
}

// Sends float

void USART_sendFloat_word8(float32_t *fdata){
	uint8_t *data, size;

	data = (uint8_t*)fdata;
	size = sizeof(data)/sizeof(uint8_t);

	for(uint8_t i=0; i<size; i++){
		while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
		USART_SendData(USART6, (uint16_t)(*(data+i)));
	}
}


// Sends many floats
void USART_sendManyFloat_word8(float32_t *fdata, uint8_t nfloat){
	uint8_t *data, size;

	for(uint8_t k=0; k<nfloat; k++){
		data = (uint8_t*)(fdata+k);
		size = sizeof(data)/sizeof(uint8_t);
		for(uint8_t i=0; i<size; i++){
			while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
			USART_SendData(USART6, (uint16_t)(*(data+i)));
		}
	}
}


