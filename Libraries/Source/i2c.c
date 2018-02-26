/* https://github.com/g4lvanix/STM32F4-workarea/blob/master/Project/I2C-master-example/main.c */
/* https://github.com/g4lvanix/STM32F4-examples/blob/master/I2C%20Master/main.c */
#include "i2c.h"

/*
       |PINSPACK 1   |PINSPACK 2   |PINSPACK 3
I2CX   |SCL   SDA    |SCL   SDA    |SCL   SDA
I2C1   |PB6   PB7    |PB8   PB9    |PB6   PB9
I2C2   |PB10  PB11   |PF1   PF0    |PH4   PH5
I2C3   |PA8   PC9    |PH7   PH8    |-     -
*/
void init_I2C1(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB

	// Connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

	// configure I2C1
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1

	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t slaveAddress, uint8_t direction){
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, slaveAddress, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
void I2C_write_ack(I2C_TypeDef* I2Cx, uint8_t slaveAddress, uint8_t data){
	// Generate start condition
	I2C_start(I2Cx, slaveAddress<<1, I2C_Direction_Transmitter);
	// Enable acknowledge check sent from slave at the end of data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	// Generate stop condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx, uint8_t slaveAddress){
	uint8_t data = 0;

	// Generate start condition
	I2C_start(I2Cx, slaveAddress<<1, I2C_Direction_Receiver);
	// Enable acknowledge check sent from slave at the end of data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// Wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// Read data from I2C data register and return data byte
	data = I2C_ReceiveData(I2Cx);
	// Generate stop condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
	return data;
}

void I2C_writeMult_ack(I2C_TypeDef* I2Cx, uint8_t slaveAddress, uint8_t num, uint8_t* all_data){
	uint8_t data = 0;

	I2C_start(I2Cx, slaveAddress<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	for(uint8_t i=0; i<num; i++){
		data = all_data[i];
		// Enable acknowledge check sent from slave at the end of data
		I2C_AcknowledgeConfig(I2Cx, ENABLE);
		I2C_SendData(I2Cx, data);
		// wait for I2C1 EV8_2 --> byte has been transmitted
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_writeMult_nack(I2C_TypeDef* I2Cx, uint8_t slaveAddress, uint8_t num, uint8_t* all_data){
	uint8_t data = 0;

	I2C_start(I2Cx, slaveAddress<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	for(uint8_t i=0; i<num-1; i++){
		data = all_data[i];
		// Enable acknowledge check sent from slave at the end of data
		I2C_AcknowledgeConfig(I2Cx, ENABLE);
		I2C_SendData(I2Cx, data);
		// wait for I2C1 EV8_2 --> byte has been transmitted
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}
	data = all_data[num-1];
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_SendData(I2Cx, data);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_readMult_ack(I2C_TypeDef* I2Cx, uint8_t slaveAddress, uint8_t num, uint8_t *received_data){
	// Start a transmission in Master receiver mode
	I2C_start(I2Cx, slaveAddress<<1, I2C_Direction_Receiver);
	for(uint8_t i=0; i<(num-1); i++){
		// Enable acknowledge check sent from slave at the end of data
		I2C_AcknowledgeConfig(I2Cx, ENABLE);
		// Wait until one byte has been received
		while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
		// Read data from I2C data register and return data byte
		received_data[i] = I2C_ReceiveData(I2Cx);
	}

	// Slave do not acknowledge the last byte
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// Wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// Read data from I2C data register and return data byte
	received_data[num-1] = I2C_ReceiveData(I2Cx);
	I2C_GenerateSTOP(I2Cx, ENABLE);
}


/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx, uint8_t slaveAddress){
	// Start a transmission in Master receiver mode
	I2C_start(I2Cx, slaveAddress<<1, I2C_Direction_Receiver);
	// disable acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// nack also generates stop condition after last byte received
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}
