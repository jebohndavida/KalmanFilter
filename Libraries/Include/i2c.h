#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#ifndef _i2c_H_
#define _i2c_H_

void init_I2C1(void);
void init_I2C2(void);
void I2C_start(I2C_TypeDef* I2Cx, uint8_t slaveAddress, uint8_t direction);
void I2C_write_ack(I2C_TypeDef* I2Cx, uint8_t slaveAddress, uint8_t data);
void I2C_writeMult_ack(I2C_TypeDef* I2Cx, uint8_t slaveAddress, uint8_t num, uint8_t* all_data);
void I2C_writeMult_nack(I2C_TypeDef* I2Cx, uint8_t slaveAddress, uint8_t num, uint8_t* all_data);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx, uint8_t slaveAddress);
void I2C_readMult_ack(I2C_TypeDef* I2Cx, uint8_t slaveAddress, uint8_t num, uint8_t *received_data);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx, uint8_t slaveAddress);

#endif
