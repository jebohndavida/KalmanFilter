/* MPU9250 I2C Protocol - MPU9250REV1.0 p.34 */
#include "arm_math.h"
#include "mpu9250.h"
#include "i2c.h"
#include "main.h"
  
/*//////////////////////////////////////////////////////////////////*/
/*---------------------Registers read e write---------------------- */
/*//////////////////////////////////////////////////////////////////*/

// Write a byte (Data) in device (slaveAddress) at register (reg)
// Obs.:MPU9250 always acknowledge received data from master
void writeRegister(I2C_TypeDef *I2Cx, uint8_t slaveAddress, uint8_t reg, uint8_t data){
	uint8_t reg_data[2] = {reg, data};
	I2C_writeMult_ack(I2Cx, slaveAddress, 2, reg_data);
}

// Read 1byte from I2C device at address slaveAddress.
// Obs.:MPU9250 do not acknowledge last sent data to master
uint8_t readRegister(I2C_TypeDef *I2Cx, uint8_t slaveAddress, uint8_t reg){
	I2C_write_ack(I2Cx, slaveAddress, reg);
	return I2C_read_nack(I2Cx, slaveAddress);
}

// Write bytes to registers. reg_data = {reg1, data1, data2, data3, ...}. MPU increments reg address
// automatically (MPU9250REV1.0 p.34)
void writeMultRegister(I2C_TypeDef *I2Cx, uint8_t slaveAddress, uint8_t count, uint8_t *reg_data){
	I2C_writeMult_ack(I2Cx, slaveAddress, count, reg_data);
}

// Read Nbytes bytes from I2C device at address Address. MPU increments reg address
// automatically (MPU9250REV1.0 p.35)
void readMultRegister(I2C_TypeDef *I2Cx, uint8_t slaveAddress, uint8_t reg, uint8_t count, uint8_t* received_data){
	I2C_write_ack(I2Cx, slaveAddress, reg);
	I2C_readMult_ack(I2Cx, slaveAddress, count, received_data);
}

/*//////////////////////////////////////////////////////////////////*/
/*----------------------MPU9250 configuration-----------------------*/
/*//////////////////////////////////////////////////////////////////*/

MPU9250_Result_t mpu9250Init(MPU9250_t *mpux, I2C_TypeDef *I2Cx, uint8_t mpuAddress){
	I2C_TypeDef *var = I2C1;

	/* Initialize mpu9250 addresses */
	mpux->mpuAddress = mpuAddress;
	mpux->magAddress = MAG_ADDRESS;
	mpux->I2Cx = I2Cx;

	/* Verify identity */
	if(readRegister(mpux->I2Cx, mpux->mpuAddress, WHO_AM_I) != WHO_I_AM)
		return MPU9250_Result_Device_Is_Not_MPU9250;

	/* Wakeup mpu9250 */
	writeRegister(mpux->I2Cx, mpux->mpuAddress, PWR_MNGM_1, 0x00);

	/* Initialize mpu9250 struct */
	for(uint8_t i=0; i<9; i++){
		mpux->rawData[i] = 0;
		mpux->measure[i] = 0;
	}
	for(uint8_t i=0; i<3; i++){
		mpux->gyroOffset[i] = 0;
		mpux->magOffset[i] = 0;
		mpux->angle[i] = 0;
		mpux->acc[i] = 0;
		mpux->gyro[i] = 0;
		mpux->mag[i] = 0;
		mpux->accPrev[i] = 0;
		mpux->gyroPrev[i] = 0;
		mpux->magPrev[i] = 0;
	}
	TimingDelay = 4; // min 35ms
	Delay();

	return MPU9250_Result_Ok;
}


//FIFO config
//


/*//////////////////////////////////////////////////////////////////*/
/*----------------------Sensors configuration-----------------------*/
/*//////////////////////////////////////////////////////////////////*/
MPU9250_Result_t configAccGyro(MPU9250_t *mpux, uint8_t gyroFreq, uint8_t gyroScale, uint8_t accScale, uint8_t accFreq){
	uint8_t reg_data[5] = {GYRO_DLPF_REG, gyroFreq, gyroScale, accScale, accFreq};
	uint8_t received_data[4] = {0,0,0,0};

	writeMultRegister(mpux->I2Cx, mpux->mpuAddress, 5, reg_data);
	mpux->accDLPF = accFreq;
	mpux->accScale = accScale;
	mpux->gyroDLPF = gyroFreq;
	mpux->gyroScale = gyroScale;

	readMultRegister(mpux->I2Cx, mpux->mpuAddress, GYRO_DLPF_REG, 4, received_data);
	for(uint8_t i=0; i<4; i++)
		if(received_data[i] != reg_data[i+1])
			return MPU9250_AccGyroConfig_Error;
	return MPU9250_Result_Ok;
}

void configOffset(MPU9250_t *mpux){
	int16_t max[3] = {0x8000,0x8000,0x8000}; //minimum value is -32768
	int16_t min[3] = {0x7FFF,0x7FFF,0x7FFF}; //maximum value is 32767
	float32_t avg;
/*----------------------Gyroscope--------------------*/

	for(int count=0; count < 50; count++){
	  // Get samples from gyroscope
	  readGyro(mpux);
	  for(uint8_t i=0; i<3; i++){
		  //stores the maximum value
		  if(mpux->rawData[i+3] > max[i])
			  max[i] = mpux->rawData[i+3];
		  //stores the minimum value
		  if(mpux->rawData[i+3] < min[i])
			  min[i] = mpux->rawData[i+3];
	  }
	TimingDelay = 1; //20ms
	Delay();
	}

	mpux->gyroOffset[0] = (max[0] + min[0])/2;
	mpux->gyroOffset[1] = (max[1] + min[1])/2;
	mpux->gyroOffset[2] = (max[2] + min[2])/2;


/*----------------------Magnetometer--------------------*/
	mpux->magOffset[0] = 26;
	mpux->magOffset[1] = 14;
	mpux->magOffset[2] = 35;
	mpux->magScale[0] = 1.154;
	mpux->magScale[1] = 1.17;
	mpux->magScale[2] = 1.1;
}

/*---------------------------Gyroscope-----------------------------*/


/*-------------------------Acceletormeter---------------------------*/


/*-------------------------Magnetometer---------------------------*/
MPU9250_Result_t configMag(MPU9250_t *mpux, uint8_t resolution){
	// Set by pass mode for magnetometer configuration
	for (uint32_t i = 0; i < 10000; i++);
	writeRegister(mpux->I2Cx, mpux->mpuAddress, MPU9250_I2C_BYPASS_REG, MPU9250_I2C_BYPASS_EN<<1);
	// Request 16bits (8Hz) or 14bits p.15 of datasheet
	writeRegister(mpux->I2Cx, mpux->magAddress, MAG_RES_REG, resolution);
	if(readRegister(mpux->I2Cx, mpux->magAddress, MAG_RES_REG) != resolution)
		return MPU9250_MagConfig_Error;
	// Set by pass mode to read magnetometer
	writeRegister(mpux->I2Cx, mpux->mpuAddress, MPU9250_I2C_BYPASS_REG, MPU9250_I2C_BYPASS_EN<<1);

	return MPU9250_Result_Ok;
 }


/*//////////////////////////////////////////////////////////////////*/
/*-------------------Data reading and conversions-------------------*/
/*//////////////////////////////////////////////////////////////////*/

void readMag(MPU9250_t *mpux){
	uint8_t Buf[6] = {0,0,0,0,0,0};

	// Read register ST1 and wait for the DRDY(data ready) bit
	while(!(readRegister(mpux->I2Cx, mpux->magAddress, MAG_STA1_REG) & MAG_DATA_RDY_BIT));
	// Read magnetometer data
	readMultRegister(mpux->I2Cx, mpux->magAddress, MAG_DATA_XL, 6, Buf);
	// Required to read ST2 after data reading is complete (AK8963.pdf p.30)
	readRegister(mpux->I2Cx, mpux->magAddress, MAG_STA2_REG);
	// Create 16 bits values from 8 bits data
	// Swap x and y axis and -z because of mag orientation
	mpux->rawData[6] = (Buf[1]<<8 | Buf[0]);
	mpux->rawData[7] = (Buf[3]<<8 | Buf[2]);
	mpux->rawData[8] = -(Buf[5]<<8 | Buf[4]); //left-handed
}

void readMagSmooth(MPU9250_t *mpux, uint8_t nSamples){
	float32_t max[3] = {-4912, -4912, -4912};
	float32_t min[3] = {4912, 4912, 4912};

	for(uint8_t count=0; count<2*nSamples; count++){
		// Get samples from gyroscope
		readMag(mpux);
		data2measure(mpux);
		for(uint8_t i=0; i<3; i++){
		  //stores the maximum value
		  if(mpux->measure[i+6] > max[i])
			  max[i] = mpux->measure[i+6];
		  //stores the minimum value
		  if(mpux->measure[i+6] < min[i])
			  min[i] = mpux->measure[i+6];
		}
		TimingDelay = 1;
		Delay();
	}
	for(uint8_t i=0; i<3; i++)
		mpux->measure[i+6] = (max[i] + min[i])/2;
}
  
void readAcc(MPU9250_t *mpux){
	uint8_t Buf[6] = {0,0,0,0,0,0};

	// Read 9 registers in sequence
	readMultRegister(mpux->I2Cx, mpux->mpuAddress, ACC_DATA_XH, 6, Buf);
	mpux->rawData[1] = (Buf[0]<<8 | Buf[1]); //left-handed
	mpux->rawData[0] = (Buf[2]<<8 | Buf[3]);
	mpux->rawData[2] = (Buf[4]<<8 | Buf[5]);
}


void readGyro(MPU9250_t *mpux){
	uint8_t Buf[9] = {0,0,0,0,0,0};

	// Read 9 registers in sequence
	readMultRegister(mpux->I2Cx, mpux->mpuAddress, GYRO_DATA_XH, 6, Buf);
	mpux->rawData[4] = -(Buf[0]<<8 | Buf[1]); //left-handed
	mpux->rawData[3] = -(Buf[2]<<8 | Buf[3]);
	mpux->rawData[5] = -(Buf[4]<<8 | Buf[5]);
}

void readGyroSmooth(MPU9250_t *mpux, uint8_t nSamples){
	float32_t max[3] = {-1000, -1000, -1000};
	float32_t min[3] = {1000, 1000, 1000};

	for(uint8_t count=0; count<nSamples; count++){
		// Get samples from gyroscope
		readGyro(mpux);
		data2measure(mpux);
		for(uint8_t i=0; i<3; i++){
		  //stores the maximum value
		  if(mpux->measure[i+3] > max[i])
			  max[i] = mpux->measure[i+3];
		  //stores the minimum value
		  if(mpux->measure[i+3] < min[i])
			  min[i] = mpux->measure[i+3];
		}
		TimingDelay = 1;
		Delay();
	}
	for(uint8_t i=0; i<3; i++)
		mpux->measure[i+3] = (max[i] + min[i])/2;
}

void readAccGyro(MPU9250_t *mpux){
	uint8_t Buf[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	// Read 14 registers in sequence
	readMultRegister(mpux->I2Cx, mpux->mpuAddress, ACC_DATA_XH, 14, Buf);
	// Create 16 bits values from 8 bits data
	// Accelerometer
	mpux->rawData[1] = (Buf[0]<<8 | Buf[1]);
	mpux->rawData[0] = (Buf[2]<<8 | Buf[3]);
	mpux->rawData[2] = (Buf[4]<<8 | Buf[5]);
	// bytes 6 and 7 are for temperature sensors
	// Gyroscope
	mpux->rawData[4] = -(Buf[8]<<8 | Buf[9]);
	mpux->rawData[3] = -(Buf[10]<<8 | Buf[11]);
	mpux->rawData[5] = -(Buf[12]<<8 | Buf[13]);
}

void readSensors(MPU9250_t *mpux){
//	updatePrevMeasure(mpux);
	readAccGyro(mpux);
	readMag(mpux);
	data2measure(mpux);
}

void readSensorsSmooth(MPU9250_t *mpux, uint32_t nSamples){
	updatePrevMeasure(mpux);
	readAcc(mpux);
	readGyroSmooth(mpux, nSamples);
	readMagSmooth(mpux, nSamples);
}

void readReference(MPU9250_t *mpux, uint32_t nSamples){
	readSensorsSmooth(mpux, nSamples);
	// Update measurement vectors
	for(uint8_t i=0; i<3; i++){
		mpux->accRef[i] = mpux->measure[i];
		mpux->gyroRef[i] = mpux->measure[i+3];
		mpux->magRef[i] = mpux->measure[i+6];
	}
	// Initiate integrated gyroscope
	for(uint8_t i=0; i<3; i++)
		mpux->angle[i] = 0;
}


void data2measure(MPU9250_t *mpux){
	float32_t ang2rad = PI/180;
	float32_t g2mps = 9.81;
	float32_t aux[3];

	// Converts accelerometer data to acceleration measurements [g]
	// according to scale sensitivity
	switch(mpux->accScale){
	case ACC_FULL_SCALE_2_G:
		for(uint8_t i=0; i<3; i++){
			mpux->measure[i] = (mpux->rawData[i]*0.0610)/1000; //m/s²
		}
		break;
	case ACC_FULL_SCALE_4_G:
		for(uint8_t i=0; i<3; i++){
			mpux->measure[i] = (mpux->rawData[i]*0.12207)/1000; //m/s²
		}
		break;
	case ACC_FULL_SCALE_8_G:
		for(uint8_t i=0; i<3; i++){
			mpux->measure[i] = (mpux->rawData[i]*0.2441)/1000; //m/s²
		}
		break;
	case ACC_FULL_SCALE_16_G:
		for(uint8_t i=0; i<3; i++){
			mpux->measure[i] = (mpux->rawData[i]*0.48828)/1000; //m/s²
		}
		break;
	}

	// Converts gyroscope data to angular velocities measurements [DPS]
	// according to scale sensitivity (see p.8 of datasheet)
	switch(mpux->gyroScale){
	case GYRO_FULL_SCALE_250_DPS:
		for(uint8_t i=3; i<6; i++){
			mpux->measure[i] = (mpux->rawData[i] - mpux->gyroOffset[i-3])*0.000763*ang2rad; //rad/s
		}
		break;
	case GYRO_FULL_SCALE_500_DPS:
		for(uint8_t i=3; i<6; i++){
			mpux->measure[i] = (mpux->rawData[i] - mpux->gyroOffset[i-3])*0.01527*ang2rad; //rad/s
		}
		break;
	case GYRO_FULL_SCALE_1000_DPS:
		for(uint8_t i=3; i<6; i++){
			mpux->measure[i] = (mpux->rawData[i] - mpux->gyroOffset[i-3])*0.03049*ang2rad; //rad/s
		}
		break;
	case GYRO_FULL_SCALE_2000_DPS:
		for(uint8_t i=3; i<6; i++){
			mpux->measure[i] = (mpux->rawData[i] - mpux->gyroOffset[i-3])*0.06097*ang2rad;
		}
		break;
	}

	// Converts magnetometer data to real magnectic field measurements [uT]
	// according to scale sensitivity in 16bits (0.15uT/LSB) RM-MPU-9250A-00.pdf p.50.
	// Earth magnetic field 0º lat and 0º long is 23uT.
	// 1T - e4G
	// 23e-6T - x
	// 1G - e-4T -> x = 23e-6T = 23e-2G = 230mG
	//http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
	for(uint8_t i=6; i<9; i++){
		//aux[i] = (float32_t)(mpux->rawData[i] - mpux->magOffset[i-6]);
		mpux->measure[i] = mpux->magScale[i-6]*((0.15*(float32_t)mpux->rawData[i]) - mpux->magOffset[i-6]);//0.15*(float32_t)mpux->rawData[i];
	}

	// Update measurement vectors
	for(uint8_t i=0; i<3; i++){
		mpux->acc[i] = mpux->measure[i];
		mpux->gyro[i] = mpux->measure[i+3];
		mpux->mag[i] = mpux->measure[i+6];
	}
}

void updatePrevMeasure(MPU9250_t *mpux){
	// Save previous measurements
	for(uint8_t i=0; i<3; i++){
		mpux->accPrev[i] = mpux->acc[i];
		mpux->gyroPrev[i] = mpux->gyro[i];
		mpux->magPrev[i] = mpux->mag[i];
	}
}

void intGyro(MPU9250_t *mpux, float32_t Ts){
	float32_t rad2ang = 57.3;
	// Converts angular velocities to angle in degrees
	// utilizing integration with zero order holder
	for(uint8_t i=0; i<3; i++){
		if(mpux->angle[i] < 1000 || mpux->angle[i] > -1000)
			mpux->angle[i] += (rad2ang*(mpux->measure[i+3] * Ts));
	}
}

void verify(float32_t max, float32_t min, float32_t *value){
	if(*value > max)
		*value = max;
	if(*value < min)
		*value = min;
}

