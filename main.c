#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "arm_math.h"
#include <math.h>
#include "i2c.h"
#include "usart.h"
#include "mpu9250.h"
#include "kalman.h"
#include "NewtonGauss.h"
#include "main.h"

int main(void){
	float32_t lastElapsedTime = 0.2;
	MPU9250_t mpu;
	MPU9250_Result_t mpuStatus;
	float32_t data[22], ypr[3];
	float32_t qKalman[4] = {0,0,0,1}, qNewtonGauss[4] = {0,0,0,1}, qInit[4] = {0,0,0,1}
			, qz[4] = {0,0,0,0}, Pinit[16] = {1,0,0,0,0,1,0,0 ,0,0,1,0,0,0,0,1};
	float32_t ngLoops = 0, ngFactor = 0;
	uint8_t count = 0;

	/*------------------------------------------------------------
	 ********************** Delay Configuration ******************
	 *-----------------------------------------------------------*/
	if (SysTick_Config(SystemCoreClock/1000)) // 1ms period
	    while (1);

	/*------------------------------------------------------------
	 ***************** Communication Configuration ****************
	 *-----------------------------------------------------------
	 * USART 6: baud 115200, no parity, 8bits, 1 stop bit, pins: Tx-PC6 Rx-PC7 */
	init_USART6();

	/* I2C1: clk: 100kHz, acknowledge on write and read, pins: SCK-PB6 SDA-PB7 */
	init_I2C1();

	/*------------------------------------------------------------
	 ******************* MPU9250 Configuration *******************
	 *-----------------------------------------------------------*/
	mpuStatus = mpu9250Init(&mpu, I2C1, MPU9250_ADDRESS);
	mpuStatus = configAccGyro(&mpu, GYRO_DLPF_41HZ, GYRO_FULL_SCALE_2000_DPS, ACC_FULL_SCALE_8_G, ACC_DLPF_41HZ);
	mpuStatus = configMag(&mpu, MAG_RES_16BITS);
	configOffset(&mpu);

	/*------------------------------------------------------------
	 *************** Sensors initial measurements ****************
	 *-----------------------------------------------------------*/
	//Gets reference measurements
	readReference(&mpu, 10);

	while(1){
		/* Restart elapsed time counter */
		msTicks = 0;
		count++;

		/* Get new measures from sensors */
		readSensors(&mpu);

		intGyro(&mpu, lastElapsedTime);

		/* Apply NewtonGauss algorithm */
		ApplyNewtonGauss(mpu.acc, mpu.mag, mpu.accRef, mpu.magRef, qInit, qNewtonGauss, &ngLoops, &ngFactor);

		/* Apply Kalman Filter */
		ApplyKalmanFilter(qNewtonGauss, qKalman, Pinit, mpu.gyro, lastElapsedTime, qKalman);

		/* Calculates Yaw Pitch e Roll angles */
		Quat2YPR(qKalman, ypr, mpu.gyro);

		if(count == 8){
			count = 0;
			for(uint8_t i=0; i<4; i++)
				data[i] = qKalman[i];
			for(uint8_t i=0; i<9; i++)
				data[i+4] = mpu.measure[i];
			for(uint8_t i=0; i<3; i++)
				data[i+13] = ypr[i];
			data[16] = lastElapsedTime;
			for(uint8_t i=0; i<3; i++)
				data[i+17] = mpu.angle[i];
			data[20] = ngLoops;
			data[21] = ngFactor;
			USART_sendManyFloat_word8(data, 22);
		}

		TimingDelay = 1; //10ms
	    Delay();
		lastElapsedTime = msTicks/100;
	}
}


void Delay(){
	//TimingDelay = nTime;
	while(TimingDelay != 0);
}

void SysTick_Handler(void){
	msTicks++;
	if (TimingDelay > 0x00)
		  TimingDelay--;
}

void Quat2YPR(float32_t *q, float32_t *ypr, float32_t *w){
	float32_t gravity[3] = {0,0,0};

	// x
	gravity[0] = 2 * (q[0]*q[2] - q[3]*q[1]);
	// y
	gravity[1] = 2 * (q[3]*q[0] + q[1]*q[2]);
	// z
	gravity[2] = q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2];

	 // yaw: (about Z axis)
	ypr[0] = -57.3 * atan2(2*q[0]*q[1] - 2*q[3]*q[2], 2*q[3]*q[3] + 2*q[0]*q[0] - 1);
	// pitch: (nose up/down, about Y axis)
	ypr[1] = -57.3 * atan2(gravity[0], sqrtf(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
	// roll: (tilt left/right, about X axis)
	ypr[2] = 57.3 * atan2(gravity[1], sqrtf(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
}

void qRotate3D(float32_t *q, float32_t *vect, float32_t *rotatedVect){
	arm_matrix_instance_f32 qMat, vectMat, dstMat;
	float32_t qMatData[9] = {0,0,0,0,0,0,0,0,0}, vectData[3] = {0,0,0}, dstData[3] = {0,0,0};

	// Left-handed clockwise
	qMatData[0] = 	   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	qMatData[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
	qMatData[2] = 2 * (q[0] * q[2] - q[1] * q[3]);

	qMatData[3] = 2 * (q[0] * q[1] - q[2] * q[3]);
	qMatData[4] = 	  -q[0] * q[0] + q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	qMatData[5] = 2 * (q[1] * q[2] + q[0] * q[3]);

	qMatData[6] = 2 * (q[0] * q[2] + q[1] * q[3]);
	qMatData[7] = 2 * (q[1] * q[2] - q[0] * q[3]);
	qMatData[8] =     -q[0] * q[0] - q[1] * q[1] + q[2] * q[2] + q[3] * q[3];

	for(uint8_t i=0; i<3; i++)
		vectData[i] = vect[i];

	arm_mat_init_f32(&qMat, 3, 3, qMatData);
	arm_mat_init_f32(&vectMat, 3, 1, vectData);
	arm_mat_init_f32(&dstMat, 3, 1, dstData);

	/* Apply rotation */
	arm_mat_mult_f32(&qMat, &vectMat, &dstMat);

	/* Update rotated vector */
	for(uint8_t i=0; i<3; i++)
		rotatedVect[i] = dstData[i];
}

void qRotate4D(float32_t *q, float32_t *p){
	arm_matrix_instance_f32 qMat, vectMat, dstMat;
	float32_t qMatData[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
			vectData[4] = {0,0,0,0}, dstData[4] = {0,0,0,0};

	qMatData[0]  =  q[3];
	qMatData[1]  = -q[2];
	qMatData[2]  =  q[1];
	qMatData[3]  =  q[0];

	qMatData[4]  =  q[2];
	qMatData[5]  =  q[3];
	qMatData[6]  = -q[0];
	qMatData[7]  =  q[1];

	qMatData[8]  = -q[1];
	qMatData[9]  =  q[0];
	qMatData[10] =  q[3];
	qMatData[11] =  q[2];

	qMatData[12] = -q[0];
	qMatData[13] = -q[1];
	qMatData[14] = -q[2];
	qMatData[15] =  q[3];

	for(uint8_t i=0; i<4; i++)
		vectData[i] = p[i];

	arm_mat_init_f32(&qMat, 4, 4, qMatData);
	arm_mat_init_f32(&vectMat, 4, 1, vectData);
	arm_mat_init_f32(&dstMat, 4, 1, dstData);

	/* Apply rotation */
	arm_mat_mult_f32(&qMat, &vectMat, &dstMat);

	/* Update rotated vector */
	for(uint8_t i=0; i<4; i++)
		p[i] = dstData[i];
}

