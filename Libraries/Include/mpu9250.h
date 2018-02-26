#include "stm32f4xx.h" // for uint8_t type
#include "stm32f4xx_i2c.h"
#include "arm_math.h"

#ifndef _mpu9250_H_
#define _mpu9250_H_

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
#define    PWR_MNGM_1 				  0x6B
#define    WHO_AM_I					  0x75
#define    WHO_I_AM					  0x71
#define    MPU9250_I2Cx_CLK			  400000
#define    MPU9250_I2Cx				  I2C1
#define    MPU9250_I2C_BYPASS_REG	  0x37
#define    MPU9250_I2C_BYPASS_EN	  0x01
#define    MPU9250_TS				  33 //ms

#define    GYRO_DLPF_REG	          0x1A
#define    GYRO_SCALE_REG	          0x1B
#define    GYRO_DATA_XH 	          0x43
#define    GYRO_DATA_XL 	          0x44
#define    GYRO_DATA_YH 	          0x45
#define    GYRO_DATA_YL 	          0x46
#define    GYRO_DATA_ZH 	          0x47
#define    GYRO_DATA_ZL 	          0x48
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
#define    GYRO_DLPF_5HZ              0x06 //delay 33.48ms
#define    GYRO_DLPF_10HZ             0x05 //delay 17.85ms
#define    GYRO_DLPF_20HZ             0x04 //delay 9.9ms
#define    GYRO_DLPF_41HZ             0x03 //delay 5.9ms
#define    GYRO_DLPF_92HZ             0x02
#define    GYRO_DLPF_184HZ            0x01
#define    GYRO_DLPF_250HZ            0x00
#define    GYRO_DLPF_3600HZ           0x30 //any value
#define    GYRO_DLPF_8800HZ           0x40 //any value
#define    GYRO_FCHOICE_B_MODE_0      0x00 //default
#define    GYRO_FCHOICE_B_MODE_1      0x01 //Bw 8.8kHz Fs 32kHz
#define    GYRO_FCHOICE_B_MODE_2      0x02 //Bw 3.6kHz Fs 32kHz
#define    GYRO_DLPF_5HZ_TS           33.48
#define    GYRO_DLPF_41HZ_TS          5.9
#define    GYRO_DLPF_92HZ_TS          3.9
#define    GYRO_DLPF_184HZ_TS         1
#define    GYRO_DLPF_250HZ_TS         0.97
#define    GYRO_DLPF_3600HZ_M0_TS     0.17
#define    GYRO_DLPF_3600HZ_M2_TS     0.11
#define    GYRO_DLPF_8800HZ_TS        0.064

#define    ACC_SCALE_REG         	  0x1C
#define    ACC_DLPF_REG         	  0x1D
#define	   ACC_DATA_XH				  0x3B
#define	   ACC_DATA_XL				  0x3C
#define	   ACC_DATA_YH				  0x3D
#define	   ACC_DATA_YL				  0x3E
#define	   ACC_DATA_ZH				  0x3F
#define	   ACC_DATA_ZL				  0x40
#define    ACC_FULL_SCALE_2_G         0x00
#define    ACC_FULL_SCALE_4_G         0x08 //8
#define    ACC_FULL_SCALE_8_G         0x10 //16
#define    ACC_FULL_SCALE_16_G        0x18 //24
#define    ACC_DLPF_5HZ               0x06 //67ms
#define    ACC_DLPF_10HZ              0x05 //delay 35.70ms
#define    ACC_DLPF_20HZ              0x04 //delay 19.80ms
#define    ACC_DLPF_41HZ              0x03 //11.8ms
#define    ACC_DLPF_92HZ              0x02
#define    ACC_DLPF_184HZ             0x01

#define	   MAG_RES_REG				  0x0A
#define	   MAG_RES_16BITS			  0x16
#define	   MAG_RES_14BITS			  0x10
#define	   MAG_DATA_XL				  0x03
#define	   MAG_DATA_XH				  0x04
#define	   MAG_DATA_YL				  0x05
#define	   MAG_DATA_YH				  0x06
#define	   MAG_DATA_ZL				  0x07
#define	   MAG_DATA_ZH				  0x08
#define	   MAG_STA1_REG  			  0x02
#define	   MAG_STA2_REG  			  0x09
#define	   MAG_DATA_RDY_BIT			  0x01

#define	   TEMP_DATA_H				  0x41
#define	   TEMP_DATA_L				  0x42

typedef enum{
	MPU9250_Result_Ok,          			// Everything OK
	MPU9250_Result_Device_Is_Not_MPU9250,	// Connected device with address is not MPU6050
	MPU9250_MagConfig_Error,				// Mag configuration error
	MPU9250_AccGyroConfig_Error				// AccGyro configuration error
} MPU9250_Result_t;

typedef struct MPU9250Struct{
        uint8_t magAddress;// = 0x0C;
        uint8_t mpuAddress;// = 0x68;
        I2C_TypeDef *I2Cx;
        uint8_t accScale;
        uint8_t accDLPF;
        uint8_t gyroScale;
        uint8_t gyroDLPF;
        uint8_t magMode;
        float32_t accOffset[3];
        float32_t gyroOffset[3];
        float32_t magOffset[3];
        float32_t magScale[3];
        float32_t gyroTs;
        int16_t rawData[9];
        float32_t measure[9];
        float32_t smoothMeasure[9];
        float32_t acc[3];
        float32_t gyro[3];
        float32_t mag[3];
        float32_t accPrev[3];
        float32_t gyroPrev[3];
        float32_t magPrev[3];
        float32_t accRef[3];
        float32_t gyroRef[3];
        float32_t magRef[3];
        float32_t angle[3];

} MPU9250_t;

/* I2C read/write register functions */
void writeRegister(I2C_TypeDef*, uint8_t, uint8_t, uint8_t);
void writeMultRegister(I2C_TypeDef*, uint8_t, uint8_t, uint8_t*);
uint8_t readRegister(I2C_TypeDef*, uint8_t, uint8_t);
void readMultRegister(I2C_TypeDef*, uint8_t, uint8_t, uint8_t, uint8_t*);

/* MPU9250 read/write register functions */
MPU9250_Result_t mpu9250Init(MPU9250_t*, I2C_TypeDef*, uint8_t);
MPU9250_Result_t configAccGyro(MPU9250_t*, uint8_t, uint8_t, uint8_t, uint8_t);
MPU9250_Result_t configMag(MPU9250_t*, uint8_t);
void configOffset(MPU9250_t*);
void readMag(MPU9250_t*);
void readMagSmooth(MPU9250_t*, uint8_t);
void readAcc(MPU9250_t*);
void readGyro(MPU9250_t*);
void readGyroSmooth(MPU9250_t*, uint8_t);
void readAccGyro(MPU9250_t*);
void readSensors(MPU9250_t*);
void readSensorsSmooth(MPU9250_t*, uint32_t);
void readReference(MPU9250_t*, uint32_t);
void updatePrevMeasure(MPU9250_t*);
void data2measure(MPU9250_t*);
void intGyro(MPU9250_t*, float32_t);
void getSmoothMeasures(MPU9250_t *mpux);
void verify(float32_t max, float32_t min, float32_t *value);

#endif
