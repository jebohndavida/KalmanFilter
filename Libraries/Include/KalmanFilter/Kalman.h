#include "arm_math.h"

#ifndef _Kalman_H_
#define _Kalman_H_

typedef struct KalmanStruct{
	/* Kalman Matrices */
	uint16_t nRows;
	uint16_t nCols;
	float32_t Ts;
	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 P;
	arm_matrix_instance_f32 Ppriori;
	arm_matrix_instance_f32 H;
	arm_matrix_instance_f32 Ht;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 R;
	arm_matrix_instance_f32 omega;
	arm_matrix_instance_f32 Eye;
	arm_matrix_instance_f32 q;
	arm_matrix_instance_f32 qPriori;
	arm_matrix_instance_f32 qz;
	arm_matrix_instance_f32 w;

	float32_t qMod;
	float32_t wMod;
	/* Kalman data vectors */
	float32_t qData[4];
	float32_t qPrioriData[4];
	float32_t qzData[4];
	float32_t wData[3];
	float32_t AData[16];
	float32_t KData[16];
	float32_t PData[16];
	float32_t PprioriData[16];
	float32_t Pmod;  // Covariance matrix
	float32_t HData[16];
	float32_t HtData[16];
	float32_t QData[16];
	float32_t RData[16];
	float32_t omegaData[16];
	float32_t EyeData[16];

	uint8_t lastQuadrant;

} Kalman_t;

void KalmanInit(Kalman_t*);
void ApplyKalmanFilter(float32_t *qNewtonGauss, float32_t *qInit, float32_t *Pinit, float32_t *gyroMeasure,
		float32_t elapsedTime, float32_t *qDst);
void vectModK(float32_t*, float32_t*, uint8_t);
void updateMatValueK(arm_matrix_instance_f32*, arm_matrix_instance_f32*);

#endif
