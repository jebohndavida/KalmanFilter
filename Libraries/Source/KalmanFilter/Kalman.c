/* Observations:
 * arm_mat_inverse_f32 modifies source matrix
 * arm_mat_sub_f32 does not modify source matrices
 * arm_mat_add_f32  does not modify source matrices
 * Do not pass variables through references to matrix function resolves the problem of
 * modifying src matrices and eliminate the need for matCopy matrices */

#include <math.h>
#include "arm_math.h"
#include "Kalman.h"
#include "main.h"

void KalmanInit(Kalman_t *k){
	k->nRows = 4;
	k->nCols = 4;
	k->wMod = 0;
	k->qMod = 0;
	k->Pmod = 100;

	for(uint8_t i=0; i<16; i++){
		k->AData[i] = 0;
		k->KData[i] = 0;
		k->HData[i] = 0;
		k->HtData[i] = 0;
		k->QData[i] = 0;
		k->RData[i] = 0;
		k->EyeData[i] = 0;
	}

	k->HData[0] = 1;
	k->HData[5] = 1;
	k->HData[10] = 1;
	k->HData[15] = 1;

	k->HtData[0] = 1;
	k->HtData[5] = 1;
	k->HtData[10] = 1;
	k->HtData[15] = 1;

	k->QData[0] = 1e-6;
	k->QData[5] = 1e-6;
	k->QData[10] = 1e-6;
	k->QData[15] = 1e-6;

	k->RData[0] = 5e-5;
	k->RData[5] = 5e-5;
	k->RData[10] = 5e-5;
	k->RData[15] = 5e-5;

	k->EyeData[0] = 1;
	k->EyeData[5] = 1;
	k->EyeData[10] = 1;
	k->EyeData[15] = 1;

	/* Initialize Kalman Matrices */
	arm_mat_init_f32(&k->A, k->nRows, k->nCols, k->AData);
	arm_mat_init_f32(&k->K, k->nRows, k->nCols, k->KData);
	arm_mat_init_f32(&k->q, k->nRows, 1, k->qData);
	arm_mat_init_f32(&k->qPriori, k->nRows, 1, k->qPrioriData);
	arm_mat_init_f32(&k->P, k->nRows, k->nCols, k->PData); //
	arm_mat_init_f32(&k->Ppriori, k->nRows, k->nCols, k->PprioriData);
	arm_mat_init_f32(&k->H, k->nRows, k->nCols, k->HData);
	arm_mat_init_f32(&k->Ht, k->nRows, k->nCols, k->HtData); // H' is the same
	arm_mat_init_f32(&k->Q, k->nRows, k->nCols, k->QData);
	arm_mat_init_f32(&k->R, k->nRows, k->nCols, k->RData);
	arm_mat_init_f32(&k->Eye, k->nRows, k->nCols, k->EyeData);
	arm_mat_init_f32(&k->qz, k->nRows, 1, k->qzData);
	arm_mat_init_f32(&k->w, 3, 1, k->wData);
	arm_mat_init_f32(&k->omega, k->nRows, k->nCols, k->omegaData);

}

void ApplyKalmanFilter(float32_t *qNewtonGauss, float32_t *qInit, float32_t *Pinit, float32_t *gyroMeasure,
		float32_t elapsedTime, float32_t *qDst){
	float32_t scale = 0;
	uint8_t count = 0;
	float32_t PmodBest = 100;

	Kalman_t k;

	/* Auxiliary matrices */
	arm_matrix_instance_f32 auxA_4_1;
	arm_matrix_instance_f32 auxB_4_1;
	arm_matrix_instance_f32 auxA_4_4;
	arm_matrix_instance_f32 auxB_4_4;
	arm_matrix_instance_f32 auxC_4_4;
	arm_matrix_instance_f32 auxD_4_4;
	float32_t auxAData_4_1[4];
	float32_t auxBData_4_1[4];
	float32_t auxAData_4_4[16];
	float32_t auxBData_4_4[16];
	float32_t auxCData_4_4[16];
	float32_t auxDData_4_4[16];

	KalmanInit(&k);

	arm_mat_init_f32(&auxA_4_1, 4, 1, auxAData_4_1);
	arm_mat_init_f32(&auxB_4_1, 4, 1, auxBData_4_1);
	arm_mat_init_f32(&auxA_4_4, 4, 4, auxAData_4_4);
	arm_mat_init_f32(&auxB_4_4, 4, 4, auxBData_4_4);
	arm_mat_init_f32(&auxC_4_4, 4, 4, auxCData_4_4);
	arm_mat_init_f32(&auxD_4_4, 4, 4, auxDData_4_4);

	/*------------------------------------------------------------
	 ********************* Update Measurements *******************
	 *------------------------------------------------------------
	Update measured (NewtonGauss) and initial quaternions */
	for(uint8_t i=0; i<4; i++){
		k.qzData[i] = qNewtonGauss[i];
		k.qData[i] = qInit[i];
		qDst[i] = qInit[i];
	}
	for(uint8_t i=0; i<16; i++)
		k.PData[i] = Pinit[i];
	/* Update sampling rate */
	k.Ts = elapsedTime;

	/* Update angular velocity */
	for(uint8_t i=0; i<3; i++)
		k.wData[i] = gyroMeasure[i];

	/* Updates Omega matrix
	 |          0,  wData[2],  -wData[1], wData[0],|
	 |-wData[2],           0,   wData[0], wData[1],|
	 | wData[1], -wData[0],            0, wData[2],|
	 |-wData[0], -wData[1],  -wData[2],          0 |
	*/

	k.omegaData[0] =  0;
	k.omegaData[1] =  k.wData[2];
	k.omegaData[2] = -k.wData[1];
	k.omegaData[3] =  k.wData[0];

	k.omegaData[4] = -k.wData[2];
	k.omegaData[5] =  0;
	k.omegaData[6] =  k.wData[0];
	k.omegaData[7] =  k.wData[1];

	k.omegaData[8] =  k.wData[1];
	k.omegaData[9] = -k.wData[0];
	k.omegaData[10] = 0;
	k.omegaData[11] = k.wData[2];

	k.omegaData[12] = -k.wData[0];
	k.omegaData[13] = -k.wData[1];
	k.omegaData[14] = -k.wData[2];
	k.omegaData[15] =  0;

	vectModK(k.wData, &k.wMod, 3);

	/*------------------------------------------------------------
	 ******************* Prediction: time update *****************
	 *------------------------------------------------------------
	 * Process Matrix update:
	 * o.A = eye(4) + 0.5*Ts*o.omega - 0.125*Ts^2*o.wMod*eye(4); */
	 scale = 0.125*k.Ts*k.Ts*k.wMod*k.wMod;
	 arm_mat_scale_f32(&k.Eye, scale, &auxA_4_4);
	 scale = 0.5*k.Ts;
	 arm_mat_scale_f32(&k.omega, scale, &auxB_4_4);
	 arm_mat_sub_f32(&auxB_4_4, &auxA_4_4, &auxC_4_4);
	 arm_mat_add_f32(&k.Eye, &auxC_4_4, &k.A);

	/* Quaternion prediction:
	 *  o.qPriori = o.A * q; */
	 arm_mat_mult_f32(&k.A, &k.q, &k.qPriori);

	 /* Covariance Matrix prediction:
	  *  o.Ppriori = o.A * o.P * o.A' + o.Q; */
	 arm_mat_mult_f32(&k.A, &k.P, &auxA_4_4);
	 arm_mat_trans_f32(&k.A, &auxB_4_4);
	 arm_mat_mult_f32(&auxA_4_4, &auxB_4_4, &auxC_4_4);
	 arm_mat_add_f32(&auxC_4_4, &k.Q, &k.Ppriori);


	/*------------------------------------------------------------
	 **************** Estimation: measurement update *************
	 *------------------------------------------------------------
	 * Kalman Gain:
	 * o.K = o.Ppriori*o.H'/(o.H*o.Ppriori*o.H' + o.R); */
	 arm_mat_mult_f32(&k.Ppriori, &k.Ht, &auxA_4_4);
	 arm_mat_mult_f32(&k.H, &k.Ppriori, &auxB_4_4);
	 arm_mat_mult_f32(&auxB_4_4, &k.Ht, &auxC_4_4);
	 arm_mat_add_f32(&auxC_4_4, &k.R, &auxB_4_4);
	 updateMatValueK(&auxB_4_4, &auxD_4_4);
	 arm_mat_inverse_f32(&auxD_4_4, &auxC_4_4);
	 arm_mat_mult_f32(&auxA_4_4, &auxC_4_4, &k.K);

	 /* Quaternion estimation:
	  * o.q = o.qPriori + o.K*(o.z - (o.H*o.qPriori)); */
	 arm_mat_mult_f32(&k.H, &k.qPriori, &auxA_4_1);
	 arm_mat_sub_f32(&k.qz, &auxA_4_1, &auxB_4_1);
	 arm_mat_mult_f32(&k.K, &auxB_4_1, &auxA_4_1);
	 arm_mat_add_f32(&k.qPriori, &auxA_4_1, &k.q);

	 /* Quaternion Normalization:
	  * o.q = o.q/sqrt(o.q(1)^2+o.q(2)^2+o.q(3)^2+o.q(4)^2); */
	 vectModK(k.qData, &k.qMod, 4);
	 if(k.qMod>0)
		 arm_mat_scale_f32(&k.q, 1/k.qMod, &k.q);

	 /* Covariance Matrix Estimation:
	  * o.P = o.Ppriori*eye(4) - o.Ppriori*o.K*o.H; */
	 arm_mat_mult_f32(&k.Ppriori, &k.Eye, &auxA_4_4);
	 arm_mat_mult_f32(&k.Ppriori, &k.K, &auxB_4_4);
	 arm_mat_mult_f32(&auxB_4_4, &k.H, &auxC_4_4);
	 arm_mat_sub_f32(&auxA_4_4, &auxC_4_4, &k.P);

	 for(uint8_t i=0; i<4; i++)
		qDst[i] = k.qData[i];

	 for(uint8_t i=0; i<16; i++)
		Pinit[i] = k.PData[i];
}

void vectModK(float32_t *src, float32_t *dst, uint8_t size){
	dst[0] = 0;
	for(uint8_t i=0; i < size; i++)
		dst[0] += (src[i]*src[i]);
	dst[0] = sqrtf(dst[0]);
}

void updateMatValueK(arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst){
	for(uint8_t i=0; i<(src->numCols*src->numRows); i++)
		dst->pData[i] = src->pData[i];
}



