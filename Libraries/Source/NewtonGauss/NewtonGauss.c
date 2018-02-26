#include <math.h>
#include "arm_math.h"
#include "NewtonGauss.h"
//#include "matrixHelper.h"

void NewtonGaussInit(NewtonGauss_t *ng){
	/* NewtonGauss Matrices */
	arm_mat_init_f32(&ng->Jacob, 6, 4, ng->JacobData);
	arm_mat_init_f32(&ng->Jacobt, 4, 6, ng->JacobtData);
	arm_mat_init_f32(&ng->R, 3, 3, ng->RData);
	arm_mat_init_f32(&ng->E, 6, 1, ng->EData);
	arm_mat_init_f32(&ng->q, 4, 1, ng->qd);
	arm_mat_init_f32(&ng->acc, 3, 1, ng->accd);
	arm_mat_init_f32(&ng->accPrev, 3, 1, ng->accdPrev);
	arm_mat_init_f32(&ng->mag, 3, 1, ng->magd);
	arm_mat_init_f32(&ng->magPrev, 3, 1, ng->magdPrev);
}

void ApplyNewtonGauss(float32_t *accd, float32_t *magd, float32_t *accdPrev,
		float32_t *magdPrev, float32_t *qInit, float32_t *qDst,
		float32_t *nLoops, float32_t *ngFactor){

	NewtonGauss_t ng;
	float32_t modAdj = 100, modAdjBest = 100;
	uint8_t count = 0;
	arm_matrix_instance_f32 auxA_3_1;
	arm_matrix_instance_f32 auxB_3_1;
	arm_matrix_instance_f32 auxC_3_1;
	arm_matrix_instance_f32 auxA_4_1;
	arm_matrix_instance_f32 auxB_4_1;
	arm_matrix_instance_f32 auxA_4_4;
	arm_matrix_instance_f32 auxB_4_4;
	float32_t auxAData_3_1[3];
	float32_t auxBData_3_1[3];
	float32_t auxCData_3_1[3];
	float32_t auxAData_4_1[4];
	float32_t auxBData_4_1[4];
	float32_t auxAData_4_4[16];
	float32_t auxBData_4_4[16];

	NewtonGaussInit(&ng);
	arm_mat_init_f32(&auxA_3_1, 3, 1, auxAData_3_1);
	arm_mat_init_f32(&auxB_3_1, 3, 1, auxBData_3_1);
	arm_mat_init_f32(&auxC_3_1, 3, 1, auxCData_3_1);
	arm_mat_init_f32(&auxA_4_1, 4, 1, auxAData_4_1);
	arm_mat_init_f32(&auxB_4_1, 4, 1, auxBData_4_1);
	arm_mat_init_f32(&auxA_4_4, 4, 4, auxAData_4_4);
	arm_mat_init_f32(&auxB_4_4, 4, 4, auxBData_4_4);

	/* Initialize with measurements */
	for(uint8_t i=0; i<3; i++){
		ng.accd[i] = accd[i]; // scale acc to the same order of magnitude of magnetometer
		ng.accdPrev[i] = accdPrev[i];
		ng.magd[i] = magd[i];
		ng.magdPrev[i] = magdPrev[i];
	}

	/* Initial guess */
	for(uint8_t i=0; i<4; i++){
		ng.qd[i] = qInit[i];
		ng.qBest[i] = qInit[i];
	}

	/* Interates until error is small enough or 10 loops */
	while(count < 15){
		count++;
		/*------------------------------------------------------------
		 ************* Update Rotation by Matrix quaternion **********
		 *-----------------------------------------------------------*/
		// Clockwise
		ng.RData[0] =    ng.qd[0]*ng.qd[0] - ng.qd[1]*ng.qd[1] - ng.qd[2]*ng.qd[2] + ng.qd[3]*ng.qd[3];
		ng.RData[1] = 2*(ng.qd[0]*ng.qd[1] + ng.qd[2]*ng.qd[3]);
		ng.RData[2] = 2*(ng.qd[0]*ng.qd[2] - ng.qd[1]*ng.qd[3]);

		ng.RData[3] = 2*(ng.qd[0]*ng.qd[1] - ng.qd[2]*ng.qd[3]);
		ng.RData[4] =   -ng.qd[0]*ng.qd[0] + ng.qd[1]*ng.qd[1] - ng.qd[2]*ng.qd[2] + ng.qd[3]*ng.qd[3];
		ng.RData[5] = 2*(ng.qd[1]*ng.qd[2] + ng.qd[0]*ng.qd[3]);

		ng.RData[6] = 2*(ng.qd[0]*ng.qd[2] + ng.qd[1]*ng.qd[3]);
		ng.RData[7] = 2*(ng.qd[1]*ng.qd[2] - ng.qd[0]*ng.qd[3]);
		ng.RData[8] =   -ng.qd[0]*ng.qd[0] - ng.qd[1]*ng.qd[1] + ng.qd[2]*ng.qd[2] + ng.qd[3]*ng.qd[3];


		/*------------------------------------------------------------
		 ****************** Update Error Function ********************
		 *-----------------------------------------------------------*/
		/* Calculates new error:
		 * E = [(accdPrev' - R*accdData') (magPrev' - R*magData')]';
		 * Obs.: R*acc' != acc*R */
		arm_mat_mult_f32(&ng.R, &ng.accPrev, &auxA_3_1);
		arm_mat_sub_f32(&ng.acc, &auxA_3_1, &auxB_3_1);
		//arm_mat_mult_f32(&ng.R, &ng.acc, &auxA_3_1);
		//arm_mat_sub_f32(&ng.accPrev, &auxA_3_1, &auxB_3_1);

		arm_mat_mult_f32(&ng.R, &ng.magPrev, &auxA_3_1);
		arm_mat_sub_f32(&ng.mag, &auxA_3_1, &auxC_3_1);
		//arm_mat_mult_f32(&ng.R, &ng.mag, &auxA_3_1);
		//arm_mat_sub_f32(&ng.magPrev, &auxA_3_1, &auxC_3_1);

		/* Update Error data vector
		 * EData = [Eaccx Eaccy Eaccz Emagx Emagy Emagz]' */
		for(uint8_t i=0; i<3; i++)
			ng.EData[i] = auxBData_3_1[i];
		for(uint8_t i=0; i<3; i++)
			ng.EData[i+3] = auxCData_3_1[i];

		/*------------------------------------------------------------
		 ****************** Update Jacobian Matrix *******************
		 *-----------------------------------------------------------*/
		// Left-handed clockwise from top
		ng.JacobData[0]  = -2*ng.qd[0]*accd[0] - 2*ng.qd[1]*accd[1] - 2*ng.qd[2]*accd[2];
		ng.JacobData[1]  =  2*ng.qd[1]*accd[0] - 2*ng.qd[0]*accd[1] + 2*ng.qd[3]*accd[2];
		ng.JacobData[2]  =  2*ng.qd[2]*accd[0] - 2*ng.qd[3]*accd[1] - 2*ng.qd[0]*accd[2];
		ng.JacobData[3]  = -2*ng.qd[3]*accd[0] - 2*ng.qd[2]*accd[1] + 2*ng.qd[1]*accd[2];

		ng.JacobData[4]  = -2*ng.qd[1]*accd[0] + 2*ng.qd[0]*accd[1] - 2*ng.qd[3]*accd[2];
		ng.JacobData[5]  = -2*ng.qd[0]*accd[0] - 2*ng.qd[1]*accd[1] - 2*ng.qd[2]*accd[2];
		ng.JacobData[6]  =  2*ng.qd[3]*accd[0] + 2*ng.qd[2]*accd[1] - 2*ng.qd[1]*accd[2];
		ng.JacobData[7]  =  2*ng.qd[2]*accd[0] - 2*ng.qd[3]*accd[1] - 2*ng.qd[0]*accd[2];

		ng.JacobData[8]  = -2*ng.qd[2]*accd[0] + 2*ng.qd[3]*accd[1] + 2*ng.qd[0]*accd[2];
		ng.JacobData[9]  = -2*ng.qd[3]*accd[0] - 2*ng.qd[2]*accd[1] + 2*ng.qd[1]*accd[2];
		ng.JacobData[10] = -2*ng.qd[0]*accd[0] - 2*ng.qd[1]*accd[1] - 2*ng.qd[2]*accd[2];
		ng.JacobData[11] = -2*ng.qd[1]*accd[0] + 2*ng.qd[0]*accd[1] - 2*ng.qd[3]*accd[2];

		ng.JacobData[12] = -2*ng.qd[0]*magd[0] - 2*ng.qd[1]*magd[1] - 2*ng.qd[2]*magd[2];
		ng.JacobData[13] =  2*ng.qd[1]*magd[0] - 2*ng.qd[0]*magd[1] + 2*ng.qd[3]*magd[2];
		ng.JacobData[14] =  2*ng.qd[2]*magd[0] - 2*ng.qd[3]*magd[1] - 2*ng.qd[0]*magd[2];
		ng.JacobData[15] = -2*ng.qd[3]*magd[0] - 2*ng.qd[2]*magd[1] + 2*ng.qd[1]*magd[2];

		ng.JacobData[16] = -2*ng.qd[1]*magd[0] + 2*ng.qd[0]*magd[1] - 2*ng.qd[3]*magd[2];
		ng.JacobData[17] = -2*ng.qd[0]*magd[0] - 2*ng.qd[1]*magd[1] - 2*ng.qd[2]*magd[2];
		ng.JacobData[18] =  2*ng.qd[3]*magd[0] + 2*ng.qd[2]*magd[1] - 2*ng.qd[1]*magd[2];
		ng.JacobData[19] =  2*ng.qd[2]*magd[0] - 2*ng.qd[3]*magd[1] - 2*ng.qd[0]*magd[2];

		ng.JacobData[20] = -2*ng.qd[2]*magd[0] + 2*ng.qd[3]*magd[1] + 2*ng.qd[0]*magd[2];
		ng.JacobData[21] = -2*ng.qd[3]*magd[0] - 2*ng.qd[2]*magd[1] + 2*ng.qd[1]*magd[2];
		ng.JacobData[22] = -2*ng.qd[0]*magd[0] - 2*ng.qd[1]*magd[1] - 2*ng.qd[2]*magd[2];
		ng.JacobData[23] = -2*ng.qd[1]*magd[0] + 2*ng.qd[0]*magd[1] - 2*ng.qd[3]*magd[2];

		/*------------------------------------------------------------
		 *************** Calculate next quaternion *******************
		 *-----------------------------------------------------------*/
		/* qNext = qd - (J'*J)\J'*E => inv(J'*J)*(J'*E) */
		arm_mat_trans_f32(&ng.Jacob, &ng.Jacobt);
		arm_mat_mult_f32(&ng.Jacobt, &ng.Jacob, &auxA_4_4);
		arm_mat_inverse_f32(&auxA_4_4, &auxB_4_4);
		arm_mat_mult_f32(&ng.Jacobt, &ng.E, &auxA_4_1);
		arm_mat_mult_f32(&auxB_4_4, &auxA_4_1, &auxB_4_1);
		arm_mat_sub_f32(&ng.q, &auxB_4_1, &auxA_4_1);
		/* Update Adjustment module */
		vectMod(auxBData_4_1, &modAdj, 4);

		/* Normalize and update next quaternion */
		vectMod(auxAData_4_1, &ng.qMod, 4);
		if(ng.qMod > 0)
			arm_mat_scale_f32(&auxA_4_1, 1/ng.qMod, &ng.q);

		/* Update best quaternion according to smaller adjustment */
		if(modAdj < modAdjBest){
			modAdjBest = modAdj;
			*ngFactor = modAdj;
			*nLoops = count;
			for(uint8_t i=0; i<4; i++)
				ng.qBest[i] = ng.qd[i];
			if(modAdj <= 1e-2)
				break;
		}
	}
	for(uint8_t i=0; i<4; i++)
		qDst[i] = ng.qBest[i];
}

void vectMod(float32_t *src, float32_t *dst, uint8_t size){
	dst[0] = 0;
	for(uint8_t i=0; i < size; i++)
		dst[0] += (src[i]*src[i]);
	dst[0] = sqrtf(dst[0]);
}

