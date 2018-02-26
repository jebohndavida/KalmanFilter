#include "arm_math.h"

#ifndef _matrixHelper_H_
#define _matrixHelper_H_

typedef struct MatrixHelperStruct{
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

	arm_matrix_instance_f32 auxA_3_1;
	arm_matrix_instance_f32 auxB_3_1;
	arm_matrix_instance_f32 auxC_3_1;
	arm_matrix_instance_f32 auxA_4_1;
	arm_matrix_instance_f32 auxB_4_1;
	arm_matrix_instance_f32 auxA_4_4;
	arm_matrix_instance_f32 auxB_4_4;
	arm_matrix_instance_f32 auxC_4_4;
	arm_matrix_instance_f32 auxD_4_4;

	float32_t auxAData_3_1[3];
	float32_t auxBData_3_1[3];
	float32_t auxCData_3_1[3];
	float32_t auxAData_4_1[4];
	float32_t auxBData_4_1[4];
	float32_t auxAData_4_4[16];
	float32_t auxBData_4_4[16];
	float32_t auxCData_4_4[16];
	float32_t auxDData_4_4[16];

} MatrixHelper_t;


void matrixHelperInit(MatrixHelper_t*);
void vectMod(arm_matrix_instance_f32*, float32_t*);
void updateMatValue(arm_matrix_instance_f32*, arm_matrix_instance_f32*);
void matrixSub(arm_matrix_instance_f32*, arm_matrix_instance_f32*, arm_matrix_instance_f32*);
arm_matrix_instance_f32 matrixAdd(arm_matrix_instance_f32*, arm_matrix_instance_f32*);
void matixMult(arm_matrix_instance_f32*, arm_matrix_instance_f32*, arm_matrix_instance_f32*);


#endif
