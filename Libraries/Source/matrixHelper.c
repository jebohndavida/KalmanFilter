#include "arm_math.h"
#include "matrixHelper.h"

void matrixHelperInit(MatrixHelper_t *m){
	for(uint8_t i=0; i<3; i++){
		m->auxAData_3_1[i] = 0;
		m->auxBData_3_1[i] = 0;
		m->auxBData_3_1[i] = 0;
	}
	for(uint8_t i=0; i<4; i++){
		m->auxAData_4_1[i] = 0;
		m->auxBData_4_1[i] = 0;
	}
	for(uint8_t i=0; i<16; i++){
		m->auxAData_4_4[i] = 0;
		m->auxBData_4_4[i] = 0;
		m->auxCData_4_4[i] = 0;
		m->auxDData_4_4[i] = 0;
	}

	/* Initialize Auxiliary Matrices */
	arm_mat_init_f32(&m->auxA_3_1, 3, 1, m->auxAData_3_1);
	arm_mat_init_f32(&m->auxB_3_1, 3, 1, m->auxBData_3_1);
	arm_mat_init_f32(&m->auxC_3_1, 3, 1, m->auxCData_3_1);
	arm_mat_init_f32(&m->auxA_4_1, 4, 1, m->auxAData_4_1);
	arm_mat_init_f32(&m->auxB_4_1, 4, 1, m->auxBData_4_1);
	arm_mat_init_f32(&m->auxA_4_4, 4, 4, m->auxAData_4_4);
	arm_mat_init_f32(&m->auxB_4_4, 4, 4, m->auxBData_4_4);
	arm_mat_init_f32(&m->auxC_4_4, 4, 4, m->auxCData_4_4);
	arm_mat_init_f32(&m->auxD_4_4, 4, 4, m->auxDData_4_4);
}

void vectMod(arm_matrix_instance_f32 *src, float32_t *dst){
	dst[0] = 0;
	for(uint8_t i=0; i<(src->numCols*src->numRows); i++)
		dst[0] += (src->pData[i]*src->pData[i]);
	dst[0] = sqrtf(dst[0]);
}

void updateMatValue(arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dst){
	for(uint8_t i=0; i<(src->numCols*src->numRows); i++)
		dst->pData[i] = src->pData[i];
}

void matrixSub(arm_matrix_instance_f32 *a,
		arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *c){

	for(uint8_t i = 0; i < a->numRows*a->numCols; i++)
		c->pData[i] = a->pData[i] - b->pData[i];
}

arm_matrix_instance_f32 matrixAdd(arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *b){
	arm_matrix_instance_f32 c;

	c.numRows = a->numRows;
	c.numCols = a->numCols;
	for(uint8_t i = 0; i < a->numRows*a->numCols; i++)
		c.pData[i] = a->pData[i] + b->pData[i];
	return c;
}

