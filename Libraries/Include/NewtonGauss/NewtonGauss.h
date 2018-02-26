#include "arm_math.h"

#ifndef _NewtonGauss_H_
#define _NewtonGauss_H_

typedef struct NewtonGaussStruct{
		/* NewtonGauss Matrices */
		arm_matrix_instance_f32 Jacob;
		arm_matrix_instance_f32 Jacobt;
		arm_matrix_instance_f32 R;
		arm_matrix_instance_f32 E;
		arm_matrix_instance_f32 q;
		arm_matrix_instance_f32 acc;
		arm_matrix_instance_f32 accPrev;
		arm_matrix_instance_f32 mag;
		arm_matrix_instance_f32 magPrev;

		/* NewtonGauss data vectors */
		float32_t EData[6];
		float32_t qd[4];
		float32_t qBest[4];
		float32_t qMod;
		float32_t JacobData[24];
		float32_t JacobtData[24];
		float32_t RData[9];
		float32_t accd[3];
		float32_t accdPrev[3];
		float32_t magd[3];
		float32_t magdPrev[3];

} NewtonGauss_t;

void NewtonGaussInit(NewtonGauss_t*);
void ApplyNewtonGauss(float32_t*, float32_t*, float32_t*, float32_t*, float32_t*,
		float32_t*, float32_t*, float32_t*);
void vectMod(float32_t*, float32_t*, uint8_t);

#endif


