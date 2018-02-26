#include "arm_math.h"
#include "main.h"

#ifndef _ANGULAR_H_
#define _ANGULAR_H_


typedef struct QuatStruct{
	float32_t vect[4];
	float32_t x;
	float32_t y;
	float32_t z;
	float32_t w; //real part
}Quat_t;

typedef struct EulerStruct{
	double_t roll;
	double_t pitch;
	double_t yaw;
}Euler_t;

void Quat2Eul(Quat_t*, Euler_t*);
void setQuat(Quat_t*, float32_t*);

#endif
