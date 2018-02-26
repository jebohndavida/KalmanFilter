#include "angular.h"
#include "arm_math.h"
#include "main.h"
#include <math.h>

void Quat2Eul(Quat_t *q, Euler_t *e){
	double_t test, sqx, sqy, sqz;

	test = q->x*q->y + q->z*q->w;
	if (test > 0.499) { // singularity at north pole
		e->roll = 2 * atan2(q->x,q->w);
		e->pitch = PI/2;
		e->yaw = 0;
		return;
	}
	if (test < -0.499) { // singularity at south pole
		e->roll = -2 * atan2(q->x,q->w);
		e->pitch = - PI/2;
		e->yaw = 0;
		return;
	}

    sqx = q->x*q->x;
    sqy = q->y*q->y;
    sqz = q->z*q->z;

    e->roll = atan2(2*q->y*q->w-2*q->x*q->z , 1 - 2*sqy - 2*sqz);
    e->pitch = asin(2*test);
    e->yaw = atan2(2*q->x*q->w-2*q->y*q->z , 1 - 2*sqx - 2*sqz);
}

void setQuat(Quat_t *q, float32_t *qVect){
	q->x = qVect[0];
	q->y = qVect[1];
	q->z = qVect[2];
	q->w = qVect[3];
}




