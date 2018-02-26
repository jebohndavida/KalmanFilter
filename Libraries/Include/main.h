#include "arm_math.h"

#ifndef _MAIN_H_
#define _MAIN_H_

#define z 2
#define y 1
#define x 0

volatile float32_t TimingDelay;
volatile float32_t msTicks;

void Delay();
void Quat2YPR(float32_t *, float32_t *, float32_t *);
void qRotate3D(float32_t *, float32_t *, float32_t *);
void qRotate4D(float32_t *, float32_t *);

#endif
