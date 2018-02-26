# KalmanFilter
Embedded Kalman Filter algorithm developed in C for ARM Cortex STM32F407, utilizing quaternions translated to Yaw, Pitch and Roll as output.
Sensors used: MPU9250 (accel, gyro e mag).
Newton-Gauss algorithm to calculate the equivalent quaterion from accel and mag measurements.
Usart library with float data transmition.
Math library (matrices operations) from CMSIS DSP.
IDE: CoIDE.
