// HAL_Servo.h

#ifndef _HAL_Servo_h
#define _HAL_Servo_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define RC_PulseWidth_Min 1000
#define RC_PulseWidth_Max 2000

#define Servo_Ch0_OUT_PIN	30
#define Servo_Ch1_OUT_PIN	29

#define Servo_Ch0_PWR_PIN	11
#define Servo_Ch1_PWT_PIN	12


void Servos_Init(void);

void Servo_Out(int Channel, int PulseWidth);


#endif

