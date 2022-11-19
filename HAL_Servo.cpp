// 
// 
// 

#include "HAL_Servo.h"
#include "HAL.h"
#include <Servo.h>

Servo Servo_Channel[2];

void Servo_Out(int Channel, int PulseWidth)
{
	// output a value to a servo.
	// V1.0 13/10/2016 John Semmens

	// check that the request is within resonable bounds before writing to the servo.
	if ((RC_PulseWidth_Min <= PulseWidth) && (PulseWidth <= RC_PulseWidth_Max)) {
				Servo_Channel[Channel].writeMicroseconds(PulseWidth);
	}
};

void Servos_Init(void)
{
	// initalise the array of Servo objects by attaching to pins.
	Servo_Channel[0].attach(Servo_Ch0_OUT_PIN);
	Servo_Channel[1].attach(Servo_Ch1_OUT_PIN);
	
	pinMode(Servo_Ch0_PWR_PIN, OUTPUT);
	digitalWrite(Servo_Ch0_PWR_PIN, LOW);

	// set each channel to the failsafe value
	Servo_Channel[0].write(1500);
	Servo_Channel[1].write(1500);
}