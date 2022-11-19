// HAL_WingAngle.h

#ifndef _HAL_WingAngle_h
#define _HAL_WingAngle_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "HAL.h"
#include "WindAngle_MPU9250_t3.h"

class HALWingAngle
{
	public:
		int Angle;
		EquipmentStatusType PortStatus;
		EquipmentStatusType StbdStatus;

		float PortTemperature;
		float StbdTemperature;

		WindAngle_9250 WingSailAngleSensorPort;
		WindAngle_9250 WingSailAngleSensorStbd;

		void Read(void);
		void Init(void);

		int Deviation;

		int DeviationCalc(int MagneticAngle);
};

#endif

