// HAL_Time.h

#ifndef _HAL_TIME_h
#define _HAL_TIME_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//struct Time {
//	byte second;
//	byte minute;
//	byte hour;
//	byte dayOfWeek;
//	byte dayOfMonth;
//	byte month;
//	int year;
//	int raw;
//};

void time_init(void);

void sync_RTC_to_GPS(void);

//void time_update(void);

time_t getTeensy3Time(void);

#endif

