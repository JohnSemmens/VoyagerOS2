// HAL_GPS.h

#ifndef _HAL_GPS_h
#define _HAL_GPS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "HAL.h"
#include "location.h"



//
//const unsigned char GPS_Init_String0[] = { 0x00 };

// CFG - TP set pulse rate to 5Hz
const unsigned char GPS_CFG_TP_200ms[] = { 0xB5, 0x62, 0x06, 0x07, 0x14, 0x00, 0x40, 0x0D, 0x03, 0x00, 0x50, 0xC3, 0x00, 0x00, 0x01, 0x01
										  ,0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0x2A 
										  ,0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x07, 0x15, 0x3E };

// CFG - TP set pulse rate to 1000ms period 10ms on
const unsigned char GPS_CFG_TP_1000_10ms[] = { 0xB5, 0x62, 0x06, 0x07, 0x14, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x10, 0x27, 0x00, 0x00, 0x01, 0x01
										  ,0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1D, 0xCD
										  ,0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x07, 0x15, 0x3E };

// CFG - TP set pulse rate to 2000ms period 10ms on
const unsigned char GPS_CFG_TP_2000_10ms[] = { 0xB5, 0x62, 0x06, 0x07, 0x14, 0x00, 0x80, 0x84, 0x1E, 0x00, 0x10, 0x27, 0x00, 0x00, 0x01, 0x01
										  ,0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAE, 0xC1
										  ,0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x07, 0x15, 0x3E };

// CFG -RXM Power Saving Mode
const unsigned char GPS_CFG_RXM_PSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92
										  ,0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x11, 0x1F, 0x48 };


// CFG -RXM Power Saving Mode - On/Off 100s
const unsigned char GPS_CFG_RXM_PSM_OO_100s[] = { 0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x0E, 0x90, 0x00, 0x00, 0xA0, 0x86
												, 0x01, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01
												, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40
												, 0x01, 0x00, 0xCD, 0x33 };

const unsigned char GPS_CFG_RXM_PSM_OO_100s2[] = { 0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x0E, 0x90, 0x00, 0x00, 0xA0, 0x86
												, 0x01, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01
												, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40
												, 0x01, 0x00, 0xCD, 0x33 
												, 0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x3B, 0x49, 0x72 };
						

class HALGPS
{
public:
	// initialise the software serial port for the GPS.
	void Init(void);

	// Read the GPS and place the result into the Global variable: Currentloc
	void Read(void);

	// check if the GPS location is valid (or if the simulated location is valid)
	bool GPS_LocationIs_Valid(Location TestLoc);

	//bool GPS_TimeIs_Valid();

	//void GPS_Read_Time(void);
//	void UpdateLocalTimeFromGPSTime(byte timezone_offset);

	//byte DaysInMonth(byte month, byte year);

	EquipmentStatusType EquipmentStatus;

	void SendConfigurationString(char* GPS_Init_String);
	void SendConfigurationString(int MsgNumber);
};


#endif

