// 
// Hardware Abstraction Layer for GPS
// to manage Location and Time
// V1.0 16/6/2021
// V1.1 14/11/2021 added Simulated GPS

#include "HAL_GPS.h"

#include "Navigation.h"
#include "location.h"
#include "configValues.h"
#include "TinyGPS++.h"
#include "DisplayStrings.h"
#include "sim_vessel.h"
#include "HAL_Time.h"
#include "TimeLib.h"

extern HALGPS gps;
extern NavigationDataType NavData;
extern HardwareSerial* Serials[];
extern configValuesType Configuration;

extern bool UseSimulatedVessel;			// flag to disable the GPS and indicate that the current location is simulated 
extern sim_vessel simulated_vessel;

extern time_t GPSTime;
//extern Time GPSUTCTime;				// Current Time from GPS. We'll call it UTC, even though that's not strictly the same.
//extern Time GPSLocalTime;				// Time from GPS converted to local time by applying timezone offset.

long GPS_Last_Loc;						// milliseconds since last GPS valid location message
long GPS_Last_Message;					// milliseconds since last GPS message

// The TinyGPS++ object
TinyGPSPlus t_gps;

void HALGPS::Init() {
	// Initialise the GPS 
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 21/10/2018 updated for change from serial to I2C 
	// V1.3 6/4/2019 added delay at the end, because it seems solve a lock up problem.
	// V1.4 13/2/2022 added receive memory buffer as per pjrc.com tiny gps blog article

	(*Serials[Configuration.GPSPort]).begin(9600);

	// setup buffer as 100 bytes
	static char SerialReadBuffer[100];
	(*Serials[Configuration.GPSPort]).addMemoryForRead(SerialReadBuffer, 100);

	Serial.println(F("*** Initialising GPS.."));
	EquipmentStatus = EquipmentStatusType::Unknown;

	// listen for any serial data from the GPS for up to one second.

	char GPS_Msg[50];
	unsigned int MsgIndex = 0;
	unsigned long StartMillis = millis();
	while ( ((millis()- StartMillis) < 1000)  && (MsgIndex < sizeof(GPS_Msg)- 2) )
	{
		if ((*Serials[Configuration.GPSPort]).available())
		{
			char received = (*Serials[Configuration.GPSPort]).read();
			GPS_Msg[MsgIndex++] = received;
		}
	}
	GPS_Msg[MsgIndex] = '\0'; 
	Serial.print(F("GPS Response: "));
	Serial.println(GPS_Msg);

	if (MsgIndex > 0)
		EquipmentStatus = EquipmentStatusType::Found;
	else
		EquipmentStatus = EquipmentStatusType::NotFound;


	Serial.print(F("GPS status: "));
	Serial.println(GetEquipmentStatusString(EquipmentStatus));

	// GPS_Read_Time();
	Read();
	//UpdateLocalTimeFromGPSTime(Configuration.timezone_offset);

	//delay(300); // it seems to be necessary to delay a bit before doing anything else or a lock up occurs.
				// not sure what's happening, but this seems to fix it.
				// was 100ms. changed to 300ms because of some occasional start up lockups to see if it helps.

	if (Configuration.UseGPSInitString)
	{ 
		Serial.println(F("GPS sending init string. "));
		(*Serials[Configuration.GPSPort]).write(GPS_CFG_TP_1000_10ms, sizeof(GPS_CFG_TP_1000_10ms));
		delay(100);
		(*Serials[Configuration.GPSPort]).write(GPS_CFG_RXM_PSM, sizeof(GPS_CFG_RXM_PSM)); 
	}
	else
	{
		Serial.println(F("GPS NOT sending init string. "));
	}

	Serial.println(F("*** Initialising GPS complete."));
	Serial.println();
};

void HALGPS::Read() {
	// V1.1 22/10/2016 updated to support parameterised serial port
	// V1.2 14/4/2018 added GPS directly provided course and speed to Navdata object
	// V1.3 21/10/2018 updated for change from serial to I2C 

	static long prev_GPS_Last_Message;
	static long prev_GPS_Last_Loc;

	while ((*Serials[Configuration.GPSPort]).available()) //available() returns the number of new bytes available from the GPS module
	{
		t_gps.encode((*Serials[Configuration.GPSPort]).read()); //Feed the GPS parser
	}

	// if not using a simulated GPS position (i.e. if real) then populate the NavData
	if (!UseSimulatedVessel)
	{
		NavData.Currentloc.lat = t_gps.location.lat() * 10000000UL;
		NavData.Currentloc.lng = t_gps.location.lng() * 10000000UL;
		NavData.CurrentLocTimeStamp = millis();

		// get course and speed directly from GPS
		NavData.COG = t_gps.course.deg();

		NavData.SOG_knt = (float)t_gps.speed.knots();
		NavData.SOG_mps = (float)t_gps.speed.mps();
	}
	else // populate with simulated data
	{
		NavData.Currentloc.lat = simulated_vessel.Currentloc.lat;
		NavData.Currentloc.lng = simulated_vessel.Currentloc.lng;
		NavData.CurrentLocTimeStamp = millis();

		NavData.COG = simulated_vessel.Heading;
		NavData.SOG_mps = simulated_vessel.SOG_mps;
		NavData.SOG_knt = simulated_vessel.SOG_mps * 1.94384449; // knot/mps;
	}

	// get the date and time from GPS into GPSTime object.
	TimeElements tm;
	tm.Year = t_gps.date.year() - 1970;
	tm.Month = t_gps.date.month();
	tm.Day = t_gps.date.day();
	tm.Hour = t_gps.time.hour();
	tm.Minute = t_gps.time.minute();
	tm.Second = t_gps.time.second();
	GPSTime = makeTime(tm);
	GPSTime += (Configuration.timezone_offset * SECS_PER_HOUR); // apply timezone offset

	// update gps message timing statistics
	GPS_Last_Message = millis() - prev_GPS_Last_Message;
	prev_GPS_Last_Message = millis();

	// update gps valid location timing statistics
	if (NavData.Currentloc.lat != 0 && NavData.Currentloc.lng != 0) {
		GPS_Last_Loc = millis() - prev_GPS_Last_Loc;
		prev_GPS_Last_Loc = millis();
	}

};

//
//void HALGPS::GPS_Read_Time() {
//	// V1.0 21/10/2018 Read the Date and Time from the GPS
//	// V1.1 30/3/2019 updated to return a reasonably formatted date, even if the GPS Time is not available.
//	//				This is to make the system more resilient. 
//
//	while ((*Serials[Configuration.GPSPort]).available()) //available() returns the number of new bytes available from the GPS module
//	{
//		t_gps.encode((*Serials[Configuration.GPSPort]).read()); //Feed the GPS parser
//	}
//
//	// get the date and time from GP into CurrentTime object.
//	GPSUTCTime.year = t_gps.date.year();
//	GPSUTCTime.month = t_gps.date.month();
//	GPSUTCTime.dayOfMonth = t_gps.date.day();
//
//	GPSUTCTime.hour = t_gps.time.hour();
//	GPSUTCTime.minute = t_gps.time.minute();
//	GPSUTCTime.second = t_gps.time.second();
//
//	//if (GPSUTCTime.year == 0)
//	//{
//	//	GPSUTCTime.year = 9999; // the year
//	//	GPSUTCTime.month = 9;
//	//	GPSUTCTime.dayOfMonth = 9;
//	//	GPSUTCTime.hour = 99;
//	//	GPSUTCTime.minute = 99;
//	//}
//};
//
//void HALGPS::UpdateLocalTimeFromGPSTime(byte timezone_offset)
//{
//	// update the CurrentLocalTime from the CurrentUTCTime provided from the GPS sentences.
//	// use the offset passed in as a parameter.
//
//	GPSLocalTime.second = GPSUTCTime.second;
//	GPSLocalTime.minute = GPSUTCTime.minute;
//
//	GPSLocalTime.hour = GPSUTCTime.hour + timezone_offset;
//
//	// if we've moved into next day, then increment day, and deduct 24 hours from hour. 
//	if (GPSLocalTime.hour >= 24)
//	{
//		GPSLocalTime.dayOfMonth = GPSUTCTime.dayOfMonth + 1;
//		GPSLocalTime.hour = GPSLocalTime.hour - 24;
//	}
//	else
//	{
//		GPSLocalTime.dayOfMonth = GPSUTCTime.dayOfMonth;
//	}
//
//	// if we've rolled past end of month, then increment month and reset day number
//	if (GPSLocalTime.dayOfMonth > DaysInMonth(GPSLocalTime.month, GPSLocalTime.year))
//	{
//		GPSLocalTime.month = GPSUTCTime.month + 1;
//		GPSLocalTime.dayOfMonth = 1;
//	}
//	else
//	{
//		GPSLocalTime.month = GPSUTCTime.month;
//	}
//
//	// if we've rolled into month 13 then increment year and reset month.
//	if (GPSLocalTime.month >= 13)
//	{
//		GPSLocalTime.year = GPSUTCTime.year + 1;
//		GPSLocalTime.month = 1;
//	}
//	else
//	{
//		GPSLocalTime.year = GPSUTCTime.year;
//	}
//}

//
//byte HALGPS::DaysInMonth(byte month, byte year)
//{
//	// return the number of days on a given month.
//	// this is used in converting from between time zones.
//	// The test for leap years has bee simplified, because the next issue other than usual 4 year test, is in the year 2100.
//
//	// https://www.daniweb.com/programming/software-development/threads/265791/calculating-days-in-a-month
//
//	int Days;
//	if (month == 4 || month == 6 || month == 9 || month == 11)
//		Days = 30;
//
//	else if (month == 02)
//	{
//		//bool isLeapYear = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
//		// use a simpler/faster leap year check. It will be 80 years before it matters.
//		bool isLeapYear = (year % 4 == 0);
//
//		if (isLeapYear == 0)
//			Days = 29;
//		else
//			Days = 28;
//	}
//	else
//		Days = 31;
//
//	return Days;
//}

bool HALGPS::GPS_LocationIs_Valid(Location TestLoc) {
	// check if the GPS location is valid (or if the simulated location is valid)
	// V1.1 21/10/2018 Updated to allow GPS_Last_Loc time in ms of zero to considered ok (i.e ">=" vs ">")

	bool valid = false;

	if (UseSimulatedVessel)
	{
		// if simulated, just check for non-zero lat/lon 
		valid = (TestLoc.lat != 0 && TestLoc.lng != 0);
	}
	else
	{
		// if real GPS, check for non=zero lat/lon and valid location provided in last 10 seconds
		valid = (TestLoc.lat != 0 && TestLoc.lng != 0 && GPS_Last_Loc >= 0 && GPS_Last_Loc < 10000);
	}

	return valid;
};
//
//bool HALGPS::GPS_TimeIs_Valid() {
//	// check if the GPS time is valid 
//	// V1.1 19/05/2020 John Semmens
//
//	return (CurrentUTCTime.dayOfMonth > 0 && CurrentUTCTime.hour <= 24) ;
//};

void HALGPS::SendConfigurationString(char *GPS_Init_String)
{
	(*Serials[Configuration.GPSPort]).print(GPS_Init_String);
}

void HALGPS::SendConfigurationString(int MsgNumber)
{
	switch(MsgNumber)
	{
	case 0: 
		break;

	case 1:
		(*Serials[Configuration.GPSPort]).write(GPS_CFG_TP_1000_10ms, sizeof(GPS_CFG_TP_1000_10ms));
		break;

	case 2:
		(*Serials[Configuration.GPSPort]).write(GPS_CFG_TP_2000_10ms, sizeof(GPS_CFG_TP_2000_10ms));
		break;

	case 3:
		(*Serials[Configuration.GPSPort]).write(GPS_CFG_TP_200ms, sizeof(GPS_CFG_TP_200ms));
		break;

	case 4:
		(*Serials[Configuration.GPSPort]).write(GPS_CFG_RXM_PSM, sizeof(GPS_CFG_RXM_PSM));
		break;

	case 5:
		(*Serials[Configuration.GPSPort]).write(GPS_CFG_RXM_PSM, sizeof(GPS_CFG_RXM_PSM));
		(*Serials[Configuration.GPSPort]).write(GPS_CFG_RXM_PSM_OO_100s, sizeof(GPS_CFG_RXM_PSM_OO_100s));
		break;

	case 6:
		(*Serials[Configuration.GPSPort]).write(GPS_CFG_RXM_PSM_OO_100s2, sizeof(GPS_CFG_RXM_PSM_OO_100s2));
		break;
	default:;
	}
}