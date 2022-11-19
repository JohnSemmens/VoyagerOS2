// configValues.h

#ifndef _CONFIGVALUES_h
#define _CONFIGVALUES_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Navigation.h"
#include "CommandState_Processor.h"

static const int EEPROM_Storage_Version_Const = 02;   // change this number to force config to be cleared and revert to default.

struct configValuesType {
	byte EEPROM_Storage_Version = EEPROM_Storage_Version_Const; // stored object version. this is to test if the data being retrieve is valid with reference to this version. 

	char DisplayScreenView; // character code representing the current Local Display Screen
	char SDCardLogDelimiter; // Delimiter character for the SD Card Log files.
	bool SaveStateValues;  // set whether the current state of the vessel should be saved. set to False for testing. Set to True for real mission.

	byte timezone_offset; // offset to our timezone from UTC/GPS time +10 hours for Eastern Standard Time, and +11 for Eastern Dalylight Saving Time.

	// sailing limits
	ManoeuvreType TackingMethod; // this describes the method tacking. i.e. tack or gybe
	byte MinimumAngleUpWind;	// minumum angle off the wind when close hauled. Typically about 45 degrees. Zero means its posible to sail directly upwind.
	byte MinimumAngleDownWind;	// minimum angle off dead down wind. Force the vessel to tack down wind. Zero means no constraint.
	int WPCourseHoldRadius;		// metres - Waypoint Course Hold Radius - don't change course inside this circle.

	unsigned int RTHTimeManualControl;  // Time under Manual Control (Steer Wind or Steer Compass) to Return To Home (RTH) 
								// This is a filesafe to cause the vessel to Return To Home if there is no further communications (i.e. loss of signal)

	int DefaultMaxCTE; // Max CTE for cases where it is not explicitly set. E.g. Return to Home

	//int MinimumWindChangeTime;		 // sec - min time to elaspse before changing sailing state due to an apparent wind change.

	int TWD_Offset;   // degrees -  offset from AWA to TWD. about 30 degrees. This is a hack to guess TWD. 

	float TargetHeadingFilterConstant; // Filter constant the Target Heading for the Helm versus the CTS value.

	byte SailableAngleMargin; //degrees. this to inhibit tacking when the angle is marginal. i.e. ensure angle is not marginal.

	// Calibration values
	int WindAngleCalibrationOffset; // Calibration Offset for the Wind Angle Indicator

	// Compass rotation offset configuration
	int CompassOffsetAngle; // Compass rotation offset configuration e.g. 0 or 180 degrees

	float MagnetVariation;			// around 12 degrees for Port Philip. Positive values are East. 

	// Compass USFS Max
	// Cardinal corrections
	int CompassError000;
	int CompassError090;
	int CompassError180;
	int CompassError270;

	int MaxFileSize; // Maximum file size for log files on the SD Card. // kbytes 1024 bytes.

	//// RC Fail safe values for each channel. Microseconds typically 1000 to 2000 microseconds. 
	//// Use a value of zero for no setting.
	//uint16_t RC_FailSafe_Ch0; 
	//uint16_t RC_FailSafe_Ch1;
	//uint16_t RC_FailSafe_Ch2;
	//uint16_t RC_FailSafe_Ch3;

	//uint16_t RC_FailSafe_Time; // time before failsafe setting are actioned. Seconds. typically 10s.

	//int RC_IN_Channel_Command; // channel for RC Command; 0
	//int RC_IN_Channel_Steering; // steering input channel  1
	//int RC_IN_Channel_Sail; // sail control channel 2
	//int RC_IN_Channel_Motor; // Motor control channel 3

	int Servo_Channel_Steering; // channel steering and port steering channel in the case of dual rudder servos is true 
	int Servo_Channel_Steering_Stbd; //  starboard sterering channel in the case of dual rudder servos is true 
	int Servo_Channel_Sail; // sail control channel 2
	int Servo_Channel_Motor; // Motor control channel 3

	bool UseGPSInitString; // send Intitialisation Strings to GPS.

	bool DualRudder; // False: means normal single rudder system. True: means Dual Rudder system with auto cut-off.
	bool UseMotor;	 // True: means that a drive motor has been installed and is available to use.

	int SatCommsPort; // Serial Port for Satellite Communications - terse 
	int GPSPort;
	int LoRaPort;  // Serial Port for CLI - verbose

	uint32_t LoRaPortBaudRate;	//  9600 Baud
	uint32_t USBPortBaudRate;		// Baud Rate
	uint32_t SatCommsPortBaudRate;  // Baud Rate

	// Steering PID
	double pidKp, pidKi, pidKd;			// steering PID Constants
	double pidOutputmin, pidOutputmax;   // steering PID output limits
	int pidDirection;
	double pidCentre; // set this to be the steering servo neutral position
	float SteeringFilterConstant;

	long LoiterRadius; // maximum loiter radius when using the "Loiter here" Vessel Command (not the Mission Command)
	
	int TrimTabScale; // Scale and Offset for mapping trim tab angle to the Servo input signal in microseconds
	int	TrimTabOffset; 

	int TrimTabDefaultAngle;

	// Wingsail Magnetic Angle Sensor MPU9250 
	// Cardinal corrections
	int WingAngleError000 =  +2;
	int WingAngleError090 =  -9;
	int WingAngleError180 =  -4;
	int WingAngleError270 =   0;
	// Scale factors
	int WingAngle_mXScale;
	int WingAngle_mYScale;

	int CTE_CorrectionGain; // this the gain of the CTE steering correction adjustment


};

/* Storage Map for EEPROM
	
	1. VesselUsageCounters				(Address 0)    size  32
	2. Configuration Values Structure   (Address 32)   Size 288
	3. Mission Array				    (address 320)  Size 968
	4. Vessel State Values Structure    (address 1288) Size  36
			Total:	   							           1324 bytes
*/

void Save_EEPROM_VesselUsage(void);
void Load_EEPROM_VesselUsage(void);

void Load_EEPROM_ConfigValues(void);
void Save_EEPROM_ConfigValues(void);
void Save_EEPROM_ConfigValues_LeaveVersion(void);

void load_EEPROM_Mission(void);
void Save_EEPROM_Mission(void);

void Load_EEPROM_StateValues(void);
void Save_EEPROM_StateValues(void);

// return true or false to indicate if the current stored data structures have a version consistent with the current software
bool EEPROM_Storage_Version_Valid(void);

void Load_Config_default_values(void);
void Load_ConfigValues(void);

void ReadHWConfigNumber(void);

#endif

