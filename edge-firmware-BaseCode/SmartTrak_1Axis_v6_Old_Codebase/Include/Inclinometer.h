// *************************************************************************************************
//									I n c l i n o m e t e r . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Inclinometer Function declarations
//
// *************************************************************************************************
#ifndef INCLINOMETER_H
	#define INCLINOMETER_H
#endif

#ifndef _MMA845X_H_
	#error mma845x.h must be #included first
#endif

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

/***********************************************************************************************
**
**  Variable type definition: ACCELEROMETER_DATA
*/
typedef struct
{
  UINT8 x_msb;
  UINT8 x_lsb;
  UINT8 y_msb;
  UINT8 y_lsb;
  UINT8 z_msb;
  UINT8 z_lsb;

} XYZ_DATA, *PTR_XYZ_DATA;

#define ACCELEROMETER_SAMPLE_LEN	6

/***********************************************************************************************
**
**  Variable type definition: ACCELEROMETER_SAMPLE
 * This union allows us to read in 6 bytes, the complete accelerometer sample,
 * and then treat them as named variables, rather than an indexed array
*/
typedef union
{
  UINT8 RAW_Data[ACCELEROMETER_SAMPLE_LEN];
  struct
  {
    XYZ_DATA XYZ;
  } XYZ_Sample;

} ACCELEROMETER_SAMPLE, *PTR_ACCELEROMETER_SAMPLE;

typedef struct
{
	ACCELEROMETER_SAMPLE Acc_Sample;
	
	// INT16 values
	INT16 nAcc_X_Value;
	INT16 nAcc_Y_Value;
	INT16 nAcc_Z_Value;
	
	float fAcc_X_Value;
	float fAcc_Y_Value;
	float fAcc_Z_Value;

	float fX_Angle;
	float fY_Angle;
	float fZ_Angle;
	
} INCLINOMETER_SAMPLE, *PTR_INCLINOMETER_SAMPLE;


// structure for averaging 3D angles
typedef struct
{
	float fX_Angle;
	float fY_Angle;

} ANGLE_AVERAGING_SAMPLE, *PTR_ANGLE_AVERAGING_SAMPLE;

#define	INCLINOMETER_AVERAGING_COUNT		3

//#define	PI	3.14159
#define	PI	M_PI

#define	RADIANS_TO_DEGREES(radians)			((radians * 180.0) / PI)

#define MAX_I2C_ERROR_CNT			10		// maximum allowable I2C Inclinometer read errors before shutting down motion
#define MAX_I2C_HrdStal_CNT			100             // 2mins to hard stall
//-------------------------------------------------------------------------------------------------------
// Function Declarations
//-------------------------------------------------------------------------------------------------------

BOOL ReadInclinometerSample(PTR_INCLINOMETER_SAMPLE Inclination);
BOOL FormatInclination(char *ptrOutputStr, PTR_INCLINOMETER_SAMPLE Inclination);
BOOL AverageInclinometerSample(PTR_INCLINOMETER_SAMPLE Inclination);
BOOL Init_InclinometerSampleAveraging(void);
BOOL FormatAverageInclination(char *ptrOutputStr);


//-------------------------------------------------------------------------------------------------------
// Global Variables
//-------------------------------------------------------------------------------------------------------

#ifdef DEFINE_GLOBALS
	GLOBAL INCLINOMETER_SAMPLE pgInclination;												// structure for reading and processing accelerometer
	GLOBAL ANGLE_AVERAGING_SAMPLE pgAngleAveragingSamples[INCLINOMETER_AVERAGING_COUNT];	// array of inclinometer samples for averaging
	GLOBAL ANGLE_AVERAGING_SAMPLE pgAngleAverage;											// current rolling average
        GLOBAL BOOL MAN_EAST=0;
        GLOBAL BOOL MAN_WEST=0;
        GLOBAL BOOL MAN_STOW=0;
#else
	GLOBAL INCLINOMETER_SAMPLE pgInclination;
	GLOBAL ANGLE_AVERAGING_SAMPLE pgAngleAveragingSamples[];
	GLOBAL ANGLE_AVERAGING_SAMPLE pgAngleAverage;
        GLOBAL BOOL MAN_EAST;
        GLOBAL BOOL MAN_WEST;
        GLOBAL BOOL MAN_STOW;
#endif


// end of Inclinometer.h
