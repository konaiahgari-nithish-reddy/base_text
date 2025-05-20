// *************************************************************************************************
//									I n c l i n o m e t e r . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Inclinometer Functions
//
// *************************************************************************************************

#define INCLINOMETER_C

//-----------------------------------------------------------------------------
//								#include files
//-----------------------------------------------------------------------------

#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>				// Microchip PIC32 peripheral library main header
//lint +e14

#include <string.h>				// Microchip string functions
//#include <ctype.h>				// tolower()

#include <math.h>				// trig functions

#include "gsfstd.h"				// gsf standard #defines

#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "EventFlags.h"			// event flag definitions and globals

#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
#include "MotionStats.h"		// motion statistics for reporting
#include "MoveSequenceFSM.h"	// move sequence FSM

#include "AppTimer.h"			// timer errors

#include "TimeDelay.h"			// delay in increments of 10uS

#include "StrConversions.h"		// ASCII string <==> numeric conversions
#include "ftoa.h"
#include "CoordTranslate.h"

#ifdef USE_MMA8452Q_INCLINOMETER
	#include "mma845x.h"              // MMA845xQ definitions
	#include "Inclinometer.h"
#endif

#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif

enum tagInclinometerErrors
{
	INCLINOMETER_ERROR_NONE = INCLINOMETER_ERROR_BASE,
	INCLINOMETER_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	INCLINOMETER_ERROR_UNEXPECTED_EVENT,		// 2 unexpected event
	INCLINOMETER_ERROR_INVALID_STATE,			// 3 not a valid state
	INCLINOMETER_ERROR_INVALID_SUBSTATE,		// 4 not a valid state
	INCLINOMETER_ERROR_ACCESS_FAILURE,			// 5 I2C or other access error
	INCLINOMETER_ERROR_DIVIDE_BY_ZERO,			// 6 divide by zero detected
	INCLINOMETER_ERROR_UNEXPECTED_DELTA,		// 7 unexpected change from previous value
        INCLINOMETER_ERROR_READ_FAIL,                   //8 reading of inclinometer fail
	INCLINOMETER_ERROR_UNPROCESSED_EVENT = INCLINOMETER_ERROR_BASE + 0x0F

};


#ifdef NOTDEF
      switch (functional_block)
      {
        /////////////////////////////////////////////////////////////////////////////////////////
        case FBID_FULL_XYZ_SAMPLE:
          /*
          **  FULL XYZ Sample Registers 0x01-0x06 (MMA8451Q=14 bit, MMA8452Q= 12 bit, MMA8453Q= 10 bit
          **
          **  Poll ZYXDR bit in Status Register
          */

          RegisterFlag.Byte = IIC_RegRead(SlaveAddressIIC, STATUS_00_REG);
          if (RegisterFlag.ZYXDR_BIT == 1)
          {
            /*
            **  Read the  XYZ sample data
            */
            if (deviceID>3)

            {
              IIC_RegReadN(SlaveAddressIIC, OUT_X_MSB_REG, 3, &value[0]);



            }
            else
            {
              IIC_RegReadN(SlaveAddressIIC, OUT_X_MSB_REG, 6, &value[0]);
            }
            /*
            **  Output results
            */
            OutputTerminal (FBID_FULL_XYZ_SAMPLE, &value[0]);
          }
          break;



void loop()
{
  static byte source;

  /* If int1 goes high, all data registers have new data */
  if (digitalRead(int1Pin))  // Interrupt pin, should probably attach to interrupt function
  //if (readRegister(0x00)&0x7) // Polling, you can use this instead of the interrupt pins
  {
    readRegisters(0x01, 6, &data[0]);  // Read the six data registers into data array

    /* For loop to calculate 12-bit ADC and g value for each axis */
    for (int i=0; i<6; i+=2)
    {
      accelCount[i/2] = ((data[i] << 8) | data[i+1]) >> 4;  // Turn the MSB and LSB into a 12-bit value
      if (data[i] > 0x7F)
      {  // If the number is negative, we have to make it so manually (no 12-bit data type)
        accelCount[i/2] = ~accelCount[i/2] + 1;
        accelCount[i/2] *= -1;  // Transform into negative 2's complement #
      }
      accel[i/2] = (float) accelCount[i/2]/((1<<12)/(2*scale));  // get actual g value, this depends on scale being set
    }
    /* For loop to print out values */
    for (int i=0; i<3; i++)
    {
      Serial.print(accel[i], 4);  // Print g values
      //Serial.print(accelCount[i], DEC);  // Print adc count values, feel free to uncomment this line
      Serial.print("\t\t");
    }
    Serial.println();
  }
  /* If int2 goes high, either p/l has changed or there's been a single/double tap */
  if (digitalRead(int2Pin))
  {
    source = readRegister(0x0C);  // Read the interrupt source reg.
    if ((source & 0x10)==0x10)  // If the p/l bit is set, go check those registers
      portraitLandscapeHandler();
    else if ((source & 0x08)==0x08)  // Otherwise, if tap register is set go check that
      tapHandler();
    delay(100);  // Delay here for a little printing visibility, make it longer, or delete it
  }
}

#endif


#if defined(USE_MMA8452Q_INCLINOMETER)
// *****************************************************************************
//				R e a d I n c l i n o m e t e r S a m p l e ( )
// *****************************************************************************

FILE_GLOBAL WORD gusStatusCtr = 0;		// counter to keep track of how many times we have to check Accelerometer status for new data; debugging only

// read X, Y, Z values and convert to 12 bit signed values, then to INT16 values
BOOL ReadInclinometerSample(PTR_INCLINOMETER_SAMPLE Inclination)
{
	BYTE bStatusRegister;
	INT16 sResult;
	BOOL bNegative = FALSE;
	BOOL bRetVal;

	// Using x y and z from accelerometer, calculate x and y angles
	float fResult;
	float x2, y2, z2; //24 bit

	if (MMA845x_Active() IS_FALSE)				// enable analog blocks for measurement
	{
		RuntimeError(INCLINOMETER_ERROR_ACCESS_FAILURE);
		return FALSE;							// communications error
	}

	gusStatusCtr = 0;							// clear status counter

	// in Active mode, analog blocks are enabled, so accelerometer measurements are made
	// wait for new data, indicated by ZYXDR_MASK bit in STATUS_00_REG

	do
	{
		if (ReadInclinometerRegister(STATUS_00_REG, &bStatusRegister) IS_FALSE)
		{
			RuntimeError(INCLINOMETER_ERROR_ACCESS_FAILURE);
			return FALSE;
		}

		++gusStatusCtr;							// count times through this loop; for debugging only. Typical value around 0x0C? Problem?

	// new data available?
	}
	while ((bStatusRegister & ZYXDR_MASK) IS 0);

#ifdef NOTDEF
	// read status register
	if (ReadInclinometerRegister(STATUS_00_REG, &bStatusRegister) IS_FALSE)
		return FALSE;

	// new data available?
	if ((bStatusRegister & ZYXDR_MASK) IS 0)
		return FALSE;
#endif

	if (MMA845x_Standby() IS_FALSE)				// disable analog blocks so no additional measurements occur
	{
		RuntimeError(INCLINOMETER_ERROR_ACCESS_FAILURE);
		return FALSE;							// communications error
	}
	

	// read data from Accelerometer
	if (ReadInclinometerArray(OUT_X_MSB_REG, &(Inclination->Acc_Sample.RAW_Data[0]), ACCELEROMETER_SAMPLE_LEN) IS_TRUE)
	{
		//*****************************
		//			X Axis
		//*****************************
		sResult = (INT16)(Inclination->Acc_Sample.XYZ_Sample.XYZ.x_msb * 16);					// shift 8MSB to correct position in INT16 fResult
		// check for negative value
		if ((Inclination->Acc_Sample.XYZ_Sample.XYZ.x_msb & OUT_SIGN_MASK) IS OUT_SIGN_MASK)	// check for negative number
		{
			// value is negative, so keep track of negative state
			bNegative = TRUE;
		}
		// read 4LSB of x axis; data is in the MSB position
		sResult += (Inclination->Acc_Sample.XYZ_Sample.XYZ.x_lsb / 16);							// shift 4LSB to LSB position
		if (bNegative IS TRUE)							// check for 8MSB negative number
		{
			sResult |= (INT16)VALUE_SIGN_BIT;			// restore sign bit in MSB location, also sets 3 unused bits
		}

		Inclination->nAcc_X_Value = sResult;			// save signed 12 bit value
		Inclination->fAcc_X_Value = (float)sResult;		// save as a float

		//*****************************
		//			Y Axis
		//*****************************
		bNegative = FALSE;
		sResult = (INT16)(Inclination->Acc_Sample.XYZ_Sample.XYZ.y_msb * 16);				// shift 8MSB to correct position in INT16 result
		// check for negative value
		if ((Inclination->Acc_Sample.XYZ_Sample.XYZ.y_msb & OUT_SIGN_MASK) IS OUT_SIGN_MASK)	// check for negative number
		{
			// value is negative, so keep track of negative state
			bNegative = TRUE;
		}
		// read 4LSB of x axis; data is in the MSB position
		sResult += (Inclination->Acc_Sample.XYZ_Sample.XYZ.y_lsb / 16);						// shift 4LSB to LSB position
		if (bNegative IS TRUE)							// check for 8MSB negative number
		{
			sResult |= (INT16)VALUE_SIGN_BIT;			// restore sign bit in MSB location, also sets 3 unused bits
		}

		Inclination->nAcc_Y_Value = sResult;			// save signed 12 bit value
		Inclination->fAcc_Y_Value = (float)sResult;		// save as a float

		//*****************************
		//			Z Axis
		//*****************************
		bNegative = FALSE;
		sResult = (INT16)(Inclination->Acc_Sample.XYZ_Sample.XYZ.z_msb * 16);				// shift 8MSB to correct position in INT16 result
		// check for negative value
		if ((Inclination->Acc_Sample.XYZ_Sample.XYZ.z_msb & OUT_SIGN_MASK) IS OUT_SIGN_MASK)	// check for negative number
		{
			// value is negative, so keep track of negative state
			bNegative = TRUE;
		}
		// read 4LSB of x axis; data is in the MSB position
		sResult += (Inclination->Acc_Sample.XYZ_Sample.XYZ.z_lsb / 16);						// shift 4LSB to LSB position
		if (bNegative IS TRUE)							// check for 8MSB negative number
		{
			sResult |= (INT16)VALUE_SIGN_BIT;			// restore sign bit in MSB location, also sets 3 unused bits
		}

		Inclination->nAcc_Z_Value = sResult;			// save signed 12 bit value
		Inclination->fAcc_Z_Value = (float)sResult;		// save as a float

		bRetVal = TRUE;

		//*****************************
		//		Calculate 3D Angles
		//*****************************

   // Lets get the deviations from our baseline
//   x_val = (float)accel_value_x-(float)accel_center_x;
//   y_val = (float)accel_value_y-(float)accel_center_y;
//   z_val = (float)accel_value_z-(float)accel_center_z;


		// Work out the squares
		x2 = (Inclination->fAcc_X_Value * Inclination->fAcc_X_Value);
		y2 = (Inclination->fAcc_Y_Value * Inclination->fAcc_Y_Value);
		z2 = (Inclination->fAcc_Z_Value * Inclination->fAcc_Z_Value);

		//X Axis, result is in Radians
		fResult = sqrtf(y2 + z2);

		if ((fResult > 0.01) OR (fResult < -0.01))			// check for divide by 0
		{
			fResult = Inclination->fAcc_X_Value / fResult;
			Inclination->fX_Angle = RADIANS_TO_DEGREES(atanf(fResult));		// convert to degrees
                      
		}
		else
		{
			// prevent divide by 0
			Inclination->fX_Angle = 0.0;
			RuntimeError(INCLINOMETER_ERROR_DIVIDE_BY_ZERO);
		}

		//Y Axis, result is in Radians
		fResult = sqrtf(x2 + z2);

		if ((fResult > 0.01) OR (fResult < -0.01))			// check for divide by 0
		{
			fResult = Inclination->fAcc_Y_Value / fResult;
			Inclination->fY_Angle = RADIANS_TO_DEGREES(atanf(fResult));		// convert to degrees
		}
		else
		{
			// prevent divide by 0
			Inclination->fY_Angle = 0.0;
			RuntimeError(INCLINOMETER_ERROR_DIVIDE_BY_ZERO);
		}
	}
	else
	{
		RuntimeError(INCLINOMETER_ERROR_ACCESS_FAILURE);
		bRetVal = FALSE;
	}

	return bRetVal;
	
}

// return X, Y, Z values and X, Y 3D Angles in a formatted string

BOOL FormatInclination(char *ptrOutputStr, PTR_INCLINOMETER_SAMPLE Inclination)
{

	int nStatus;

	strcpy(ptrOutputStr, "Acc X: ");
	INT16StoASCIIstr(Inclination->nAcc_X_Value, INT16S_WIDTH, ptrOutputStr + strlen(ptrOutputStr));

	strcat(ptrOutputStr, "\tAcc Y: ");
	INT16StoASCIIstr(Inclination->nAcc_Y_Value, INT16S_WIDTH, ptrOutputStr + strlen(ptrOutputStr));

	strcat(ptrOutputStr, "\tAcc Z: ");
	INT16StoASCIIstr(Inclination->nAcc_Z_Value, INT16S_WIDTH, ptrOutputStr + strlen(ptrOutputStr));

	strcat(ptrOutputStr, "\tDeg X: ");
	strcat(ptrOutputStr, ftoa2(Inclination->fX_Angle, &nStatus));		// display angle as degrees, 2 decimal places

	strcat(ptrOutputStr, "\tDeg Y: ");
	strcat(ptrOutputStr, ftoa2(Inclination->fY_Angle, &nStatus));		// display angle as degrees, 2 decimal places

	return TRUE;

}


// average multiple Inclinometer samples. Values are stores as degrees
BOOL AverageInclinometerSample(PTR_INCLINOMETER_SAMPLE Inclination)
{

	static BYTE newSampleIndex = 0;
	BYTE i;
	float fSumOfXSamples = 0.0;
	float fSumOfYSamples = 0.0;
	BOOL bRetVal = TRUE;

	// copy sample into averaging array
	pgAngleAveragingSamples[newSampleIndex].fX_Angle = Inclination->fX_Angle;
	pgAngleAveragingSamples[newSampleIndex].fY_Angle = Inclination->fY_Angle;

	// sum values in averaging array
	for (i = 0; i < INCLINOMETER_AVERAGING_COUNT; i++)
	{
		fSumOfXSamples += pgAngleAveragingSamples[i].fX_Angle;
		fSumOfYSamples += pgAngleAveragingSamples[i].fY_Angle;
	}

	// divide by sample count and save in global structure
	pgAngleAverage.fX_Angle = fSumOfXSamples / INCLINOMETER_AVERAGING_COUNT;
	pgAngleAverage.fY_Angle = fSumOfYSamples / INCLINOMETER_AVERAGING_COUNT;

	// bump array index
	++newSampleIndex;
	if(newSampleIndex IS INCLINOMETER_AVERAGING_COUNT)
	{
		newSampleIndex = 0;
	}

	return bRetVal;

}

// Initialize Inclinometer Averaging by refilling the averaging array (twice). Values are stores as degrees
BOOL Init_InclinometerSampleAveraging(void)
{
	BYTE i;

	// read inclinometer and fill averaging array TWICE to remove any remaining data
	for (i = 0; i < ( 2 * INCLINOMETER_AVERAGING_COUNT); i++)
	{
		if (ReadInclinometerSample(&pgInclination) IS_FALSE)					// read accelerometer, calculate 3D angles
		{
                    RuntimeError(INCLINOMETER_ERROR_READ_FAIL);
                    return FALSE;														// some sort of error has occured
		}
		IGNORE_RETURN_VALUE AverageInclinometerSample(&pgInclination);			// add to averaging array, calculate new running average
	}
	
	return TRUE;
}


BOOL FormatAverageInclination(char *ptrOutputStr)
{

	int nStatus;

	strcpy(ptrOutputStr, "\tAvg Inclination Deg X: ");
	strcat(ptrOutputStr, ftoa2(pgAngleAverage.fX_Angle, &nStatus));

	strcat(ptrOutputStr, "\tDeg Y: ");
	strcat(ptrOutputStr, ftoa2(pgAngleAverage.fY_Angle, &nStatus));

	return TRUE;

}


#endif	// defined(USE_MMA8452Q_INCLINOMETER)

#ifdef USE_INCLINOMETER_FEEDBACK

// *****************************************************************************
//					M o t i o n S e n s o r _ I n i t ( )
// *****************************************************************************

void MotionSensor_Init(void)
{

	// ********************************
	//		Setup Timer
	// ********************************

	// make sure interrupts are OFF
	MotionSensor_DisableInt(AXIS_AZIMUTH);
	MotionSensor_DisableInt(AXIS_ELEVATION);
	#ifndef _lint		// too many complex PC-Lint errors in hardware access

		// Setup Timer 3
		//	Timer 3 ON
		//	Divide by 256 (note timer input is 10MHZ)
		//  Internal source
		// Will provide a tick of 25.6uS. The shortest timing cycle we are likely to measure is 15mS

		//		   <<-------------T3CON------------->>   PR3	TMR3 = 0
		OpenTimer3(T3_ON | T3_PS_1_256 | T3_SOURCE_INT, 0xFFFF);
	#endif	// _lint

}


// *****************************************************************************
//					M o t i o n S e n s o r _ T i c k ( )
// *****************************************************************************
// called from the foreground loop to process the current speed and generate PWM change flags

INT32 fglLastMoveDistanceTicks[NUM_MOTORS] = {0L, 0L};

void MotionSensor_Tick(enum tagMotors eMotor)
{

	WORD wCurrentTimerValue[NUM_MOTORS];

	#ifdef USE_MOTION_SENSOR_TICK_TRIGGER
		Trigger1Level(1);					// trigger to allow viewing this event on a scope
	#endif

	// make sure the expected calling flag is in fact SET
	if (IS_BITCLEAR(efMotionSensorEvents[eMotor], EF_MOTION_SENSOR_TICK))
		{
		RuntimeError(MTN_SENSOR_ERROR_UNEXPECTED_TICK);
		}

	// clear the calling event flag
	BITCLEAR(efMotionSensorEvents[eMotor], EF_MOTION_SENSOR_TICK);

	// check for correct motor designation
	if (eMotor IS_NOT MOTOR_AZIMUTH)
	{
		RuntimeError(MTN_SENSOR_ERROR_UNEXPECTED_TICK);
		return;
	}


	//*******************************************************
	// this code is the equivalent of the MotionSensor_InterruptHandler

	// ********************************************
	//		Update Position
	// ********************************************
	// save previous position (in ticks)
	fglPreviousPosition[eMotor] = fglCurrentPosition[eMotor];
	
	// update position from the most recent inclinometer measurement
	fglCurrentPosition[eMotor] = ConvertDegreesToMSITicks(pgAngleAverage.fX_Angle, AXIS_AZIMUTH);
	fglCurrentPosition[MOTOR_ELEVATION] = 0L;

	// ********************************************
	//		Check for requested motion done
	// ********************************************
	// check for requested motion complete, based on averaged inclinometer values
	// If motion has completed, motion phase changes from Constant Speed Run to Deceleration
	// ==> because we IN MOTION, this value will actually be incorrect
	// ==>> distance traveled during deceleration and coasting WILL be a motion error

	if ((GetMotionState(eMotor) IS ST_MOTION_CONSTANT_SPEED) OR (GetMotionState(eMotor) IS ST_MOTION_CON_INCREASE_PWM) OR (GetMotionState(eMotor) IS ST_MOTION_CON_DECREASE_PWM))
	{
		if (pgePWMDirection[eMotor] IS PWM_DIR_FORWARD)
		{
			if (pgAngleAverage.fX_Angle > pgfEndingPositionDegrees[eMotor])
			{
				BITSET(efMotionEvents[eMotor], EF_MOTION_ANGLE_MOVE_DONE);
				pgMotionStats[eMotor].fEndingAngle = pgAngleAverage.fX_Angle;		// save angle at this point, NOT end of motion!

			}
		}
		else if (pgePWMDirection[eMotor] IS PWM_DIR_REVERSE)
		{
			if (pgAngleAverage.fX_Angle < pgfEndingPositionDegrees[eMotor])
			{
				BITSET(efMotionEvents[eMotor], EF_MOTION_ANGLE_MOVE_DONE);
				pgMotionStats[eMotor].fEndingAngle = pgAngleAverage.fX_Angle;		// save angle at this point, NOT end of motion!
			}
		}
	}

	// ********************************************
	//		Calculate Distance Moved
	// ********************************************
	// do we need to go through all the polarity cases here?
	fglLastMoveDistanceTicks[eMotor] = fglCurrentPosition[eMotor] - fglPreviousPosition[eMotor];

	// ********************************************
	//				Filter Values
	// ********************************************

	// THIS WILL NOT PERMIT REVERSE MOVES. OOOPS!

	// the values used here are converted from floats, and the Accelerometer value can be noisy, so it is POSSIBLE to get a NEGATIVE value here
//	if (fglLastMoveDistanceTicks[eMotor] < 0)
//		fglLastMoveDistanceTicks[eMotor]  = 0;

	// ********************************************
	//		track overall MSI Ticks
	// ********************************************
	// bump MSI ticks-in-current-motion-phase and total-ticks-in-move counters
	++pgwStateMSICtr[eMotor];										// increment count of MSI ticks in current motion state; always checked for >= LIMIT
	++pgulMoveTotalMSICtr[eMotor];									// count total MSI ticks in move

	// test for completed MSI ticks in motion phase
	// (this should be in the ISR to avoid latency issues)
	if (pgwStateMSICtr[eMotor] >= pgwStateMSILimit[eMotor])			// if we have completed the MSI ticks for the motion phase
		{
		// check for flag overrun, which could occur if we are coasting to a stop
		if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE))
			{
			RuntimeError(MTN_SENSOR_ERROR_MSI_COUNT_DONE_OVERRUN);
			}
		BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);	// set event flag for faster processing
		}

	// ********************************************
	//		Read Timer T3, current time in Ticks
	// ********************************************
	// read the time between Inclinometer Reads
	wCurrentTimerValue[eMotor] = ReadMSITimer(eMotor);

	// ********************************************
	//		Track Position by counting ticks
	// ********************************************
	//fglCurrentPosition[] is updated when this function is called, from the last Inclinometer reading

#ifdef NOTDEF
	// keep track of the current position by bumping fglCurrentPosition on each Motion Sensor tick.
	// NOTE: the 0 point is arbitrary;this will require a 'move to center' capability to set the 0 point

	switch(pgePWMDirection[eMotor])
		{
		case PWM_DIR_REVERSE:
			--fglCurrentPosition[eMotor] -= ;
			break;

		case PWM_DIR_FORWARD:
			++fglCurrentPosition[eMotor];
			break;

		case PWM_DIR_STOPPED:
		case PWM_DIR_UNKNOWN:							// error value
		default:
			// if we do not have a direction as above, we should not get a motion sensor tick!
			RuntimeError(MTN_SENSOR_ERROR_UNEXPECTED_INT);
			break;
		}
#endif


	// ********************************************
	//		track per-motion-phase MSI Ticks
	// ********************************************
	// keep track of Motion Sensor ticks per motion phase
	// this is done for debug logging, and is NOT required for the final system
	// (this should be in the ISR to avoid latency issues)
	switch(pgeMotionPhase[eMotor])
		{
		case PHASE_STOPPED:						// motion has actually STOPPED (Motion Sensor tick timeout), includes STALLED
			// this is a bit of a kludge, because we CERTAINLY do not exit the STOPPED state based on a state MSI tick count!
			BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);		// unconditionally clear flag

			RuntimeError(MTN_SENSOR_ERROR_UNEXPECTED_EVENT);
			break;

		case PHASE_ACCELERATION:				// accelerating speed
			++pgwMSI_AccelerationCtr[eMotor];	// MSI ticks for each of the motion phases
			break;

		case PHASE_CONSTANT_SPEED:			// constant speed operation
		case PHASE_OPEN_LOOP:				// used ONLY for open loop operation
			++pgulMSI_ConstantSpeedCtr[eMotor];
			break;

		case PHASE_DECELERATION:			// decelerating
			++pgwMSI_DecelerationCtr[eMotor];
			break;

		case PHASE_MINIMUM_PWM:				// constant speed at minimum PMW
			++pgwMSI_MinimumPWMCtr[eMotor];

			// this is a bit of a kludge, because we do not exit the MINIMUM_PWM state based on a state MSI tick count
			// (we don't really know how many ticks will occur in this state; it is just whatever did not get done during deceleration.)
//////			BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);		// unconditionally clear flag
			break;

		case PHASE_COASTING:				// motor is OFF, some coasting MAY occur
			++pgwMSI_CoastCtr[eMotor];

			// this is a bit of a kludge, because we do not exit the COASTING state based on a state MSI tick count
			BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);		// unconditionally clear flag
			break;

		case PHASE_INIT:					// not even valid here
		default:
			RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_PHASE);
			break;
		}



	// ********************************************
	//			Calculate Speed
	// ********************************************
	// (this does not involve any more measurements, so it does not HAVE to be in the ISR)

	switch(pgeMotionType[eMotor])
		{
		case MOTION_STARTING:
			// this is the FIRST MSI tick after starting up, so we cannot do any speed calculations yet
			// fgwLastTimerValue is set below

			// this must be the first encoder interrupt after being STOPPED, so we cannot calculate speed
			pguCurrentSpeed[eMotor] = 0;

			// update stall counter to 1.5 times (?) longest expected interval to avoid a motion stall during startup
			//SetMotionStallCtr(eMotor, MTN_SENSOR_STARTUP_STALL_PERIOD(eMotor));

			#ifdef OPEN_LOOP_MOVES_ONLY
				// kludge for open loop ONLY operation, which #ifdefs out most of the tick handler
				pgeMotionType[eMotor] = MOTION_POWERED;
			#endif
			break;

		case MOTION_POWERED:
			// we have been in motion, so we can calculate speed

			#ifdef CALC_AVERAGE_SPEED
				// keep the previous speed value
				fgwLastSpeed[eMotor] = pguCurrentSpeed[eMotor];
			#endif

			// *********************************
			//	Calculate operating speed
			// *********************************
			// update the speed value, the interrupt-to-interrupt time, (in 6.4uS ticks)
			// check for (free running) counter rollover during the timed interval
			// NOTE: this is depedent on the free running T3 timer value stored during the PREVIOUS interrupt, fgwLastTimerValue[eMotor]
			if (wCurrentTimerValue[eMotor] > fgwLastTimerValue[eMotor])
				{
				// no counter rollover
				// calculate speed (number of counter ticks) since last interrupt, counter INCREMENTS
				pguCurrentSpeed[eMotor] = wCurrentTimerValue[eMotor] - fgwLastTimerValue[eMotor];
				}
			else
				{
				// counter has rolled over
				pguCurrentSpeed[eMotor] = wCurrentTimerValue[eMotor] + (MTN_SENSOR_TMR_MAX_VALUE - fgwLastTimerValue[eMotor]);
				}


			// *********************************
			//		track speed values
			// *********************************
			// keep track of maximum speed - NOTE: speed is measured as a count of Tcy, so smaller is faster
			if  (pguCurrentSpeed[eMotor] < pgMotionStats[eMotor].MaximumSpeed)
				{
				pgMotionStats[eMotor].MaximumSpeed = pguCurrentSpeed[eMotor];
				}

			// keep track of minimum speed
			if  (pguCurrentSpeed[eMotor] > pgMotionStats[eMotor].MinimumSpeed)
				{
				pgMotionStats[eMotor].MinimumSpeed = pguCurrentSpeed[eMotor];
				}

			// update stall counter to 1.5 times (?) the current speed value
			// bounds check current speed value, calculation is (x) * 3 / 2 or (x) * 3
			if (pguCurrentSpeed[eMotor] < MAX_MTN_SENSOR_TICK_FOR_STALL )
			{
				//SetMotionStallCtr(eMotor, MTN_SENSOR_MOTION_STALL_PERIOD(pguCurrentSpeed[eMotor]));
			}
			else
			{
				// using the current speed value will result in an undersize stall counter value
				// so limit counter value to maximum usable value
				//SetMotionStallCtr(eMotor, MAX_MTN_SENSOR_TICK_FOR_STALL);
			}

			#ifdef CALC_AVERAGE_SPEED
				// calculate the average speed
				++fgwSampleCtr;
				if (fgwSampleCtr > 100)
					{
					// restart all sampling
					fgwSampleCtr = 1;
					fgwSumOfSpeeds = 0;
					fgwLastSpeed = pguCurrentSpeed;		// to prevent meaningless Error From Sample values
					}

				fgwSumOfSpeeds += pguCurrentSpeed;
				pgwAverageSpeed = fgwSumOfSpeeds / fgwSampleCtr;		// note that sampleCtr is at LEAST 1

				// calculate error from average speed
				pgnErrorFromAverage = (int)pgwAverageSpeed - (int)pguCurrentSpeed;

				// calculate error from most recent sample
				pgnErrorFromSample = (int)fgwLastSpeed - (int)pguCurrentSpeed;
			#endif

			break;

		case MOTION_COASTING:			// motor is OFF, some coasting MAY occur
		case MOTION_BRAKING:
			// power is off, and we are COASTING, this is processed just see how long we coast!

			// this must be the first encoder interrupt after being STOPPED, so we cannot calculate speed
//			pguCurrentSpeed = 0;

			// update stall counter to 1.5 (?) times longest expected interval
			//SetMotionStallCtr(eMotor, MTN_SENSOR_COASTING_STALL_PERIOD(eMotor));
			SetMotionStallCtr(eMotor, 4000 /*MTN_SENSOR_COASTING_STALL_PERIOD(eMotor)*/);

			////RuntimeError(MTN_SENSOR_ERROR_COASTING_INT);		// this is an event, not an error!
			break;

		case MOTION_INIT:				// initial state, only at power up
		case MOTION_STOPPED:			// motion has actually STOPPED (Motion Sensor tick timeout)
		case MOTION_STALLED:			// system has STALLED; no exit
		default:
			RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_INT);
			break;
		}		// end 	switch(pgeMotionType[eMotor])

	// update stored timer value for next interrupt speed calculation
	fgwLastTimerValue[eMotor] = wCurrentTimerValue[eMotor];

	// end of equivalent of the MotionSensor_InterruptHandler
	//*******************************************************


	// save the Current Orientation to MCU RAM copy of RTCC NV RAM, so we can recover it if power is lost
	ptrRTCC_RAM_MechanicalOrientation->lLastAzimuth = CurrentPosition_Read(MOTOR_AZIMUTH);
	ptrRTCC_RAM_MechanicalOrientation->lLastElevation = CurrentPosition_Read(MOTOR_ELEVATION);

	#ifdef USE_MOTION_SENSOR_TICK_RAM_UPDATE_TRIGGER
		Trigger1Level(1);					// trigger to allow viewing this event on a scope
	#endif

	#ifdef USE_DS3232_RTCC
		// write MCU RAM copy of Current Orientation to NV RTCC RAM, so we can recover it if power is lost
		IGNORE_RETURN_VALUE UpdateRTCCRAMOrientation();
	#endif

	#ifdef USE_MOTION_SENSOR_TICK_RAM_UPDATE_TRIGGER
		Trigger1Level(0);					// trigger to allow viewing this event on a scope
	#endif

	// speed handling depends on the motion phase
	switch(pgeMotionPhase[eMotor])
		{
		case PHASE_STOPPED:					// motion has actually STOPPED (Motion sensor tick timeout), includes STALLED
			RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_PHASE);
			break;

		case PHASE_ACCELERATION:			// accelerating speed
			switch(pgeMotionType[eMotor])
				{
				case MOTION_STARTING:
					// this is the FIRST MSI tick after starting up, so we cannot do any calculations yet
					// pgMotionStats.wAccelerationPWMAdjustmentCount = 0;

					// in acceleration, every MSI tick results in a PWM value update
					BITSET(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);

					// we have now gone through ONE MSI interrupt and ONE MSI Tick routine, so on the NEXT MSI Tick, we can process the measured speed
					pgeMotionType[eMotor] = MOTION_POWERED;
					break;

				case MOTION_POWERED:
					// we have been in motion, so we can calculate speed
					// we are NOW past the very first MSI tick, for which we do NOT have any speed info

					// bounds check incremented MotionProfile speed index
					if (pgbMotionProfileSpeedIndex[eMotor] <= (pgbMotionProfileIndexMax[eMotor] - pgbMotionProfileIndexIncrement[eMotor]))
//					if (pgbMotionProfileSpeedIndex[eMotor] <= (pgbMotionProfileIndexMax[eMotor] - ABS(fglLastMoveDistanceTicks[eMotor])) )
						{
						// bump index into motion profile table of expected speed values
						// calculate new motion profile index, and look at expected speed value (in Tcy counts)

						// bump motion profile table index ==> should really be bumped by fglLastMoveDistanceTicks[]
						pgbMotionProfileSpeedIndex[eMotor] += pgbMotionProfileIndexIncrement[eMotor];

						// read expected speed from motion profile table
						fguExpectedSpeed[eMotor] = MotionProfileSpeedAndPWM[eMotor][(pgbMotionProfileSpeedIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + MSI_SPEED_OFFSET];

						// keep track of speed error for status display
						pgsSpeedError[eMotor] = (MOTION_PROFILE_SPEED_ERR_TYPE)(fguExpectedSpeed[eMotor] - pguCurrentSpeed[eMotor]);

						#ifdef MOTION_ERROR_TABLE
							// a positive value means that the current speed is numerically less than the expected speed - meaning we are going TOO FAST
							// a negative value means that the current speed is numerically more than the expected speec - meaning we are going TOO SLOW
							if (pgwMotionProfileSpeedErrorIndex < SPEED_ERROR_TABLE_LEN)
							{
								pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bMotionProfileIndex = pgbMotionProfileSpeedIndex[eMotor];
								pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].ExpectedSpeed = fguExpectedSpeed[eMotor];
								pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].MeasuredSpeed = pguCurrentSpeed[eMotor];
								pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].SpeedError = pgsSpeedError[eMotor];
								pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].cPWMCorrection = pgcDutyCycleCorrection[eMotor];
								pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bPWM = (BYTE)pgbPWMDutyCycle[eMotor];
								pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].LastMoveDistanceTicks = fglLastMoveDistanceTicks[eMotor];

								++pgwMotionProfileSpeedErrorIndex;					// bump table index
							}
						#endif

						// compare current speed to expected maximum speed.
						// Speed is measured as a count of Tcy ticks, between MSI ticks, so smaller is faster
						if (pguCurrentSpeed[eMotor] < (fguExpectedSpeed[eMotor] - MTN_SENSOR_SPEED_TOLERANCE))		// smaller is faster
							{
							// set event flag for motion FSM
							BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);
							}
						else if (pguCurrentSpeed[eMotor] > (fguExpectedSpeed[eMotor] + MTN_SENSOR_SPEED_TOLERANCE))
							{
							// set event flag for motion FSM
							BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);
							}
						else
							{
							// in acceleration, every MSI tick results in a PWM value update of SOME type
							BITSET(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);
							}
						}
					else
						{
						RuntimeError(MTN_SENSOR_ERROR_NO_EVENT_GENERATED);
						}

					break;

				// NONE of these motion types should occur during PHASE_ACCELERATION
				case MOTION_INIT:				// initial state, only at power up
				case MOTION_COASTING:			// motor is OFF, some coasting MAY occur
				case MOTION_BRAKING:			// motor is OFF, brake is on, some coasting MAY occur
				case MOTION_STOPPED:			// motion has actually STOPPED (Motion Sensor tick timeout)
				case MOTION_STALLED:			// system has STALLED; no exit
				default:
					RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK);
					break;

				}		// end 	switch(pgeMotionType[eMotor])
			break;

		case PHASE_OPEN_LOOP:				// used ONLY for open loop operation
			pgeMotionType[eMotor] = MOTION_POWERED;			// to allow speed measurement
			// in Open Loop mode, nothing else to do here; NO speed adjustments
			break;

		case PHASE_CONSTANT_SPEED:			// constant speed operation
			if (pgeMotionType[eMotor] IS MOTION_POWERED)
				{
				// NOTE that index into motion profile table of expected speed values does NOT change during the Constant Speed phase
				// calculate motion profile index (which does not change during Constant Speed), and look up expected speed value (in Tcy counts)
				fguExpectedSpeed[eMotor] = MotionProfileSpeedAndPWM[eMotor][(pgbMotionProfileSpeedIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + MSI_SPEED_OFFSET];

				// keep track of speed error for status display
				pgsSpeedError[eMotor] = (MOTION_PROFILE_SPEED_ERR_TYPE)(fguExpectedSpeed[eMotor] - pguCurrentSpeed[eMotor]);

				#ifdef MOTION_ERROR_TABLE
					// a positive value means that the current speed is numerically less than the expected speed - meaning we are going TOO FAST
					// a negative value means that the current speed is numerically more than the expected speec - meaning we are going TOO SLOW
					if (pgwMotionProfileSpeedErrorIndex < SPEED_ERROR_TABLE_LEN)
					{
						pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bMotionProfileIndex = pgbMotionProfileSpeedIndex[eMotor];
						pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].ExpectedSpeed = fguExpectedSpeed[eMotor];
						pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].MeasuredSpeed = pguCurrentSpeed[eMotor];
						pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].SpeedError = pgsSpeedError[eMotor];
						pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].cPWMCorrection = pgcDutyCycleCorrection[eMotor];
						pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bPWM = (BYTE)pgbPWMDutyCycle[eMotor];
						pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].LastMoveDistanceTicks = fglLastMoveDistanceTicks[eMotor];

						++pgwMotionProfileSpeedErrorIndex;					// bump table index
					}
				#endif


				// compare current speed to expected maximum speed.
				// Speed is measured as a count of Tcy ticks, between MSI ticks, so smaller is faster
				if (pguCurrentSpeed[eMotor] < (fguExpectedSpeed[eMotor] - MTN_SENSOR_SPEED_TOLERANCE))		// smaller is faster
					{
					// set event flag for motion FSM
					BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);
					}
				else if (pguCurrentSpeed[eMotor] > (fguExpectedSpeed[eMotor] + MTN_SENSOR_SPEED_TOLERANCE))
					{
					// set event flag for motion FSM
					BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);
					}
				}
			else
				{
				// not a valid MOTION_xxx type
				RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK);
				}
			break;

		case PHASE_DECELERATION:			// decelerating
			// pgMotionStats.wQDecelerationPWMAdjustmentCount = 0;
			// NOTE: during deceleration, Motion Profile index pgbMotionProfileSpeedIndex is adjusted by MotionFSM.c (why?)
			// we are no longer adjusting speed, so there is no error to measure
			fguExpectedSpeed[eMotor] = MotionProfileSpeedAndPWM[eMotor][(pgbMotionProfileSpeedIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + MSI_SPEED_OFFSET];

			// keep track of speed error for status display ONLY
			pgsSpeedError[eMotor] = (MOTION_PROFILE_SPEED_ERR_TYPE)(fguExpectedSpeed[eMotor] - pguCurrentSpeed[eMotor]);


			#ifdef MOTION_ERROR_TABLE
				// a positive value means that the current speed is numerically less than the expected speed - meaning we are going TOO FAST
				// a negative value means that the current speed is numerically more than the expected speec - meaning we are going TOO SLOW
				if (pgwMotionProfileSpeedErrorIndex < SPEED_ERROR_TABLE_LEN)
				{
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bMotionProfileIndex = pgbMotionProfileSpeedIndex[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].ExpectedSpeed = fguExpectedSpeed[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].MeasuredSpeed = pguCurrentSpeed[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].SpeedError = pgsSpeedError[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].cPWMCorrection = pgcDutyCycleCorrection[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bPWM = (BYTE)pgbPWMDutyCycle[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].LastMoveDistanceTicks = fglLastMoveDistanceTicks[eMotor];

					++pgwMotionProfileSpeedErrorIndex;					// bump table index
				}
			#endif

			if (pgeMotionType[eMotor] IS MOTION_POWERED)
				{
				// in deceleration, every MSI tick results in a PWM value update
				BITSET(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);
				}
			else
				{
				// not a valid MOTION_xxx type
				RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK);
				}
			break;

		case PHASE_MINIMUM_PWM:				// constant speed at minimum PMW
			// pgMotionStats.wQDecelerationPWMAdjustmentCount = 0;
			if (pgeMotionType[eMotor] IS MOTION_POWERED)
				{
				;
				}
			else
				{
				// not a valid MOTION_xxx type
				RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK);
				}
			break;

		case PHASE_COASTING:				// motor is OFF, some coasting MAY occur
			if ((pgeMotionType[eMotor] IS MOTION_COASTING) OR (pgeMotionType[eMotor] IS MOTION_BRAKING))
				{
				;
				}
			else
				{
				// not a valid MOTION_xxx type
				RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK);
				}
			break;

		case PHASE_INIT:					// not even valid here
		default:
			RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_PHASE);
			break;
		}

	// make sure the expected calling flag is STILL CLEAR
	if (IS_BITSET(efMotionSensorEvents[eMotor], EF_MOTION_SENSOR_TICK))
		{
		RuntimeError(MTN_SENSOR_ERROR_UNEXPECTED_TICK);
		}

	// NOTE: we do not make a general check for unprocessed efADCEvents events here, because there is only ONE event type
	// (EF_MTN_SENSOR_MOTION_SENSOR_TICK) and it is interrupt generated - so another event may occur at ANY time.

	#ifdef USE_MOTION_SENSOR_TICK_TRIGGER
		Trigger1Level(0);					// trigger to allow viewing this event on a scope
	#endif

}



// *****************************************************************************
//				M o t i o n S e n s o r _ x x x x x I n t ( )
// *****************************************************************************

// enable or disable Motion Sensor interrupts
// we need to turn off the interrupts AFTER a motion stall, so that we do not get any additional interrupts until intentional motion starts again

// Enable Encoder interrupts
// Called from MotionFSM() upon entering ST_MOTION_ACCELERATE and during ST_MOTION_SOFT_STALL
void MotionSensor_EnableInt(enum tagAxis eAxis)
{
	BITSET(efMotionSensorEvents[eAxis], EF_MOTION_SENSOR_ACTIVE);
}

// Disable Encoder interrupts
// Called from MotionFSM() upon entering ST_MOTION_STOPPED
void MotionSensor_DisableInt(enum tagAxis eAxis)
{
	BITCLEAR(efMotionSensorEvents[eAxis], EF_MOTION_SENSOR_ACTIVE);
}


#endif	// USE_INCLINOMETER_FEEDBACK


// end of Inclinometer.c
