// *************************************************************************************************
//					BMS. h
// *************************************************************************************************
//
//		Project:	SAT
//
//		Contains:	PIC32 BMS Functions
//
//
// *************************************************************************************************

struct ADBUF_TYPE {
	int	buf;
	int	void1;
	int	void2;
	int	void3;
};

//
/*   POT model : 3545S-1-102L, 1k ohm, ref voltage=5v
 *   3 turns for 360 degrees(according to our gears), 3v for full 360deg
     max deg	360	0.33
     revolution	360	2.97

#define     REF_DEG     90
#define     REF_VOL     0.9747

#define     MIN_DEG     0
#define     MAX_DEG     360       //Max Peek current of sensor
#define     TOTAL_REV   (MAX_DEG-MIN_DEG)

#define     MIN_DEG_VOL   0.1
#define     MAX_DEG_VOL   3.1



#define     DEG_RESOLUTION ((MAX_DEG_VOL - MIN_DEG_VOL)/(TOTAL_REV))   //in volts  (3.3 -0.33)/360 = 0.00825
#define     VOL_RESOLUTION  (1/DEG_RESOLUTION)               //in degrees    1/
//#define     VOL_TO_DEG(X)  ((float)(X - MIN_DEG_VOL)*(float)VOL_RESOLUTION)


#define     REF_VOL_CAL     (REF_VOL - (REF_DEG*DEG_RESOLUTION))
#define     VOL_TO_DEG(X)  ((float)(X - REF_VOL_CAL)*(float)VOL_RESOLUTION)
 */
#define     CNV_ADC2V   (3.3/1023.0)  //Adc to Voltage conversion

#define BAT_VOL_CHANNEL         0
#define MOTOR1_CUR_CHANNEL      1
#define MOTOR2_CUR_CHANNEL      2
#define TEMP_CHANNEL1           3
#define TEMP_CHANNEL2           4
#define OP_CHANNEL2             5

#define NO_PARAMETERS   7
#define NO_SAMPLES      10

#define TEMP_CHANNEL    3
#define AZC_CHANNEL     0
#define ELC_CHANNEL     1



#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#ifdef DEFINE_GLOBALS
	GLOBAL_INIT ARRAY float	BMS_V=0,M1C=0,M2C=0,BTemp=0,OP1=0,OP2=0;
        GLOBAL_INIT ARRAY int nadc[NO_PARAMETERS],padc[NO_PARAMETERS];
#elif defined (DEFINE_EXTERNS)
        GLOBAL ARRAY float      BMS_V,M1C,M2C,BTemp,OP1,OP2;
        GLOBAL ARRAY int        nadc[],padc[];
#endif

