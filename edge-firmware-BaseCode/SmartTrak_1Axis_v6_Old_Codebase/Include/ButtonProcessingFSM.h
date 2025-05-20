// *************************************************************************************************
//							B u t t o n P r o c e s s i n g F S M . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Button (Input Switch) Processing FSM, User Input definitions
//
//
// *************************************************************************************************


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------
#define SW1_MODE        !PORTEbits.RE4  /* Push Button */
#define SW2_EAST        !PORTAbits.RA5  /* Push Button */
#define SW3_WEST        !PORTFbits.RF13  /* Push Button */

#define BOUNCE_COUNT    5

char *GetButtonProcessingStateString(void);
void ButtonProcessingFSM(void);
void Switch_Init(void);
void Switch_processing(void);


// end of ButtonProcessing.h


