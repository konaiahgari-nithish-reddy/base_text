/********************************************************************
 * FileName:		int.h
 * Dependencies:
 * Processor:		PIC32
 * Hardware:		N/A
 * Assembler:		N/A
 * Linker:		    N/A
 * Company:		    Microchip Technology Inc..
 *
 * Software License Agreement:
 * The software supplied herewith by Microchip Technology Incorporated
 * (the �Company�) for its PICmicro� Microcontroller is intended and
 * supplied to you, the Company�s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * $Id: INT.h,v 1.6 2006/11/07 23:29:45 C12923 Exp $
 * $Name:  $

 ********************************************************************/

#ifndef _OLD_PIC32INT_5XX_6XX_7XX_HEADER_FILE
#define _OLD_PIC32INT_5XX_6XX_7XX_HEADER_FILE

#include <p32xxxx.h>
#include <sys/attribs.h>

// SCM Name Change Mapping Defines
#define INT_SPI3        INT_SPI1A       
#define INT_SPI3E       INT_SPI1AE      
#define INT_SPI3TX      INT_SPI1ATX     
#define INT_SPI3RX      INT_SPI1ARX     

#define INT_SPI2        INT_SPI2A       
#define INT_SPI2E       INT_SPI2AE      
#define INT_SPI2TX      INT_SPI2ATX     
#define INT_SPI2RX      INT_SPI2ARX     

#define INT_SPI4        INT_SPI3A       
#define INT_SPI4E       INT_SPI3AE      
#define INT_SPI4TX      INT_SPI3ATX     
#define INT_SPI4RX      INT_SPI3ARX     

#define INT_I2C3        INT_I2C1A
#define INT_I2C3B       INT_I2C1AB
#define INT_I2C3S       INT_I2C1AS
#define INT_I2C3M       INT_I2C1AM

#define INT_I2C4        INT_I2C2A
#define INT_I2C4B       INT_I2C2AB
#define INT_I2C4S       INT_I2C2AS
#define INT_I2C4M       INT_I2C2AM

#define INT_I2C5        INT_I2C3A
#define INT_I2C5B       INT_I2C3AB
#define INT_I2C5S       INT_I2C3AS
#define INT_I2C5M       INT_I2C3AM

#define INT_U1          INT_U1A
#define INT_U1E         INT_U1AE
#define INT_U1RX        INT_U1ARX
#define INT_U1TX        INT_U1ATX

#define INT_U4          INT_U1B
#define INT_U4E         INT_U1BE
#define INT_U4RX        INT_U1BRX
#define INT_U4TX        INT_U1BTX

#define INT_U3          INT_U2A
#define INT_U3E         INT_U2AE
#define INT_U3RX        INT_U2ARX
#define INT_U3TX        INT_U2ATX

#define INT_U6          INT_U2B
#define INT_U6E         INT_U2BE
#define INT_U6RX        INT_U2BRX
#define INT_U6TX        INT_U2BTX

#define INT_U2          INT_U3A
#define INT_U2E         INT_U3AE
#define INT_U2RX        INT_U3ARX
#define INT_U2TX        INT_U3ATX

#define INT_U5          INT_U3B
#define INT_U5E         INT_U3BE
#define INT_U5RX        INT_U3BRX
#define INT_U5TX        INT_U3BTX

#define INT_SPI_2_VECTOR        INT_SPI_2A_VECTOR
#define INT_SPI_3_VECTOR        INT_SPI_1A_VECTOR
#define INT_SPI_4_VECTOR        INT_SPI_3A_VECTOR

#define INT_UART_1_VECTOR       INT_UART_1A_VECTOR
#define INT_UART_2_VECTOR       INT_UART_3A_VECTOR
#define INT_UART_4_VECTOR       INT_UART_1B_VECTOR
#define INT_UART_3_VECTOR       INT_UART_2A_VECTOR
#define INT_UART_6_VECTOR       INT_UART_2B_VECTOR
#define INT_UART_5_VECTOR       INT_UART_3B_VECTOR

#define INT_I2C_3_VECTOR        INT_I2C_1A_VECTOR
#define INT_I2C_4_VECTOR        INT_I2C_2A_VECTOR
#define INT_I2C_5_VECTOR        INT_I2C_3A_VECTOR



/****************************************************************************************************
  * <combinewith mCTGetIntFlag, mCTIntEnable, mCTGetIntEnable, mCTSetIntPriority, mCTGetIntPriority, 
  * mCTSetIntSubPriority, mCTGetIntSubPriority>
  * 
  * Core Timer Control Interrupt Functions
  * Prototype:
  *     void mCTClearIntFlag(void) int mCTGetIntFlag(void) void
  *     mCTSetIntEnable(void) int mCTGetIntEnable(void) void
  *     mCTClearIntEnable(void) void mCTSetIntPriority(int priority)
  *     int mCTGetIntPriority(void) void
  *     mCTSetIntSubPriority(subPriority) int
  *     mCTGetIntSubPriority(void)
  * Preconditions:
  *     EBASE and IntCtl.VS set up
  * Input:
  *     priority :     A value between 0 \- 7 inclusive.
  *     subPriority :  A value between 0 \- 3 inclusive
  * Return:
  *     For 'Get' functions, the returned values are of
  *     same type as 'Set' functions.
  *     
  * Side Effects:
  *     None
  * Remarks:
  *     None                                                                                                                                    
  *****************************************************************************************************
*/
#define mCTClearIntFlag()                   (IFS0CLR = _IFS0_CTIF_MASK)
#define mCTGetIntFlag()                     (IFS0bits.CTIF)
#define mCTIntEnable(enable)                (IEC0CLR = _IEC0_CTIE_MASK, IEC0SET = ((enable) << _IEC0_CTIE_POSITION))
#define mCTGetIntEnable()                   (IEC0bits.CTIE)
#define mCTSetIntPriority(priority)         (IPC0CLR = _IPC0_CTIP_MASK, IPC0SET = ((priority) << _IPC0_CTIP_POSITION))
#define mCTGetIntPriority()                 (IPC0bits.CTIP)
#define mCTSetIntSubPriority(subPriority)   (IPC0CLR = _IPC0_CTIS_MASK, IPC0SET = ((subPriority) << _IPC0_CTIS_POSITION))
#define mCTGetIntSubPriority()              (IPC0bits.CTIS)
/*********************************************************************************************************
  * <combinewith mCS0GetIntFlag, mCS0IntEnable, mCS0GetIntEnable, mCS0SetIntPriority, mCS0GetIntPriority, 
  * mCS0SetIntSubPriority, mCS0GetIntSubPriority>
  * 
  * Core Software Interrupt 0 Control Functions
  * Prototype:
  *     void mCS0ClearIntFlag(void) int mCS0GetIntFlag(void) void
  *     mCS0SetIntEnable(void) int mCS0GetIntEnable(void) void
  *     mCS0ClearIntEnable(void) void mCS0SetIntPriority(int
  *     priority) int mCS0GetIntPriority(void) void
  *     mCS0SetIntSubPriority(subPriority) int
  *     mCS0GetIntSubPriority(void)
  * Preconditions:
  *     EBASE and IntCtl.VS set up
  * Input:
  *     priority :     A value between 0 \- 7 inclusive.
  *     subPriority :  A value between 0 \- 3 inclusive
  * Return:
  *     For 'Get' functions, the returned values are of
  *     same type as 'Set' functions.
  *     
  * Side Effects:
  *     None
  * Remarks:
  *     None                                                                                                                                           
  **************************************************************************************************************
*/
#define mCS0ClearIntFlag()                  (IFS0CLR = _IFS0_CS0IF_MASK)
#define mCS0GetIntFlag()                    (IFS0bits.CS0IF)
#define mCS0IntEnable(enable)               (IEC0CLR = _IEC0_CS0IE_MASK, IEC0SET = ((enable) << _IEC0_CS0IE_POSITION))
#define mCS0GetIntEnable()                  (IEC0bits.CS0IE)
#define mCS0SetIntPriority(priority)        (IPC0CLR = _IPC0_CS0IP_MASK, IPC0SET = ((priority) << _IPC0_CS0IP_POSITION))
#define mCS0GetIntPriority()                (IPC0bits.CS0IP)
#define mCS0SetIntSubPriority(subPriority)  (IPC0CLR = _IPC0_CS0IS_MASK, IPC0SET = ((subPriority) << _IPC0_CS0IS_POSITION))
#define mCS0GetIntSubPriority()             (IPC0bits.CS0IS)
/**************************************************************************************
  * <combinewith mCS1GetIntEnable, mCS1GetIntFlag, mCS1GetIntPriority, 
  * mCS1GetIntSubPriority, mCS1IntEnable, mCS1SetIntPriority, mCS1SetIntSubPriority>
  * 
  * Core Software Interrupt 1 Control Functions
  * Prototype:
  *     void mCS1ClearIntFlag(void) int mCS1GetIntFlag(void) void
  *     mCS1SetIntEnable(void) int mCS1GetIntEnable(void) void
  *     mCS1ClearIntEnable(void) void mCS1SetIntPriority(int
  *     priority) int mCS1GetIntPriority(void) void
  *     mCS1SetIntSubPriority(subPriority) int
  *     mCS1GetIntSubPriority(void)
  * Preconditions:
  *     EBASE and IntCtl.VS set up
  * Input:
  *     priority :     A value between 0 \- 7 inclusive.
  *     subPriority :  A value between 0 \- 3 inclusive
  *     edgeMode :     1 = Rising, 0 = Falling
  * Return:
  *     For 'Get' functions, the returned values are of
  *     same type as 'Set' functions.
  *     
  * Side Effects:
  *     None
  * Remarks:
  *     None                                                                                                                                           
  ************************************************************************************
*/
#define mCS1ClearIntFlag()                  (IFS0CLR = _IFS0_CS1IF_MASK)
#define mCS1GetIntFlag()                    (IFS0bits.CS1IF)
#define mCS1GetIntEnable()                  (IEC0bits.CS1IE)
#define mCS1IntEnable(enable)               (IEC0CLR = _IEC0_CS1IE_MASK, IEC0SET = ((enable) << _IEC0_CS1IE_POSITION))
#define mCS1SetIntPriority(priority)        (IPC0CLR = _IPC0_CS1IP_MASK, IPC0SET = ((priority) << _IPC0_CS1IP_POSITION))
#define mCS1GetIntPriority()                (IPC0bits.CS1IP)
#define mCS1SetIntSubPriority(subPriority)  (IPC0CLR = _IPC0_CS1IS_MASK, IPC0SET = ((subPriority) << _IPC0_CS1IS_POSITION))
#define mCS1GetIntSubPriority()             (IPC0bits.CS1IS)

/************************************************************************************************************************************************************************************************************
  * <combinewith mINT0GetIntFlag, mINT0GetIntEnable, mINT0IntEnable, mINT0SetIntPriority, mINT0GetIntPriority, mINT0SetIntSubPriority, mINT0GetIntSubPriority, mINT0SetEdgeMode, mINT0GetEdgeMode,
  * mINT1ClearIntFlag, mINT1GetIntFlag, mINT1GetIntEnable, mINT1IntEnable, mINT1SetIntPriority, mINT1GetIntPriority, mINT1SetIntSubPriority, mINT1GetIntSubPriority, mINT1SetEdgeMode, mINT1GetEdgeMode,
  * mINT2ClearIntFlag, mINT2GetIntFlag, mINT2GetIntEnable, mINT2IntEnable, mINT2SetIntPriority, mINT2GetIntPriority, mINT2SetIntSubPriority, mINT2GetIntSubPriority, mINT2SetEdgeMode, mINT2GetEdgeMode,
  * mINT3ClearIntFlag, mINT3GetIntFlag, mINT3GetIntEnable, mINT3IntEnable, mINT3SetIntPriority, mINT3GetIntPriority, mINT3SetIntSubPriority, mINT3GetIntSubPriority, mINT3SetEdgeMode, mINT3GetEdgeMode,
  * mINT4ClearIntFlag, mINT4GetIntFlag, mINT4GetIntEnable, mINT4IntEnable, mINT4SetIntPriority, mINT4GetIntPriority, mINT4SetIntSubPriority, mINT4GetIntSubPriority, mINT4SetEdgeMode, mINT4GetEdgeMode>
  * 
  * External INTx Control Functions
  * Prototype:
  *     void mINTxClearIntFlag(void) int mINTxGetIntFlag(void) void
  *     mINTxSetIntEnable(void) int mINTxGetIntEnable(void) void
  *     mINTxClearIntEnable(void) void mINTxSetIntPriority(int
  *     priority) int mINTxGetIntPriority(void) void
  *     mINTxSetIntSubPriority(subPriority) int
  *     mINTxGetIntSubPriority(void) void mINTxSetEdgeRising(void)
  *     void mINTxGetEdgeFalling(void) int mINTxGetEdgeMode(void)
  * Preconditions:
  *     EBASE and IntCtl.VS set up
  * Input:
  *     priority :     A value between 0 \- 7 inclusive.
  *     subPriority :  A value between 0 \- 3 inclusive
  * Return:
  *     For 'Get' functions, the returned values are of
  *     same type as 'Set' functions.
  *     
  * Side Effects:
  *     None
  * Remarks:
  *     None                                                       
  ***********************************************************************************************************************************************************************************************************
*/
#define mINT0ClearIntFlag()				        (IFS0CLR = _IFS0_INT0IF_MASK)
#define mINT0GetIntFlag()				        (IFS0bits.INT0IF)
#define mINT0GetIntEnable()                     (IEC0bits.INT0IE)
#define mINT0IntEnable(enable)                  (IEC0CLR = _IEC0_INT0IE_MASK, IEC0SET = ((enable) << _IEC0_INT0IE_POSITION))
#define mINT0SetIntPriority(priority)		    (IPC0CLR = _IPC0_INT0IP_MASK, IPC0SET = ((priority) << _IPC0_INT0IP_POSITION))
#define mINT0GetIntPriority()				    (IPC0bits.INT0IP)
#define mINT0SetIntSubPriority(subPriority)     (IPC0CLR = _IPC0_INT0IS_MASK, IPC0SET = ((subPriority) << _IPC0_INT0IS_POSITION))
#define mINT0GetIntSubPriority()			    (IPC0bits.INT0IS)
#define mINT0SetEdgeMode(mode)		            (INTCONCLR = (1 << _INTCON_INT0EP_POSITION), INTCONSET = ((mode) << _INTCON_INT0EP_POSITION))
#define mINT0GetEdgeMode()				        (INTCONbits.INT0EP)
/*********************************************************************
 * External INT1 Control Functions
 ********************************************************************/
#define mINT1ClearIntFlag()				        (IFS0CLR = _IFS0_INT1IF_MASK)
#define mINT1GetIntFlag()				        (IFS0bits.INT1IF)
#define mINT1GetIntEnable()                     (IEC0bits.INT1IE)
#define mINT1IntEnable(enable)                  (IEC0CLR = _IEC0_INT1IE_MASK, IEC0SET = ((enable) << _IEC0_INT1IE_POSITION))
#define mINT1SetIntPriority(priority)		    (IPC1CLR = _IPC1_INT1IP_MASK, IPC1SET = ((priority) << _IPC1_INT1IP_POSITION))
#define mINT1GetIntPriority()				    (IPC1bits.INT1IP)
#define mINT1SetIntSubPriority(subPriority)     (IPC1CLR = _IPC1_INT1IS_MASK, IPC1SET = ((subPriority) << _IPC1_INT1IS_POSITION))
#define mINT1GetIntSubPriority()			    (IPC1bits.INT1IS)
#define mINT1SetEdgeMode(mode)		            (INTCONCLR = (1 << _INTCON_INT1EP_POSITION), INTCONSET = ((mode) << _INTCON_INT1EP_POSITION))
#define mINT1GetEdgeMode()				        (INTCONbits.INT1EP)
/*********************************************************************
 * External INT2 Control Functions
 ********************************************************************/
#define mINT2ClearIntFlag()				        (IFS0CLR = _IFS0_INT2IF_MASK)
#define mINT2GetIntFlag()				        (IFS0bits.INT2IF)
#define mINT2GetIntEnable()                     (IEC0bits.INT2IE)
#define mINT2IntEnable(enable)                  (IEC0CLR = _IEC0_INT2IE_MASK, IEC0SET = ((enable) << _IEC0_INT2IE_POSITION))
#define mINT2SetIntPriority(priority)		    (IPC2CLR = _IPC2_INT2IP_MASK, IPC2SET = ((priority) << _IPC2_INT2IP_POSITION))
#define mINT2GetIntPriority()				    (IPC2bits.INT2IP)
#define mINT2SetIntSubPriority(subPriority)     (IPC2CLR = _IPC2_INT2IS_MASK, IPC2SET = ((subPriority) << _IPC2_INT2IS_POSITION))
#define mINT2GetIntSubPriority()			    (IPC2bits.INT2IS)
#define mINT2SetEdgeMode(mode)		            (INTCONCLR = (1 << _INTCON_INT2EP_POSITION), INTCONSET = ((mode) << _INTCON_INT2EP_POSITION))
#define mINT2GetEdgeMode()				        (INTCONbits.INT2EP)
/*********************************************************************
 * External INT3 Control Functions
 ********************************************************************/
#define mINT3ClearIntFlag()				        (IFS0CLR = _IFS0_INT3IF_MASK)
#define mINT3GetIntFlag()				        (IFS0bits.INT3IF)
#define mINT3GetIntEnable()                     (IEC0bits.INT3IE)
#define mINT3IntEnable(enable)                  (IEC0CLR = _IEC0_INT3IE_MASK, IEC0SET = ((enable) << _IEC0_INT3IE_POSITION))
#define mINT3SetIntPriority(priority)		    (IPC3CLR = _IPC3_INT3IP_MASK, IPC3SET = ((priority) << _IPC3_INT3IP_POSITION))
#define mINT3GetIntPriority()				    (IPC3bits.INT3IP)
#define mINT3SetIntSubPriority(subPriority)     (IPC3CLR = _IPC3_INT3IS_MASK, IPC3SET = ((subPriority) << _IPC3_INT3IS_POSITION))
#define mINT3GetIntSubPriority()			    (IPC3bits.INT3IS)
#define mINT3SetEdgeMode(mode)		            (INTCONCLR = (1 << _INTCON_INT3EP_POSITION), INTCONSET = ((mode) << _INTCON_INT3EP_POSITION))
#define mINT3GetEdgeMode()				        (INTCONbits.INT3EP)
/*********************************************************************
 * External INT4 Control Functions
 ********************************************************************/
#define mINT4ClearIntFlag()				        (IFS0CLR = _IFS0_INT4IF_MASK)
#define mINT4GetIntFlag()				        (IFS0bits.INT4IF)
#define mINT4GetIntEnable()                     (IEC0bits.INT4IE)
#define mINT4IntEnable(enable)                  (IEC0CLR = _IEC0_INT4IE_MASK, IEC0SET = ((enable) << _IEC0_INT4IE_POSITION))
#define mINT4SetIntPriority(priority)		    (IPC4CLR = _IPC4_INT4IP_MASK, IPC4SET = ((priority) << _IPC4_INT4IP_POSITION))
#define mINT4GetIntPriority()				    (IPC4bits.INT4IP)
#define mINT4SetIntSubPriority(subPriority)     (IPC4CLR = _IPC4_INT4IS_MASK, IPC4SET = ((subPriority) << _IPC4_INT4IS_POSITION))
#define mINT4GetIntSubPriority()			    (IPC4bits.INT4IS)
#define mINT4SetEdgeMode(mode)		            (INTCONCLR = (1 << _INTCON_INT4EP_POSITION), INTCONSET = ((mode) << _INTCON_INT4EP_POSITION))
#define mINT4GetEdgeMode()				        (INTCONbits.INT4EP)

/******************************************************************************************************************************************
 * <combinewith mT1GetIntFlag, mT1GetIntEnable, mT1IntEnable, mT1SetIntPriority, mT1GetIntPriority, mT1SetIntSubPriority, mT1GetIntSubPriority,
 * mT2ClearIntFlag, mT2GetIntFlag, mT2GetIntEnable, mT2IntEnable, mT2SetIntPriority, mT2GetIntPriority, mT2SetIntSubPriority, mT2GetIntSubPriority,
 * mT3ClearIntFlag, mT3GetIntFlag, mT3GetIntEnable, mT3IntEnable, mT3SetIntPriority, mT3GetIntPriority, mT3SetIntSubPriority, mT3GetIntSubPriority,
 * mT23ClearIntFlag, mT23GetIntFlag, mT23GetIntEnable, mT23IntEnable, mT23SetIntPriority, mT23GetIntPriority, mT23SetIntSubPriority, mT23GetIntSubPriority,
 * mT4ClearIntFlag, mT4GetIntFlag, mT4GetIntEnable, mT4IntEnable, mT4SetIntPriority, mT4GetIntPriority, mT4SetIntSubPriority, mT4GetIntSubPriority,
 * mT5ClearIntFlag, mT5GetIntFlag, mT5GetIntEnable, mT5IntEnable, mT5SetIntPriority, mT5GetIntPriority, mT5SetIntSubPriority, mT5GetIntSubPriority,
 * mT45ClearIntFlag, mT45GetIntFlag, mT45GetIntEnable, mT45IntEnable, mT45SetIntPriority, mT45GetIntPriority, mT45SetIntSubPriority, mT45GetIntSubPriority>
 *
 * Timer x Interrupt Control Functions
 *
 * Function:        void    mTxClearIntFlag(void)
 *					int     mTxGetIntFlag(void)
 *					void    mTxSetIntEnable(void)
 *					int     mTxGetIntEnable(void)
 *                  void    mTxClearIntEnable(void)
 *					void    mTxSetIntPriority(int priority)
 *					int     mTxGetIntPriority(void)
 *					void    mTxSetIntSubPriority(subPriority)
 *					int     mTxGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *****************************************************************************************************************************************/
#ifdef _TMR1
#define mT1ClearIntFlag()                   (IFS0CLR = _IFS0_T1IF_MASK)
#define mT1GetIntFlag()                     (IFS0bits.T1IF)
#define mT1GetIntEnable()                   (IEC0bits.T1IE)
#define mT1IntEnable(enable)                (IEC0CLR = _IEC0_T1IE_MASK, IEC0SET = ((enable) << _IEC0_T1IE_POSITION))
#define mT1SetIntPriority(priority)         (IPC1CLR = _IPC1_T1IP_MASK, IPC1SET = ((priority) << _IPC1_T1IP_POSITION))
#define mT1GetIntPriority()                 (IPC1bits.T1IP)
#define mT1SetIntSubPriority(subPriority)   (IPC1CLR = _IPC1_T1IS_MASK, IPC1SET = ((subPriority) << _IPC1_T1IS_POSITION))
#define mT1GetIntSubPriority()              (IPC1bits.T1IS)
#endif
#ifdef _TMR2
/*********************************************************************
 * Timer 2 Interrupt Control Functions
 ********************************************************************/
#define mT2ClearIntFlag()                   (IFS0CLR = _IFS0_T2IF_MASK)
#define mT2GetIntFlag()                     (IFS0bits.T2IF)
#define mT2GetIntEnable()                   (IEC0bits.T2IE)
#define mT2IntEnable(enable)                (IEC0CLR = _IEC0_T2IE_MASK, IEC0SET = ((enable) << _IEC0_T2IE_POSITION))
#define mT2SetIntPriority(priority)         (IPC2CLR = _IPC2_T2IP_MASK, IPC2SET = ((priority) << _IPC2_T2IP_POSITION))
#define mT2GetIntPriority()                 (IPC2bits.T2IP)
#define mT2SetIntSubPriority(subPriority)   (IPC2CLR = _IPC2_T2IS_MASK, IPC2SET = ((subPriority) << _IPC2_T2IS_POSITION))
#define mT2GetIntSubPriority()              (IPC2bits.T2IS)
#endif
#ifdef _TMR3
/*********************************************************************
 * Timer 3 Interrupt Control Functions
 ********************************************************************/
#define mT3ClearIntFlag()                   (IFS0CLR = _IFS0_T3IF_MASK)
#define mT3GetIntFlag()                     (IFS0bits.T3IF)
#define mT3GetIntEnable()                   (IEC0bits.T3IE)
#define mT3IntEnable(enable)                (IEC0CLR = _IEC0_T3IE_MASK, IEC0SET = ((enable) << _IEC0_T3IE_POSITION))
#define mT3SetIntPriority(priority)         (IPC3CLR = _IPC3_T3IP_MASK, IPC3SET = ((priority) << _IPC3_T3IP_POSITION))
#define mT3GetIntPriority()                 (IPC3bits.T3IP)
#define mT3SetIntSubPriority(subPriority)   (IPC3CLR = _IPC3_T3IS_MASK, IPC3SET = ((subPriority) << _IPC3_T3IS_POSITION))
#define mT3GetIntSubPriority()              (IPC3bits.T3IS)
/*********************************************************************
 * Timer 23 Interrupt Control Functions
 ********************************************************************/
#define mT23ClearIntFlag()                   (IFS0CLR = _IFS0_T3IF_MASK)
#define mT23GetIntFlag()                     (IFS0bits.T3IF)
#define mT23GetIntEnable()                   (IEC0bits.T3IE)
#define mT23IntEnable(enable)                (IEC0CLR = _IEC0_T3IE_MASK, IEC0SET = ((enable) << _IEC0_T3IE_POSITION))
#define mT23SetIntPriority(priority)         (IPC3CLR = _IPC3_T3IP_MASK, IPC3SET = ((priority) << _IPC3_T3IP_POSITION))
#define mT23GetIntPriority()                 (IPC3bits.T3IP)
#define mT23SetIntSubPriority(subPriority)   (IPC3CLR = _IPC3_T3IS_MASK, IPC3SET = ((subPriority) << _IPC3_T3IS_POSITION))
#define mT23GetIntSubPriority()              (IPC3bits.T3IS)
#endif
#ifdef _TMR4
/*********************************************************************
 * Timer 4 Interrupt Control Functions
 ********************************************************************/
#define mT4ClearIntFlag()                   (IFS0CLR = _IFS0_T4IF_MASK)
#define mT4GetIntFlag()                     (IFS0bits.T4IF)
#define mT4GetIntEnable()                   (IEC0bits.T4IE)
#define mT4IntEnable(enable)                (IEC0CLR = _IEC0_T4IE_MASK, IEC0SET = ((enable) << _IEC0_T4IE_POSITION))
#define mT4SetIntPriority(priority)         (IPC4CLR = _IPC4_T4IP_MASK, IPC4SET = ((priority) << _IPC4_T4IP_POSITION))
#define mT4GetIntPriority()                 (IPC4bits.T4IP)
#define mT4SetIntSubPriority(subPriority)   (IPC4CLR = _IPC4_T4IS_MASK, IPC4SET = ((subPriority) << _IPC4_T4IS_POSITION))
#define mT4GetIntSubPriority()              (IPC4bits.T4IS)
#endif
#ifdef _TMR5
/*********************************************************************
 * Timer 5 Interrupt Control Functions
 ********************************************************************/
#define mT5ClearIntFlag()                   (IFS0CLR = _IFS0_T5IF_MASK)
#define mT5GetIntFlag()                     (IFS0bits.T5IF)
#define mT5GetIntEnable()                   (IEC0bits.T5IE)
#define mT5IntEnable(enable)                (IEC0CLR = _IEC0_T5IE_MASK, IEC0SET = ((enable) << _IEC0_T5IE_POSITION))
#define mT5SetIntPriority(priority)         (IPC5CLR = _IPC5_T5IP_MASK, IPC5SET = ((priority) << _IPC5_T5IP_POSITION))
#define mT5GetIntPriority()                 (IPC5bits.T5IP)
#define mT5SetIntSubPriority(subPriority)   (IPC5CLR = _IPC5_T5IS_MASK, IPC5SET = ((subPriority) << _IPC5_T5IS_POSITION))
#define mT5GetIntSubPriority()              (IPC5bits.T5IS)
/*********************************************************************
 * Timer 45 Interrupt Control Functions
 ********************************************************************/
#define mT45ClearIntFlag()                   (IFS0CLR = _IFS0_T5IF_MASK)
#define mT45GetIntFlag()                     (IFS0bits.T5IF)
#define mT45GetIntEnable()                   (IEC0bits.T5IE)
#define mT45IntEnable(enable)                (IEC0CLR = _IEC0_T5IE_MASK, IEC0SET = ((enable) << _IEC0_T5IE_POSITION))
#define mT45SetIntPriority(priority)         (IPC5CLR = _IPC5_T5IP_MASK, IPC5SET = ((priority) << _IPC5_T5IP_POSITION))
#define mT45GetIntPriority()                 (IPC5bits.T5IP)
#define mT45SetIntSubPriority(subPriority)   (IPC5CLR = _IPC5_T5IS_MASK, IPC5SET = ((subPriority) << _IPC5_T5IS_POSITION))
#define mT45GetIntSubPriority()              (IPC5bits.T5IS)
#endif
/******************************************************************************************************************************************
 * <combinewith mIC1GetIntFlag, mIC1GetIntEnable, mIC1IntEnable, mIC1EClearIntFlag, mIC1EGetIntFlag, mIC1EGetIntFlag, mIC1EIntEnable, mIC1SetIntPriority, mIC1GetIntPriority, mIC1SetIntSubPriority, mIC1GetIntSubPriority,
 * mIC2ClearIntFlag, mIC2GetIntFlag, mIC2GetIntEnable, mIC2IntEnable, mIC2EClearIntFlag, mIC2EGetIntFlag, mIC2EGetIntFlag, mIC2EIntEnable, mIC2SetIntPriority, mIC2GetIntPriority, mIC2SetIntSubPriority, mIC2GetIntSubPriority,
 * mIC3ClearIntFlag, mIC3GetIntFlag, mIC3GetIntEnable, mIC3IntEnable, mIC3EClearIntFlag, mIC3EGetIntFlag, mIC3EGetIntFlag, mIC3EIntEnable, mIC3SetIntPriority, mIC3GetIntPriority, mIC3SetIntSubPriority, mIC3GetIntSubPriority,
 * mIC4ClearIntFlag, mIC4GetIntFlag, mIC4GetIntEnable, mIC4IntEnable, mIC4EClearIntFlag, mIC4EGetIntFlag, mIC4EGetIntFlag, mIC4EIntEnable, mIC4SetIntPriority, mIC4GetIntPriority, mIC4SetIntSubPriority, mIC4GetIntSubPriority,
 * mIC5ClearIntFlag, mIC5GetIntFlag, mIC5GetIntEnable, mIC5IntEnable, mIC5EClearIntFlag, mIC5EGetIntFlag, mIC5EGetIntFlag, mIC5EIntEnable, mIC5SetIntPriority, mIC5GetIntPriority, mIC5SetIntSubPriority, mIC5GetIntSubPriority>
 * 
 * Input Capture x Interrupt Control Functions
 *
 * Function:        void    mICxClearIntFlag(void)
 *					int     mICxGetIntFlag(void)
 *					void    mICxSetIntEnable(void)
 *					int     mICxGetIntEnable(void)
 *                  void    mICxClearIntEnable(void)
 *                  void    mICxEClearIntFlag(void)    
 *                  int     mICxEGetIntFlag(void)      
 *                  int     mICxEGetIntEnable(void)    
 *                  void    mICxEIntEnable(int enable) 
 *					void    mICxSetIntPriority(int priority)
 *					int     mICxGetIntPriority(void)
 *					void    mICxSetIntSubPriority(subPriority)
 *					int     mICxGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *****************************************************************************************************************************************/
/*********************************************************************
 * Input Capture 1 Interrupt Control Functions
 ********************************************************************/
#ifdef _ICAP1
#define mIC1ClearIntFlag()                  (IFS0CLR = _IFS0_IC1IF_MASK)
#define mIC1GetIntFlag()                    (IFS0bits.IC1IF)
#define mIC1GetIntEnable()                  (IEC0bits.IC1IE)
#define mIC1IntEnable(enable)               (IEC0CLR = _IEC0_IC1IE_MASK, IEC0SET = ((enable) << _IEC0_IC1IE_POSITION))
#define mIC1EClearIntFlag()                 (IFS0CLR = _IFS0_IC1EIF_MASK)
#define mIC1EGetIntFlag()                   (IFS0bits.IC1EIF)
#define mIC1EGetIntEnable()                 (IEC0bits.IC1EIE)
#define mIC1EIntEnable(enable)              (IEC0CLR = _IEC0_IC1EIE_MASK, IEC0SET = ((enable) << _IEC0_IC1EIE_POSITION))
#define mIC1SetIntPriority(priority)        (IPC1CLR = _IPC1_IC1IP_MASK, IPC1SET = ((priority) << _IPC1_IC1IP_POSITION))
#define mIC1GetIntPriority()                (IPC1bits.IC1IP)
#define mIC1SetIntSubPriority(subPriority)  (IPC1CLR = _IPC1_IC1IS_MASK, IPC1SET = ((subPriority) << _IPC1_IC1IS_POSITION))
#define mIC1GetIntSubPriority()             (IPC1bits.IC1IS)
#endif
#ifdef _ICAP2
/*********************************************************************
 * Input Capture 2 Interrupt Control Functions
 ********************************************************************/
#define mIC2ClearIntFlag()                  (IFS0CLR = _IFS0_IC2IF_MASK)
#define mIC2GetIntFlag()                    (IFS0bits.IC2IF)
#define mIC2GetIntEnable()                  (IEC0bits.IC2IE)
#define mIC2IntEnable(enable)               (IEC0CLR = _IEC0_IC2IE_MASK, IEC0SET = ((enable) << _IEC0_IC2IE_POSITION))
#define mIC2EClearIntFlag()                 (IFS0CLR = _IFS1_IC2EIF_MASK)
#define mIC2EGetIntFlag()                   (IFS0bits.IC2EIF)
#define mIC2EGetIntEnable()                 (IEC0bits.IC2EIE)
#define mIC2EIntEnable(enable)              (IEC0CLR = _IEC1_IC2EIE_MASK, IEC1SET = ((enable) << _IEC0_IC2EIE_POSITION))
#define mIC2SetIntPriority(priority)        (IPC2CLR = _IPC2_IC2IS_MASK, IPC2SET = ((priority) << _IPC2_IC2IP_POSITION))
#define mIC2GetIntPriority()                (IPC2bits.IC2IP)
#define mIC2SetIntSubPriority(subPriority)  (IPC2CLR = _IPC2_IC2IS_MASK, IPC2SET = ((subPriority) << _IPC2_IC2IS_POSITION))
#define mIC2GetIntSubPriority()             (IPC2bits.IC2IS)
#endif
#ifdef _ICAP3
/*********************************************************************
 * Input Capture 3 Interrupt Control Functions
 ********************************************************************/
#define mIC3ClearIntFlag()                  (IFS0CLR = _IFS0_IC3IF_MASK)
#define mIC3GetIntFlag()                    (IFS0bits.IC3IF)
#define mIC3GetIntEnable()                  (IEC0bits.IC3IE)
#define mIC3IntEnable(enable)               (IEC0CLR = _IEC0_IC3IE_MASK, IEC0SET = ((enable) << _IEC0_IC3IE_POSITION))
#define mIC3EClearIntFlag()                 (IFS0CLR = _IFS1_IC3EIF_MASK)
#define mIC3EGetIntFlag()                   (IFS0bits.IC3EIF)
#define mIC3EGetIntEnable()                 (IEC0bits.IC3EIE)
#define mIC3EIntEnable(enable)              (IEC0CLR = _IEC1_IC3EIE_MASK, IEC0SET = ((enable) << _IEC0_IC3EIE_POSITION))
#define mIC3SetIntPriority(priority)        (IPC3CLR = _IPC3_IC3IS_MASK, IPC3SET = ((priority) << _IPC3_IC3IP_POSITION))
#define mIC3GetIntPriority()                (IPC3bits.IC3IP)
#define mIC3SetIntSubPriority(subPriority)  (IPC3CLR = _IPC3_IC3IS_MASK, IPC3SET = ((subPriority) << _IPC3_IC3IS_POSITION))
#define mIC3GetIntSubPriority()             (IPC3bits.IC3IS)
#endif
#ifdef _ICAP4
/*********************************************************************
 * Input Capture 4 Interrupt Control Functions
 ********************************************************************/
#define mIC4ClearIntFlag()                  (IFS0CLR = _IFS0_IC4IF_MASK)
#define mIC4GetIntFlag()                    (IFS0bits.IC4IF)
#define mIC4GetIntEnable()                  (IEC0bits.IC4IE)
#define mIC4IntEnable(enable)               (IEC0CLR = _IEC0_IC4IE_MASK, IEC0SET = ((enable) << _IEC0_IC4IE_POSITION))
#define mIC4EClearIntFlag()                 (IFS0CLR = _IFS2_IC4EIF_MASK)
#define mIC4EGetIntFlag()                   (IFS0bits.IC4EIF)
#define mIC4EGetIntEnable()                 (IEC0bits.IC4EIE)
#define mIC4EIntEnable(enable)              (IEC0CLR = _IEC0_IC4EIE_MASK, IEC0SET = ((enable) << _IEC0_IC4EIE_POSITION))
#define mIC4SetIntPriority(priority)        (IPC4CLR = _IPC4_IC4IP_MASK, IPC4SET = ((priority) << _IPC4_IC4IP_POSITION))
#define mIC4GetIntPriority()                (IPC4bits.IC4IP)
#define mIC4SetIntSubPriority(subPriority)  (IPC4CLR = _IPC4_IC4IS_MASK, IPC4SET = ((subPriority) << _IPC4_IC4IS_POSITION))
#define mIC4GetIntSubPriority()             (IPC4bits.IC4IS)
#endif
#ifdef _ICAP5
/*********************************************************************
 * Input Capture 5 Interrupt Control Functions
 ********************************************************************/
#define mIC5ClearIntFlag()                  (IFS0CLR = _IFS0_IC5IF_MASK)
#define mIC5GetIntFlag()                    (IFS0bits.IC5IF)
#define mIC5GetIntEnable()                  (IEC0bits.IC5IE)
#define mIC5IntEnable(enable)               (IEC0CLR = _IEC0_IC5IE_MASK, IEC0SET = ((enable) << _IEC0_IC5IE_POSITION))
#define mIC5EClearIntFlag()                 (IFS0CLR = _IFS2_IC5EIF_MASK)
#define mIC5EGetIntFlag()                   (IFS0bits.IC5EIF)
#define mIC5EGetIntEnable()                 (IEC0bits.IC5EIE)
#define mIC5EIntEnable(enable)              (IEC0CLR = _IEC0_IC5EIE_MASK, IEC0SET = ((enable) << _IEC0_IC5EIE_POSITION))
#define mIC5SetIntPriority(priority)        (IPC5CLR = _IPC5_IC5IP_MASK, IPC5SET = ((priority) << _IPC5_IC5IP_POSITION))
#define mIC5GetIntPriority()                (IPC5bits.IC5IP)
#define mIC5SetIntSubPriority(subPriority)  (IPC5CLR = _IPC5_IC5IS_MASK, IPC5SET = ((subPriority) << _IPC5_IC5IS_POSITION))
#define mIC5GetIntSubPriority()             (IPC5bits.IC5IS)
#endif

/******************************************************************************************************************************************
 * <combinewith mOC1GetIntFlag, mOC1GetIntEnable, mOC1IntEnable, mOC1SetIntPriority, mOC1GetIntPriority, mOC1SetIntSubPriority, mOC1GetIntSubPriority,
 * mOC2ClearIntFlag, mOC2GetIntFlag, mOC2GetIntEnable, mOC2IntEnable, mOC2SetIntPriority, mOC2GetIntPriority, mOC2SetIntSubPriority, mOC2GetIntSubPriority,
 * mOC3ClearIntFlag, mOC3GetIntFlag, mOC3GetIntEnable, mOC3IntEnable, mOC3SetIntPriority, mOC3GetIntPriority, mOC3SetIntSubPriority, mOC3GetIntSubPriority,
 * mOC4ClearIntFlag, mOC4GetIntFlag, mOC4GetIntEnable, mOC4IntEnable, mOC4SetIntPriority, mOC4GetIntPriority, mOC4SetIntSubPriority, mOC4GetIntSubPriority,
 * mOC5ClearIntFlag, mOC5GetIntFlag, mOC5GetIntEnable, mOC5IntEnable, mOC5SetIntPriority, mOC5GetIntPriority, mOC5SetIntSubPriority, mOC5GetIntSubPriority>
 * 
 * Output Compare x Interrupt Control Functions
 *
 * Function:        void    mOCxClearIntFlag(void)
 *					int     mOCxGetIntFlag(void)
 *					void    mOCxSetIntEnable(void)
 *					int     mOCxGetIntEnable(void)
 *                  void    mOCxClearIntEnable(void)
 *					void    mOCxSetIntPriority(int priority)
 *					int     mOCxGetIntPriority(void)
 *					void    mOCxSetIntSubPriority(subPriority)
 *					int     mOCxGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *****************************************************************************************************************************************/
#ifdef _OCMP1
#define mOC1ClearIntFlag()                  (IFS0CLR = _IFS0_OC1IF_MASK)
#define mOC1GetIntFlag()                    (IFS0bits.OC1IF)
#define mOC1GetIntEnable()                  (IEC0bits.OC1IE)
#define mOC1IntEnable(enable)               (IEC0CLR = _IEC0_OC1IE_MASK, IEC0SET = ((enable) << _IEC0_OC1IE_POSITION))
#define mOC1SetIntPriority(priority)        (IPC1CLR = _IPC1_IC1IP_MASK, IPC1SET = ((priority) << _IPC1_OC1IP_POSITION))
#define mOC1GetIntPriority()                (IPC1bits.OC1IP)
#define mOC1SetIntSubPriority(subPriority)  (IPC1CLR = _IPC1_OC1IS_MASK, IPC1SET = ((subPriority) << _IPC1_OC1IS_POSITION))
#define mOC1GetIntSubPriority()             (IPC1bits.OC1IS)
#endif
#ifdef _OCMP2
/*********************************************************************
 * Output Compare 2 Interrupt Control Functions
 ********************************************************************/
#define mOC2ClearIntFlag()                  (IFS0CLR = _IFS0_OC2IF_MASK)
#define mOC2GetIntFlag()                    (IFS0bits.OC2IF)
#define mOC2GetIntEnable()                  (IEC0bits.OC2IE)
#define mOC2IntEnable(enable)               (IEC0CLR = _IEC0_OC2IE_MASK, IEC0SET = ((enable) << _IEC0_OC2IE_POSITION))
#define mOC2SetIntPriority(priority)        (IPC2CLR = _IPC2_OC2IP_MASK, IPC2SET = ((priority) << _IPC2_OC2IP_POSITION))
#define mOC2GetIntPriority()                (IPC2bits.OC2IP)
#define mOC2SetIntSubPriority(subPriority)  (IPC2CLR = _IPC2_OC2IS_MASK, IPC2SET = ((subPriority) << _IPC2_OC2IS_POSITION))
#define mOC2GetIntSubPriority()             (IPC2bits.OC2IS)
#endif
#ifdef _OCMP3
/*********************************************************************
 * Output Compare 3 Interrupt Control Functions
 ********************************************************************/
#define mOC3ClearIntFlag()                  (IFS0CLR = _IFS0_OC3IF_MASK)
#define mOC3GetIntFlag()                    (IFS0bits.OC3IF)
#define mOC3GetIntEnable()                  (IEC0bits.OC3IE)
#define mOC3IntEnable(enable)               (IEC0CLR = _IEC0_OC3IE_MASK, IEC0SET = ((enable) << _IEC0_OC3IE_POSITION))
#define mOC3SetIntPriority(priority)        (IPC3CLR = _IPC3_OC3IP_MASK, IPC3SET = ((priority) << _IPC3_OC3IP_POSITION))
#define mOC3GetIntPriority()                (IPC3bits.OC3IP)
#define mOC3SetIntSubPriority(subPriority)  (IPC3CLR = _IPC3_OC3IS_MASK, IPC3SET = ((subPriority) << _IPC3_OC3IS_POSITION))
#define mOC3GetIntSubPriority()             (IPC3bits.OC3IS)
#endif
#ifdef _OCMP4
/*********************************************************************
 * Output Compare 4 Interrupt Control Functions
 ********************************************************************/
#define mOC4ClearIntFlag()                  (IFS0CLR = _IFS0_OC4IF_MASK)
#define mOC4GetIntFlag()                    (IFS0bits.OC4IF)
#define mOC4GetIntEnable()                  (IEC0bits.OC4IE)
#define mOC4IntEnable(enable)               (IEC0CLR = _IEC0_OC4IE_MASK, IEC0SET = ((enable) << _IEC0_OC4IE_POSITION))
#define mOC4SetIntPriority(priority)        (IPC4CLR = _IPC4_OC4IP_MASK, IPC4SET = ((priority) << _IPC4_OC4IP_POSITION))
#define mOC4GetIntPriority()                (IPC4bits.OC4IP)
#define mOC4SetIntSubPriority(subPriority)  (IPC4CLR = _IPC4_OC4IS_MASK, IPC4SET = ((subPriority) << _IPC4_OC4IS_POSITION))
#define mOC4GetIntSubPriority()             (IPC4bits.OC4IS)
#endif
#ifdef _OCMP5
/*********************************************************************
 * Output Compare 5 Interrupt Control Functions
 ********************************************************************/
#define mOC5ClearIntFlag()                  (IFS0CLR = _IFS0_OC5IF_MASK)
#define mOC5GetIntFlag()                    (IFS0bits.OC5IF)
#define mOC5GetIntEnable()                  (IEC0bits.OC5IE)
#define mOC5IntEnable(enable)               (IEC0CLR = _IEC0_OC5IE_MASK, IEC0SET = ((enable) << _IEC0_OC5IE_POSITION))
#define mOC5SetIntPriority(priority)        (IPC5CLR = _IPC5_OC5IP_MASK, IPC5SET = ((priority) << _IPC5_OC5IP_POSITION))
#define mOC5GetIntPriority()                (IPC5bits.OC5IP)
#define mOC5SetIntSubPriority(subPriority)  (IPC5CLR = _IPC5_OC5IS_MASK, IPC5SET = ((subPriority) << _IPC5_OC5IS_POSITION))
#define mOC5GetIntSubPriority()             (IPC5bits.OC5IS)
#endif

/*****************************************************************************************
  * <combinewith mCNGetIntEnable, mCNGetIntFlag, mCNGetIntPriority, mCNGetIntSubPriority, 
  * mCNIntEnable, mCNSetIntPriority, mCNSetIntSubPriority, mCNGetIntSubPriority>
  * 
  * Change Notice Interrupt Control Functions
  * Prototype:
  *     void mCNClearIntFlag(void) int mCNGetIntFlag(void) void
  *     mCNSetIntEnable(void) int mCNGetIntEnable(void) void
  *     mCNClearIntEnable(void) void mCNSetIntPriority(int priority)
  *     int mCNGetIntPriority(void) void
  *     mCNSetIntSubPriority(subPriority) int
  *     mCNGetIntSubPriority(void)
  * Preconditions:
  *     EBASE and IntCtl.VS set up
  * Input:
  *     priority :     A value between 0 \- 7 inclusive.
  *     subPriority :  A value between 0 \- 3 inclusive
  * Return:
  *     For 'Get' functions, the returned values are of
  *     same type as 'Set' functions.
  *     
  * Side Effects:
  *     None
  * Remarks:
  *     None                                                                                                                                                          
  ***************************************************************************************
*/

#define mCNAClearIntFlag()                   (IFS1CLR = _IFS1_CNAIF_MASK)
#define mCNAGetIntFlag()                     (IFS1bits.CNAIF)
#define mCNAGetIntEnable()                   (IEC1bits.CNAIE)
#define mCNAIntEnable(enable)                (IEC1CLR = _IEC1_CNAIE_MASK, IEC1SET = ((enable) << _IEC1_CNAIE_POSITION))

#define mCNBClearIntFlag()                   (IFS1CLR = _IFS1_CNBIF_MASK)
#define mCNBGetIntFlag()                     (IFS1bits.CNBIF)
#define mCNBGetIntEnable()                   (IEC1bits.CNBIE)
#define mCNBIntEnable(enable)                (IEC1CLR = _IEC1_CNBIE_MASK, IEC1SET = ((enable) << _IEC1_CNBIE_POSITION))

#define mCNCClearIntFlag()                   (IFS1CLR = _IFS1_CNCIF_MASK)
#define mCNCGetIntFlag()                     (IFS1bits.CNCIF)
#define mCNCGetIntEnable()                   (IEC1bits.CNCIE)
#define mCNCIntEnable(enable)                (IEC1CLR = _IEC1_CNCIE_MASK, IEC1SET = ((enable) << _IEC1_CNCIE_POSITION))

#define mCNSetIntPriority(priority)         (IPC8CLR = _IPC8_CNIP_MASK, IPC6SET = ((priority) << _IPC8_CNIP_POSITION))
#define mCNGetIntPriority()                 (IPC8bits.CNIP)
#define mCNSetIntSubPriority(subPriority)   (IPC8CLR = _IPC8_CNIS_MASK, IPC6SET = ((subPriority) << _IPC8_CNIS_POSITION))
#define mCNGetIntSubPriority()              (IPC8bits.CNIS)



/******************************************************************************************************************************************
 * <combinewith mSPI1EGetIntFlag, mSPI1EGetIntEnable, mSPI1EIntEnable, mSPI1TXClearIntFlag, mSPI1TXGetIntFlag, mSPI1TXGetIntEnable, mSPI1TXIntEnable, 
 * mSPI1RXClearIntFlag, mSPI1RXGetIntFlag, mSPI1RXGetIntEnable, mSPI1RXIntEnable, mSPI1ClearAllIntFlags, mSPI1IntDisable, mSPI1SetIntEnable, 
 * mSPI1SetIntPriority, mSPI1GetIntPriority, mSPI1SetIntSubPriority, mSPI1GetIntSubPriority,
 * mSPI2EClearIntFlag, mSPI2EGetIntFlag, mSPI2EGetIntEnable, mSPI2EIntEnable, mSPI2TXClearIntFlag, mSPI2TXGetIntFlag, mSPI2TXGetIntEnable, mSPI2TXIntEnable, 
 * mSPI2RXClearIntFlag, mSPI2RXGetIntFlag, mSPI2RXGetIntEnable, mSPI2RXIntEnable, mSPI2ClearAllIntFlags, mSPI2IntDisable, mSPI2SetIntEnable, 
 * mSPI2SetIntPriority, mSPI2GetIntPriority, mSPI2SetIntSubPriority, mSPI2GetIntSubPriority>
 *
 * SPI x Interrupt Control Functions
 *
 * Function:        void    mSPIxEClearIntFlag(void)
 *					int     mSPIxEGetIntFlag(void)
 *					void    mSPIxESetIntEnable(void)
 *					int     mSPIxEGetIntEnable(void)
 *                  void    mSPIxEClearIntEnable(void)
 *                  void    mSPIxTXClearIntFlag(void)
 *					int     mSPIxTXGetIntFlag(void)
 *					void    mSPIxTXSetIntEnable(void)
 *					int     mSPIxTXGetIntEnable(void)
 *                  void    mSPIxTXClearIntEnable(void)
 *                  void    mSPIxRXClearIntFlag(void)
 *					int     mSPIxRXGetIntFlag(void)
 *					void    mSPIxRXSetIntEnable(void)
 *					int     mSPIxRXGetIntEnable(void)
 *                  void    mSPIxRXClearIntEnable(void)
 *					void    mSSPIxetPriority(int priority)
 *					int     mSPIxGetIntPriority(void)
 *					void    mSPIxSetIntSubPriority(subPriority)
 *					int     mSPIxGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *****************************************************************************************************************************************/
#ifdef _SPI1
#define mSPI1EClearIntFlag()                (IFS1CLR = _IFS1_SPI1EIF_MASK)
#define mSPI1EGetIntFlag()                  (IFS1bits.SPI1EIF)
#define mSPI1EGetIntEnable()                (IEC1bits.SPI1EIE)
#define mSPI1EIntEnable(enable)             (IEC1CLR = _IEC1_SPI1EIE_MASK, IEC1SET = ((enable) << _IEC1_SPI1EIE_POSITION))
/*********************************************************************
 * SPI 1 Transfer
 ********************************************************************/
#define mSPI1TXClearIntFlag()               (IFS1CLR = _IFS1_SPI1TXIF_MASK)
#define mSPI1TXGetIntFlag()                 (IFS1bits.SPI1TXIF)
#define mSPI1TXGetIntEnable()               (IEC1bits.SPI1TXIE)
#define mSPI1TXIntEnable(enable)             (IEC1CLR = _IEC1_SPI1TXIE_MASK, IEC1SET = ((enable) << _IEC1_SPI1TXIE_POSITION))
/*********************************************************************
 * SPI 1 Receive
 ********************************************************************/
#define mSPI1RXClearIntFlag()               (IFS1CLR = _IFS1_SPI1RXIF_MASK)
#define mSPI1RXGetIntFlag()                 (IFS1bits.SPI1RXIF)
#define mSPI1RXGetIntEnable()               (IEC1bits.SPI1RXIE)
#define mSPI1RXIntEnable(enable)             (IEC1CLR = _IEC1_SPI1RXIE_MASK, IEC1SET = ((enable) << _IEC1_SPI1RXIE_POSITION))
/*********************************************************************
 * SPI 1 Vector
 ********************************************************************/
#define mSPI1ClearAllIntFlags()              ( IFS1CLR = (_IFS1_SPI1EIF_MASK | _IFS1_SPI1TXIF_MASK | _IFS1_SPI1RXIF_MASK))
#define mSPI1IntDisable()                    ( IEC1CLR = (_IEC1_SPI1EIE_MASK | _IEC1_SPI1TXIE_MASK | _IEC1_SPI1RXIE_MASK))
#define mSPI1SetIntEnable(flags)			 ( IEC1SET = (((flags) & 0x7) << _IEC1_SPI1EIE_POSITION))
#define mSPI1SetIntPriority(priority)        (IPC7CLR = _IPC7_SPI1IP_MASK, IPC7SET = ((priority) << _IPC7_SPI1IP_POSITION))
#define mSPI1GetIntPriority()                (IPC7bits.SPI1IP)
#define mSPI1SetIntSubPriority(subPriority)  (IPC7CLR = _IPC7_SPI1IS_MASK, IPC7SET = ((subPriority) << _IPC7_SPI1IS_POSITION))
#define mSPI1GetIntSubPriority()             (IPC7bits.SPI1IS)
#endif
/*********************************************************************
 * <combinewith mI2C1BGetIntFlag, mI2C1BGetIntEnable, mI2C1BIntEnable, 
 * mI2C1SClearIntFlag, mI2C1SGetIntFlag, mI2C1SGetIntEnable, mI2C1SIntEnable, mI2C1MClearIntFlag,
 * mI2C1MGetIntFlag, mI2C1MGetIntEnable, mI2C1MIntEnable, mI2C1ClearAllIntFlags, mI2C1IntDisable,
 * mI2C1SetIntPriority, mI2C1GetIntPriority, mI2C1SetIntSubPriority, mI2C1GetIntSubPriority,
 * mI2C2BClearIntFlag, mI2C2BGetIntFlag, mI2C2BGetIntEnable, mI2C2BIntEnable, 
 * mI2C2SClearIntFlag, mI2C2SGetIntFlag, mI2C2SGetIntEnable, mI2C2SIntEnable, mI2C2MClearIntFlag,
 * mI2C2MGetIntFlag, mI2C2MGetIntEnable, mI2C2MIntEnable, mI2C2ClearAllIntFlags, mI2C2IntDisable,
 * mI2C2SetIntPriority, mI2C2GetIntPriority, mI2C2SetIntSubPriority, mI2C2GetIntSubPriority>
 *
 * I2C x Interrupt Control Functions
 *
 * Function:        void    mI2CxBClearIntFlag(void)
 *					int     mI2CxBGetIntFlag(void)
 *					void    mI2CxBSetIntEnable(void)
 *					int     mI2CxBGetIntEnable(void)
 *                  void    mI2CxBClearIntEnable(void)
 *                  void    mI2CxSClearIntFlag(void)
 *					int     mI2CxSGetIntFlag(void)
 *					void    mI2CxSSetIntEnable(void)
 *					int     mI2CxSGetIntEnable(void)
 *                  void    mI2CxSClearIntEnable(void)
 *                  void    mI2CxMClearIntFlag(void)
 *					int     mI2CxMGetIntFlag(void)
 *					void    mI2CxMSetIntEnable(void)
 *					int     mI2CxMGetIntEnable(void)
 *                  void    mI2CxMClearIntEnable(void)
 *					void    mI2CxSetIntPriority(int priority)
 *					int     mI2CxGetIntPriority(void)
 *					void    mI2CxSetIntSubPriority(subPriority)
 *					int     mI2CxGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 ********************************************************************/
#ifdef _I2C1
#define mI2C1BClearIntFlag()                (IFS1CLR = _IFS0_I2C1BIF_MASK)
#define mI2C1BGetIntFlag()                  (IFS1bits.I2C1BIF)
#define mI2C1BGetIntEnable()                (IEC1bits.I2C1BIE)
#define mI2C1BIntEnable(enable)             (IEC1CLR = _IEC0_I2C1BIE_MASK, IEC1SET = ((enable) << _IEC1_I2C1BIE_POSITION))
/*********************************************************************
 * I2C 1 Slave
 ********************************************************************/
#define mI2C1SClearIntFlag()                (IFS1CLR = _IFS1_I2C1SIF_MASK)
#define mI2C1SGetIntFlag()                  (IFS1bits.I2C1SIF)
#define mI2C1SGetIntEnable()                (IEC1bits.I2C1SIE)
#define mI2C1SIntEnable(enable)             (IEC1CLR = _IEC1_I2C1SIE_MASK, IEC1SET = ((enable) << _IEC1_I2C1SIE_POSITION))
/*********************************************************************
 * I2C 1 Master
 ********************************************************************/
#define mI2C1MClearIntFlag()                (IFS1CLR = _IFS1_I2C1MIF_MASK)
#define mI2C1MGetIntFlag()                  (IFS1bits.I2C1MIF)
#define mI2C1MGetIntEnable()                (IEC1bits.I2C1MIE)
#define mI2C1MIntEnable(enable)             (IEC1CLR = _IEC1_I2C1MIE_MASK, IEC1SET = ((enable) << _IEC1_I2C1MIE_POSITION))
/*********************************************************************
 * I2C 1 Vector
 ********************************************************************/
#define mI2C1ClearAllIntFlags()             ( IFS1CLR = (_IFS1_I2C1BIF_MASK | _IFS1_I2C1SIF_MASK | _IFS1_I2C1MIF_MASK))
#define mI2C1IntDisable()                   ( IEC1CLR = (_IEC1_I2C1BIE_MASK | _IEC1_I2C1SIE_MASK | _IEC1_I2C1MIE_MASK))
#define mI2C1SetIntEnable(flags)			 ( IEC1SET = (((flags) & 0x7) << _IEC1_I2C1BIE_POSITION))
#define mI2C1SetIntPriority(priority)       ( IPC8CLR = _IPC8_I2C1IP_MASK, IPC8SET = ((priority) << _IPC8_I2C1IP_POSITION))
#define mI2C1GetIntPriority()               (IPC8bits.I2C1IS)
#define mI2C1SetIntSubPriority(subPriority) ( IPC8CLR = _IPC8_I2C1IS_MASK, IPC8SET = ((subPriority) << _IPC8_I2C1IS_POSITION))
#define mI2C1GetIntSubPriority()            (IPC8bits.I2C1IS)
#endif
#ifdef _I2C2
/*********************************************************************
 * I2C 2 Interrupt Control Functions
 ********************************************************************/
/*********************************************************************
 * I2C 2 Bus Colision
 ********************************************************************/
#define mI2C2BClearIntFlag()                (IFS1CLR = _IFS1_I2C2BIF_MASK)
#define mI2C2BGetIntFlag()                  (IFS1bits.I2C2BIF)
#define mI2C2BGetIntEnable()                (IEC1bits.I2C2BIE)
#define mI2C2BIntEnable(enable)             (IEC1CLR = _IEC1_I2C2BIE_MASK, IEC1SET = ((enable) << _IEC1_I2C2BIE_POSITION))
/*********************************************************************
 * I2C 2 Slave
 ********************************************************************/
#define mI2C2SClearIntFlag()                (IFS1CLR = _IFS1_I2C2SIF_MASK)
#define mI2C2SGetIntFlag()                  (IFS1bits.I2C2SIF)
#define mI2C2SGetIntEnable()                (IEC1bits.I2C2SIE)
#define mI2C2SIntEnable(enable)             (IEC1CLR = _IEC1_I2C2SIE_MASK, IEC1SET = ((enable) << _IEC1_I2C2SIE_POSITION))
/*********************************************************************
 * I2C 2 Master
 ********************************************************************/
#define mI2C2MClearIntFlag()                (IFS1CLR = _IFS1_I2C2MIF_MASK)
#define mI2C2MGetIntFlag()                  (IFS1bits.I2C2MIF)
#define mI2C2MGetIntEnable()                (IEC1bits.I2C2MIE)
#define mI2C2MIntEnable(enable)             (IEC1CLR = _IEC1_I2C2MIE_MASK, IEC1SET = ((enable) << _IEC1_I2C2MIE_POSITION))
/*********************************************************************
 * I2C 2 Vector
 ********************************************************************/
#define mI2C2ClearAllIntFlags()             ( IFS1CLR = (_IFS1_I2C2BIF_MASK | _IFS1_I2C2SIF_MASK | _IFS1_I2C2MIF_MASK))
#define mI2C2IntDisable()                   ( IEC1CLR = (_IEC1_I2C2BIE_MASK | _IEC1_I2C2SIE_MASK | _IEC1_I2C2MIE_MASK))
#define mI2C2SetIntEnable(flags)			( IEC1SET = (((flags) & 0x7) << _IEC1_I2C2BIE_POSITION))
#define mI2C2SetIntPriority(priority)       ( IPC9CLR = _IPC9_I2C2IP_MASK, IPC9SET = ((priority) << _IPC9_I2C2IP_POSITION))
#define mI2C2GetIntPriority()               ( IPC8bits.I2C2IS)
#define mI2C2SetIntSubPriority(subPriority) ( IPC9CLR = _IPC9_I2C2IS_MASK, IPC9SET = ((subPriority) << _IPC9_I2C2IS_POSITION))
#define mI2C2GetIntSubPriority()            ( IPC9bits.I2C2IS)
#endif
/*******************************************************************************************************
  * <combinewith mAD1GetIntFlag, mAD1GetIntEnable, mAD1IntEnable, mAD1SetIntPriority, 
  * mAD1GetIntPriority, mAD1SetIntSubPriority, mAD1GetIntSubPriority>
  * 
  * Analog to Digital 1 Interrupt Control Functions
  * Prototype:
  *     void mAD1ClearIntFlag(void) int mAD1GetIntFlag(void) void
  *     mAD1SetIntEnable(void) int mAD1GetIntEnable(void) void
  *     mAD1ClearIntEnable(void) void mAD1SetIntPriority(int
  *     priority) int mAD1GetIntPriority(void) void
  *     mAD1SetIntSubPriority(subPriority) int
  *     mGAD1etSubPriority(void)
  * Preconditions:
  *     EBASE and IntCtl.VS set up
  * Input:
  *     priority :     A value between 0 \- 7 inclusive.
  *     subPriority :  A value between 0 \- 3 inclusive
  * Return:
  *     For 'Get' functions, the returned values are of
  *     same type as 'Set' functions.
  *     
  * Side Effects:
  *     None
  * Remarks:
  *     None                                                                                                                                           
  ******************************************************************************************************
*/
#ifdef _ADC10
#define mAD1ClearIntFlag()                  (IFS0CLR = _IFS0_AD1IF_MASK )
#define mAD1GetIntFlag()                    (IFS0bits.AD1IF)
#define mAD1GetIntEnable()                  (IEC1bits.AD1IE)
#define mAD1IntEnable(enable)               ( IEC0CLR = _IEC0_AD1IE_MASK, IEC0SET = ((enable) << _IEC0_AD1IE_POSITION))
#define mAD1SetIntPriority(priority)        ( IPC5CLR = _IPC5_AD1IP_MASK, IPC5SET = ((priority) << _IPC5_AD1IP_POSITION))
#define mAD1GetIntPriority()                (IPC5bits.AD1IP)
#define mAD1SetIntSubPriority(subPriority)  ( IPC5CLR = _IPC5_AD1IS_MASK, IPC5SET = ((subPriority) << _IPC5_AD1IS_POSITION))
#define mAD1GetIntSubPriority()             (IPC5bits.AD1IS)
#endif
/*********************************************************************
 * <combinewith mPMPGetIntEnable, mPMPGetIntFlag, mPMPGetIntPriority, mPMPGetIntSubPriority, 
 * mPMPIntEnable, mPMPSetIntPriority, mPMPSetIntSubPriority>
 *
 * Parallel master Port Interrupt Control Functions
 *
 * Function:        void    mPMPClearIntFlag(void)
 *					int     mPMPGetIntFlag(void)
 *					void    mPMPSetIntEnable(void)
 *					int     mPMPGetIntEnable(void)
 *                  void    mPMPClearIntEnable(void)
 *					void    mPMPSetIntPriority(int priority)
 *					int     mPMPGetIntPriority(void)
 *					void    mPMPSetIntSubPriority(subPriority)
 *					int     mPMPGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 ********************************************************************/
#ifdef _PMP
#define mPMPClearIntFlag()                  (IFS1CLR = _IFS1_PMPIF_MASK)
#define mPMPGetIntFlag()                    (IFS1bits.PMPIF)
#define mPMPGetIntEnable()                  (IEC1bits.PMPIE)
#define mPMPIntEnable(enable)               ( IEC1CLR = _IEC1_PMPIE_MASK, IEC1SET = ((enable) << _IEC1_PMPIE_POSITION))
#define mPMPEClearIntFlag()                  (IFS1CLR = _IFS1_PMPEIF_MASK)
#define mPMPEGetIntFlag()                    (IFS1bits.PMPEIF)
#define mPMPEGetIntEnable()                  (IEC1bits.PMPEIE)
#define mPMPEIntEnable(enable)               ( IEC1CLR = _IEC1_PMPEIE_MASK, IEC1SET = ((enable) << _IEC1_PMPEIE_POSITION))
#define mPMPSetIntPriority(priority)        ( IPC8CLR = _IPC8_PMPIP_MASK, IPC8SET = ((priority) << _IPC8_PMPIP_POSITION))
#define mPMPGetIntPriority()                (IPC8bits.PMPIP)
#define mPMPSetIntSubPriority(subPriority)  ( IPC8CLR = _IPC8_PMPIS_MASK, IPC8SET = ((subPriority) << _IPC8_PMPIS_POSITION))
#define mPMPGetIntSubPriority()             (IPC8bits.PMPIS)
#endif
/*********************************************************************
 * <combinewith mCMP1GetIntFlag, mCMP1GetIntEnable, mCMP1IntEnable, mCMP1SetIntPriority, 
 * mCMP1GetIntPriority, mCMP1SetIntSubPriority, mCMP1GetIntSubPriority,
 * mCMP2ClearIntFlag, mCMP2GetIntEnable, mCMP2GetIntFlag, mCMP2GetIntPriority, mCMP2GetIntSubPriority, 
 * mCMP2IntEnable, mCMP2SetIntPriority, mCMP2SetIntSubPriority>
 *
 * Comparator x Interrupt Control Functions
 *
 * Function:        void    mCMPxClearIntFlag(void)
 *					int     mCMPxGetIntFlag(void)
 *					void    mCMPxSetIntEnable(void)
 *					int     mCMPxGetIntEnable(void)
 *                  void    mCMPxClearIntEnable(void)
 *					void    mCMPxxSetIntPriority(int priority)
 *					int     mCMPxxGetIntPriority(void)
 *					void    mCMPxxSetIntSubPriority(subPriority)
 *					int     mCMPxxGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 ********************************************************************/
#ifdef _CMP
#define mCMP1ClearIntFlag()                     (IFS1CLR = _IFS1_CMP1IF_MASK)
#define mCMP1GetIntFlag()                       (IFS1bits.CMP1IF)
#define mCMP1GetIntEnable()                     (IEC1bits.CMP1IE)
#define mCMP1IntEnable(enable)                  (IEC1CLR = _IEC1_CMP1IE_MASK, IEC1SET = ((enable) << _IEC1_CMP1IE_POSITION))
#define mCMP1SetIntPriority(priority)           ( IPC6CLR = _IPC6_CMP1IP_MASK, IPC6SET = ((priority) << _IPC6_CMP1IP_POSITION))
#define mCMP1GetIntPriority()                   (IPC6bits.CMP1IP)
#define mCMP1SetIntSubPriority(subPriority)     ( IPC6CLR = _IPC6_CMP1IS_MASK, IPC6SET = ((subPriority) << _IPC6_CMP1IS_POSITION))
#define mCMP1GetIntSubPriority()                (IPC6bits.CMP1IS)
/**********************************************
  * Comparator 2 Interrupt Control Functions                                                                                                                  
  **********************************************/
#define mCMP2ClearIntFlag()                     (IFS1CLR = _IFS1_CMP2IF_MASK)
#define mCMP2GetIntFlag()                       (IFS1bits.CMP2IF)
#define mCMP2GetIntEnable()                     (IEC1bits.CMP2IE)
#define mCMP2IntEnable(enable)                  (IEC1CLR = _IEC1_CMP2IE_MASK, IEC1SET = ((enable) << _IEC1_CMP2IE_POSITION))
#define mCMP2SetIntPriority(priority)           ( IPC7CLR = _IPC7_CMP2IP_MASK, IPC7SET = ((priority) << _IPC7_CMP2IP_POSITION))
#define mCMP2GetIntPriority()                   (IPC7bits.CMP2IP)
#define mCMP2SetIntSubPriority(subPriority)     ( IPC7CLR = _IPC7_CMP2IS_MASK, IPC7SET = ((subPriority) << _IPC7_CMP2IS_POSITION))
#define mCMP2GetIntSubPriority()                (IPC7bits.CMP2IS)
#endif
/*********************************************************************
 * <combinewith mFSCMGetIntEnable, mFSCMGetIntFlag, mFSCMGetIntPriority, mFSCMGetIntSubPriority, 
 * mFSCMIntEnable, mFSCMSetIntPriority, mFSCMSetIntSubPriority>
 *
 * Fail Safe Clock Monitor Interrupt Control Functions
 *
 * Function:        void    mFSCMClearIntFlag(void)
 *					int     mFSCMGetIntFlag(void)
 *					void    mFSCMSetIntEnable(void)
 *					int     mFSCMGetIntEnable(void)
 *                  void    mFSCMClearIntEnable(void)
 *					void    mFSCMSetIntPriority(int priority)
 *					int     mFSCMGetIntPriority(void)
 *					void    mFSCMSetIntSubPriority(subPriority)
 *					int     mFSCMGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 ********************************************************************/
#define mFSCMClearIntFlag()                 (IFS0CLR = _IFS0_FSCMIF_MASK)
#define mFSCMGetIntFlag()                   (IFS0bits.FSCMIF)
#define mFSCMGetIntEnable()                 (IEC0bits.FSCMIE)
#define mFSCMIntEnable(enable)              (IEC0CLR = _IEC0_FSCMIE_MASK, IEC0SET = ((enable) << _IEC0_FSCMIE_POSITION))
#define mFSCMSetIntPriority(priority)       (IPC6CLR = _IPC6_FSCMIP_MASK, IPC6SET = ((priority) << _IPC6_FSCMIP_POSITION))
#define mFSCMGetIntPriority()               (IPC6bits.FSCMIP)
#define mFSCMSetIntSubPriority(subPriority) (IPC6CLR = _IPC6_FSCMIS_MASK, IPC6SET = ((subPriority) << _IPC6_FSCMIS_POSITION))
#define mFSCMGetIntSubPriority()            (IPC6bits.FSCMIS)

/*********************************************************************
 * <combinewith mFCEGetIntEnable, mFCEGetIntFlag, mFCEGetIntPriority, mFCEGetIntSubPriority, 
 * mFCEIntEnable, mFCESetIntPriority, mFCESetIntSubPriority>
 *
 * Flash Control Event Interrupt Control Functions
 *
 * Function:        void    mFCEClearIntFlag(void)
 *					int     mFCEGetIntFlag(void)
 *					void    mFCESetIntEnable(void)
 *					int     mFCEGetIntEnable(void)
 *                  void    mFCEClearIntEnable(void)
 *					void    mFCESetIntPriority(int priority)
 *					int     mFCEGetIntPriority(void)
 *					void    mFCESetIntSubPriority(subPriority)
 *					int     mFCEGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 ********************************************************************/
#define mFCEClearIntFlag()                 (IFS0CLR = _IFS0_FCEIF_MASK)
#define mFCEGetIntFlag()                   (IFS0bits.FCEIF)
#define mFCEGetIntEnable()                 (IEC0bits.FCEIE)
#define mFCEIntEnable(enable)              (IEC0CLR = _IEC0_FCEIE_MASK, IEC0SET = ((enable) << _IEC0_FCEIE_POSITION))
#define mFCESetIntPriority(priority)       (IPC6CLR = _IPC6_FCEIP_MASK, IPC6SET = ((priority) << _IPC6_FCEIP_POSITION))
#define mFCEGetIntPriority()               (IPC6bits.FCEIP)
#define mFCESetIntSubPriority(subPriority) (IPC6CLR = _IPC6_FCEIS_MASK, IPC6SET = ((subPriority) << _IPC6_FCEIS_POSITION))
#define mFCEGetIntSubPriority()            (IPC6bits.FCEIS)

/*********************************************************************
 * <combinewith mRTCCGetIntEnable, mRTCCGetIntFlag, mRTCCGetIntPriority, mRTCCGetIntSubPriority, 
 * mRTCCIntEnable, mRTCCSetIntPriority, mRTCCSetIntSubPriority>
 *
 * Real Time Clock Interrupt Control Functions
 *
 * Function:        void    mRTCCClearIntFlag(void)
 *					int     mRTCCGetIntFlag(void)
 *					void    mRTCCSetIntEnable(void)
 *					int     mRTCCGetIntEnable(void)
 *                  void    mRTCCClearIntEnable(void)
 *					void    mRTCCSetIntPriority(int priority)
 *					int     mRTCCGetIntPriority(void)
 *					void    mRTCCSetIntSubPriority(subPriority)
 *					int     mRTCCGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 ********************************************************************/
#ifdef _RTCC
#define mRTCCClearIntFlag()                 (IFS0CLR = _IFS0_RTCCIF_MASK)
#define mRTCCGetIntFlag()                   (IFS0bits.RTCCIF)
#define mRTCCGetIntEnable()                 (IEC0bits.RTCCIE)
#define mRTCCIntEnable(enable)              (IEC0CLR = _IEC0_RTCCIE_MASK, IEC0SET = ((enable) << _IEC0_RTCCIE_POSITION))
#define mRTCCSetIntPriority(priority)       (IPC6CLR = _IPC6_RTCCIP_MASK, IPC6SET = ((priority) << _IPC6_RTCCIP_POSITION))
#define mRTCCGetIntPriority()               (IPC6bits.RTCCIP)
#define mRTCCSetIntSubPriority(subPriority) (IPC6CLR = _IPC6_RTCCIS_MASK, IPC6SET = ((subPriority) << _IPC6_RTCCIS_POSITION))
#define mRTCCGetIntSubPriority()            (IPC6bits.RTCCIS)
#endif
/**************************************************************************************************************************
  * <combinewith mDMA0GetIntFlag, mDMA0GetIntEnable, mDMA0IntEnable, mDMA0SetIntPriority, mDMA0GetIntPriority, 
  * mDMA0SetIntSubPriority, mDMA0GetIntSubPriority, mDMA1ClearIntFlag, mDMA1GetIntFlag, mDMA1GetIntEnable, 
  * mDMA1IntEnable, mDMA1SetIntPriority, mDMA1GetIntPriority, mDMA1SetIntSubPriority, mDMA1GetIntSubPriority, 
  * mDMA2ClearIntFlag, mDMA2GetIntFlag, mDMA2GetIntEnable, mDMA2IntEnable, mDMA2SetIntPriority, mDMA2GetIntPriority, 
  * mDMA2SetIntSubPriority, mDMA2GetIntSubPriority, mDMA3ClearIntFlag, mDMA3GetIntFlag, mDMA3GetIntEnable, mDMA3IntEnable, 
  * mDMA3SetIntPriority, mDMA3GetIntPriority, mDMA3SetIntSubPriority, mDMA3GetIntSubPriority, mDMA4GetIntFlag, 
  * mDMA4GetIntEnable, mDMA4IntEnable, mDMA4SetIntPriority, mDMA4GetIntPriority, 
  * mDMA4SetIntSubPriority, mDMA4GetIntSubPriority, mDMA5ClearIntFlag, mDMA5GetIntFlag, mDMA5GetIntEnable, 
  * mDMA5IntEnable, mDMA5SetIntPriority, mDMA5GetIntPriority, mDMA5SetIntSubPriority, mDMA5GetIntSubPriority, 
  * mDMA6ClearIntFlag, mDMA6GetIntFlag, mDMA6GetIntEnable, mDMA6IntEnable, mDMA6SetIntPriority, mDMA6GetIntPriority, 
  * mDMA6SetIntSubPriority, mDMA6GetIntSubPriority, mDMA7ClearIntFlag, mDMA7GetIntFlag, mDMA7GetIntEnable, mDMA7IntEnable, 
  * mDMA7SetIntPriority, mDMA7GetIntPriority, mDMA7SetIntSubPriority, mDMA7GetIntSubPriority>
  * 
  * DMA x Channel Interrupt Control Functions
  * Prototype:
  *     void mDMAxClearIntFlag(void) int mDMAxGetIntFlag(void) void
  *     mDMAxSetIntEnable(void) int mDMAxGetIntEnable(void) void
  *     mDMAxClearIntEnable(void) void mDMAxSetIntPriority(int
  *     priority) int mDMAxGetIntPriority(void) void
  *     mDMAxSetIntSubPriority(subPriority) int
  *     mDMAxGetIntSubPriority(void)
  * Preconditions:
  *     EBASE and IntCtl.VS set up
  * Input:
  *     priority :     A value between 0 \- 7 inclusive.
  *     subPriority :  A value between 0 \- 3 inclusive
  * Return:
  *     For 'Get' functions, the returned values are of
  *     same type as 'Set' functions.
  *     
  * Side Effects:
  *     None
  * Remarks:
  *     None                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
  *************************************************************************************************************************
*/
#ifdef _DMAC0
#define mDMA0ClearIntFlag()                     (IFS1CLR = _IFS1_DMA0IF_MASK)
#define mDMA0GetIntFlag()                       (IFS1bits.DMA0IF)
#define mDMA0GetIntEnable()                     (IEC1bits.DMA0IE)
#define mDMA0IntEnable(enable)                  (IEC1CLR = _IEC1_DMA0IE_MASK, IEC1SET = ((enable) << _IEC1_DMA0IE_POSITION))
#define mDMA0SetIntPriority(priority)           (IPC10CLR = _IPC10_DMA0IP_MASK, IPC10SET = ((priority) << _IPC10_DMA0IP_POSITION))
#define mDMA0GetIntPriority()                   (IPC10bits.DMA0IP)
#define mDMA0SetIntSubPriority(subPriority)     (IPC10CLR = _IPC10_DMA0IS_MASK, IPC10SET = ((subPriority) << _IPC10_DMA0IS_POSITION))
#define mDMA0GetIntSubPriority()                (IPC10bits.DMA0IS)
#endif

#ifdef _DMAC1
#define mDMA1ClearIntFlag()                     (IFS1CLR = _IFS1_DMA1IF_MASK)
#define mDMA1GetIntFlag()                       (IFS1bits.DMA1IF)
#define mDMA1GetIntEnable()                     (IEC1bits.DMA1IE)
#define mDMA1IntEnable(enable)                  (IEC1CLR = _IEC1_DMA1IE_MASK, IEC1SET = ((enable) << _IEC1_DMA1IE_POSITION))
#define mDMA1SetIntPriority(priority)           (IPC10CLR = _IPC10_DMA1IP_MASK, IPC10SET = ((priority) << _IPC10_DMA1IP_POSITION))
#define mDMA1GetIntPriority()                   (IPC10bits.DMA1IP)
#define mDMA1SetIntSubPriority(subPriority)     (IPC10CLR = _IPC10_DMA1IS_MASK, IPC10SET = ((subPriority) << _IPC10_DMA1IS_POSITION))
#define mDMA1GetIntSubPriority()                (IPC10bits.DMA1IS)
#endif

#ifdef _DMAC2
#define mDMA2ClearIntFlag()                     (IFS1CLR = _IFS1_DMA2IF_MASK)
#define mDMA2GetIntFlag()                       (IFS1bits.DMA2IF)
#define mDMA2GetIntEnable()                     (IEC1bits.DMA2IE)
#define mDMA2IntEnable(enable)                  (IEC1CLR = _IEC1_DMA2IE_MASK, IEC1SET = ((enable) << _IEC1_DMA2IE_POSITION))
#define mDMA2SetIntPriority(priority)           (IPC10CLR = _IPC10_DMA2IP_MASK, IPC10SET = ((priority) << _IPC10_DMA2IP_POSITION))
#define mDMA2GetIntPriority()                   (IPC10bits.DMA2IP)
#define mDMA2SetIntSubPriority(subPriority)     (IPC10CLR = _IPC10_DMA2IS_MASK, IPC10SET = ((subPriority) << _IPC10_DMA2IS_POSITION))
#define mDMA2GetIntSubPriority()                (IPC10bits.DMA2IS)
#endif

#ifdef _DMAC3
#define mDMA3ClearIntFlag()                     (IFS1CLR = _IFS1_DMA3IF_MASK)
#define mDMA3GetIntFlag()                       (IFS1bits.DMA3IF)
#define mDMA3GetIntEnable()                     (IEC1bits.DMA3IE)
#define mDMA3IntEnable(enable)                  (IEC1CLR = _IEC1_DMA3IE_MASK, IEC1SET = ((enable) << _IEC1_DMA3IE_POSITION))
#define mDMA3SetIntPriority(priority)           (IPC10CLR = _IPC10_DMA3IP_MASK, IPC10SET = ((priority) << _IPC10_DMA3IP_POSITION))
#define mDMA3GetIntPriority()                   (IPC10bits.DMA3IP)
#define mDMA3SetIntSubPriority(subPriority)     (IPC10CLR = _IPC10_DMA3IS_MASK, IPC10SET = ((subPriority) << _IPC10_DMA3IS_POSITION))
#define mDMA3GetIntSubPriority()                (IPC10bits.DMA3IS)
#endif


/*********************************************************************
 * <combinewith mUSBClearIntFlag, mUSBGetIntFlag, mUSBIntEnable, mUSBSetIntPriority, 
 * mUSBGetIntPriority, mUSBSetIntSubPriority, mUSBGetIntSubPriority>
 *
 * USB Interrrupt Contol Funtions
 *
 * Function:        void    mUSBClearIntFlag(void)
 *					int     mUSBGetIntFlag(void)
 *					void    mUSBIntEnable(void)
 *					int     mUSBGetIntEnable(void)
 *					void    mUSBSetIntPriority(int priority)
 *					int     mUSBGetIntPriority(void)
 *					void    mUSBSetIntSubPriority(subPriority)
 *					int     mUSBGetIntSubPriority(void)
 *
 * PreCondition:    EBASE and IntCtl.VS set up
 *
 * Inputs:			priority:  		A value between 0 - 7 inclusive.
 *					subPriority:	A value between 0 - 3 inclusive
 *
 * Output:          For 'Get' functions, the returned values are of
 *					same type as 'Set' functions.
 *
 * Side Effects:    None
 *
 * Overview:        None
 ********************************************************************/
#ifdef _USB
#define mUSBClearIntFlag()                 (IFS1CLR = _IFS1_USBIF_MASK)
#define mUSBGetIntFlag()                   (IFS1bits.USBIF)
#define mUSBGetIntEnable()                 (IEC1bits.USBIE)
#define mUSBIntEnable(enable)              (IEC1CLR = _IEC1_USBIE_MASK, IEC1SET = ((enable) << _IEC1_USBIE_POSITION))
#define mUSBSetIntPriority(priority)       (IPC7CLR = _IPC7_USBIP_MASK, IPC7SET = ((priority) << _IPC7_USBIP_POSITION))
#define mUSBGetIntPriority()               (IPC7bits.USBIP)
#define mUSBSetIntSubPriority(subPriority) (IPC7CLR = _IPC7_USBIS_MASK, IPC7SET = ((subPriority) << _IPC7_USBIS_POSITION))
#define mUSBGetIntSubPriority()            (IPC7bits.USBIS)
#endif

#endif
