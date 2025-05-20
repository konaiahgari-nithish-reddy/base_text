// *************************************************************************************************
//										S e r i a l P o r t . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	PIC32MX360F512L MCU Serial Port initialization and handling definitions

//
// *************************************************************************************************

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

// serial I/O buffer definitions. Buffer is long enough for 4 complete display lines
#define	SERIAL_BUFFER_LEN		(unsigned char)321                      // memory is not an issue with this processor
#define	MAX_BUFFER_INDEX		(unsigned char)(SERIAL_BUFFER_LEN - 1)


// The serial handler errors may be used by the Interrupt.c (serial) interrupt handler, or the SerialPort.c functions

//---------------------------------------------------------------------
//						Function Prototypes
//---------------------------------------------------------------------

// Init
void InitializeSerialPort(UART_MODULE UARTid, UINT32 lBaudRate);
void ChangeSerialBaudRate(UART_MODULE UARTid, UINT32 lBaudRate);

// Receive
int StartRx(UART_MODULE id);
BOOL AnyRxDataAvailable(UART_MODULE id);
unsigned char ReadRxdData(UART_MODULE id);

// alternate/older names
#define	StartSerialRx(x)			StartRx(x)
#define	AnySerialRxDataAvailable	AnyRxDataAvailable		// <sek> test for received byte available
#define	ReadSerialRxdData(x)		ReadRxdData(x)			// <sek> return oldest byte from queue

//UINT32 GetMenuChoice(void);
//UINT32 GetDataBuffer(char *buffer, UINT32 max_size);

// Transmit
unsigned char IsTransmitComplete(UART_MODULE id);
void StartTransmitString(UART_MODULE id, const char *pstrString);

// alternate/older names
#define	IsSerialTxComplete(x)		IsTransmitComplete(x)

//void SendDataBuffer(const char *buffer, UINT32 size);

//---------------------------------------------------------------------
//							Global Data
//---------------------------------------------------------------------

// note: the term 'pointer' is grossly misused here; these are indicies into the RxdData and TxdData ring buffers

// NOTE: these variables are globals because they are accessed in Interrupt.c and SerialPort.c
#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#if defined (DEFINE_GLOBALS)
	GLOBAL_INIT unsigned char volatile ucsRxdInIndex[UART_CNT] = {0, 0};
	GLOBAL_INIT unsigned char volatile ucsRxdOutIndex[UART_CNT] = {0, 0};
	GLOBAL_INIT ARRAY unsigned char volatile ucsRxdData[UART_CNT][SERIAL_BUFFER_LEN];	// received data ring buffer  rror 728: (Info -- Symbol 'ucsRxdData' not explicitly initialized)

	GLOBAL_INIT	volatile char * pISRTxData[UART_CNT] = {NULL, NULL};					// pointer to caller supplied Transmit buffer, used by Tx ISR

	GLOBAL_INIT	volatile BOOL bTxDone[UART_CNT] = {FALSE, FALSE};
        GLOBAL_INIT  UINT8   TXLen[UART_CNT] = {FALSE, FALSE};
        GLOBAL_INIT  UINT8   SetTXLen[UART_CNT] = {FALSE, FALSE};
        GLOBAL_INIT  UINT8   TXMode[UART_CNT] = {FALSE, FALSE};
#elif defined (DEFINE_EXTERNS)
	GLOBAL unsigned char volatile ucsRxdInIndex[];
	GLOBAL unsigned char volatile ucsRxdOutIndex[];
	GLOBAL ARRAY unsigned char volatile ucsRxdData[UART_CNT][SERIAL_BUFFER_LEN];

	GLOBAL volatile char * pISRTxData[];

	GLOBAL volatile BOOL bTxDone[];
        UINT8   TXLen[1000];
        UINT8   SetTXLen[1000];
        UINT8   TXMode[1000];
#endif


// end of SerialPort.h

