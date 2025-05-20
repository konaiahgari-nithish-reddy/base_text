

#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

#include "gsfstd.h"						// gsf standard #defines
#include "Debug.h"

#include "HardwareProfile.h"

#include "LEDs.h"				// LED display handler function definition, used for stall recovery states
#include "SmartTrak.h"

#include "Stubs.h"



void Reset(void)
{
	// PIC32 Peripheral Library Function
	SoftReset();
}

#ifdef USE_DYNAMIC_LEDS
void SetLEDs(enum tagLEDstates eLEDstate)
{

}
#endif
