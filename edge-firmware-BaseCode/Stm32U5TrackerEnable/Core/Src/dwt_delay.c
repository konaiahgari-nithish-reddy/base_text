/*
 * Simple microseconds delay routine, utilizing ARM's DWT
 * (Data Watchpoint and Trace Unit) and HAL library.
 * Intended to use with gcc compiler, but I hope it can be used
 * with any other C compiler across the Universe (provided that
 * ARM and CMSIS already invented) :)
 * Max K
 *
 *
 * This file is part of DWT_Delay package.
 * DWT_Delay is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * us_delay is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * http://www.gnu.org/licenses/.
 */

#include "stm32u5xx_hal.h"         // change to whatever MCU you use
#include "dwt_delay.h"

/**
 * Initialization routine.
 * You might need to enable access to DWT registers on Cortex-M7
 *   DWT->LAR = 0xC5ACCE55
 */

void DWT_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

#if DWT_DELAY_NEWBIE
/**
 * If you are a newbie and see magic in DWT_Delay, consider this more
 * illustrative function, where you explicitly determine a counter
 * value when delay should stop while keeping things in bounds of uint32.
*/
void DWT_Delay(uint32_t us) // microseconds
{
    uint32_t startTick  = DWT->CYCCNT,
             targetTick = DWT->CYCCNT + us * (SystemCoreClock/1000000);

    // Must check if target tick is out of bounds and overflowed
    if (targetTick > startTick) {
        // Not overflowed
        while (DWT->CYCCNT < targetTick);
    } else {
        // Overflowed
        while (DWT->CYCCNT > startTick || DWT->CYCCNT < targetTick);
    }
}
#else
/**
 * Delay routine itself.
 * Time is in microseconds (1/1000000th of a second), not to be
 * confused with millisecond (1/1000th).
 *
 * No need to check an overflow. Let it just tick :)
 *
 * @param uint32_t us  Number of microseconds to delay for
 */
/*void DWT_Delay(uint32_t us) // microseconds
{
    volatile uint32_t *DWT_CONTROL = (uint32_t *) 0xE0001000;
    volatile uint32_t *DWT_CYCCNT = (uint32_t *) 0xE0001004;
    volatile uint32_t *DEMCR = (uint32_t *) 0xE000EDFC;

    // Enable DWT cycle counter
    *DEMCR |= 0x01000000;;
    *DWT_CYCCNT = 0;
    *DWT_CONTROL = *DWT_CONTROL | 1;

    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000) / 10; // Adjust for 160MHz clock

    while (( DWT->CYCCNT - startTick) < delayTicks);
}*/

void DWT_Delay(uint32_t us) // microseconds
{
	volatile uint32_t *DWT_CONTROL = (uint32_t *) 0xE0001000;

	volatile uint32_t *DWT_CYCCNT = (uint32_t *) 0xE0001004;

	volatile uint32_t *DEMCR = (uint32_t *) 0xE000EDFC;

	*DEMCR = *DEMCR | 0x01000000;

	*DWT_CYCCNT = 0;

	*DWT_CONTROL = *DWT_CONTROL | 1;

	uint32_t startTick = DWT->CYCCNT,
			delayTicks = us * (SystemCoreClock/1000000);

	while (DWT->CYCCNT - startTick < delayTicks);
}


// This Function Provides Delay In Microseconds Using DWT
void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}

#endif
