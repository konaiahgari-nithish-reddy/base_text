/*
 * Drv8245.h
 *
 *  Created on: 13-Aug-2023
 *      Author: Ravi's PC
 */

#ifndef INC_TRACKER_DRV8245_H_
#define INC_TRACKER_DRV8245_H_

void WakeUpDrv8245(void);
void EnableDrv8245(void);
void SetMotor(uint8_t Direction);
void SetDutyCycleForward(uint32_t DutyCycle);
void SetDutyCycleReverse(uint32_t DutyCycle);
void SetPins(void);
void StopMotor(uint32_t DutyCycle);

#endif /* INC_TRACKER_DRV8245_H_ */
