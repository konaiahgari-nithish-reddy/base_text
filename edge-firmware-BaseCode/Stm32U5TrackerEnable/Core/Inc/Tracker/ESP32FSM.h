/*
 * ESP32FSM.h
 *
 *  Created on: 29-Oct-2023
 *      Author: Ravi's PC
 */

#ifndef INC_TRACKER_ESP32FSM_H_
#define INC_TRACKER_ESP32FSM_H_

void WIFI_Uart_Send(unsigned char *buf);
int WIFI_Uart_Recv(unsigned char *buf,int size);

#endif /* INC_TRACKER_ESP32FSM_H_ */
