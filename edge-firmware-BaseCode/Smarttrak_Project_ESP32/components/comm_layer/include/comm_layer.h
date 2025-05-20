#pragma once

#include "esp_err.h"
#include "cJSON.h"
#include "mqtt_client.h"

#define SIMULATION_NODE	0
#define MESH_ROOT_NODE	0
#define MQTT_WITH_CREDS

esp_err_t uart_init(void) ;
void rx_task(void *arg);
int sendData(const char *logName, const char *data);
void tx_task(void *arg);
esp_err_t construct_mqtt_packet(char *str, uint8_t len);
int transmit_stm32(uint8_t *data, uint16_t size);


