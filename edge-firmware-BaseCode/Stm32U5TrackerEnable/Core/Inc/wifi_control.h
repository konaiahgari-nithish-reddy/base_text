/*
 * wifi_control.h
 *
 *  Created on: Feb 18, 2024
 *      Author: saura
 */

#ifndef INC_WIFI_CONTROL_H_
#define INC_WIFI_CONTROL_H_

#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "cJSON.h"
#include "config.h"
#include "GenericTypeDefs.h"
#include "gsfstd.h"
#include "MotionProfile.h"
#include "MotionSensor.h"
#include "DS3232.h"
#include "mma845x.h"
#include "lis2hh12_reg.h"
#include "SmartTrak.h"
#include "RTCC.h"
#include "SerialDisplay.h"



#define RX_BUFF_SIZE	512

// Structure variable
typedef struct time_setting{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} time_setting_t;


typedef struct date_setting {
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
}date_setting_t;


typedef struct sensor_data {
	uint8_t status;
	uint8_t int_src;
	uint8_t sys_mode;
	uint8_t who_am_i;
	uint8_t ctrl_1;
	uint8_t ctrl_2;
	uint8_t ctrl_3;
	uint8_t sensor_x_h;
	uint8_t sensor_x_l;
	uint8_t sensor_y_h;
	uint8_t sensor_y_l;
	uint8_t sensor_z_h;
	uint8_t sensor_z_l;
	int16_t sensor_x;
	int16_t sensor_y;
	int16_t sensor_z;
}sensor_data_t;


typedef union float_uint32 {
	float data_float;
	uint32_t data_int;
}float_uint32_t;

typedef struct location_data {
	float_uint32_t latitude;
	float_uint32_t longitude;
	float_uint32_t altitude;
	float_uint32_t timezone;
}location_data_t;

typedef struct spa_data{
	float spa_azimuth;
	float spa_elevation;
	float setPoint_az;
	float setPoint_el;
}spa_angle_t;

typedef struct system_orientation{
	float sun_angle;
	float target_angle;
	float tracker_angle;
	// Dummy stram values
	float tracker_y_angle;
	float moduleTilt;
	float spaAZ;
	float spaEL;
	float az;
	float el;

}system_orientation_t;

typedef struct setting_data {
	// Unit Location
	float_uint32_t latitude;
	float_uint32_t longitude;
	float_uint32_t altitude;
	float_uint32_t refraction;
	float_uint32_t timezone;
	uint16_t tracking_mode;

	// Azimuth Settings
	float_uint32_t	fAZ_Offset;
	float_uint32_t	fAZ_SoftLimit_Reverse;
	float_uint32_t	fAZ_SoftLimit_Forward;
	float_uint32_t	fAZ_DeadBand;
	float_uint32_t	fAZ_NightStowThreshold;
	float_uint32_t	fAZ_NightStowPosition;
	float_uint32_t	fAZ_WindStowPosition;

	// Elevation Settings
	float_uint32_t	fEL_Offset;
	float_uint32_t	fEL_SoftLimit_Reverse;
	float_uint32_t	fEL_SoftLimit_Forward;
	float_uint32_t	fEL_DeadBand;
	float_uint32_t	fEL_NightStowThreshold;
	float_uint32_t	fEL_NightStowPosition;
	float_uint32_t	fEL_WindStowPosition;

	// Backtracking
	uint8_t bBacktrackingEnabled;
	float_uint32_t	fPanelShadowStartAngleDegrees;
	float_uint32_t	fSunShadowStartAngleDegrees;
	float_uint32_t	fSunShadowStartHeight;

	// SingleAxis Softlimits
	float_uint32_t  fSingle_SoftLimit_Forward;
	float_uint32_t  fSingle_SoftLimit_Reverse;

	// SingleAxis days limits
	float_uint32_t  fSingle_start_date;
	float_uint32_t  fSingle_stop_days;
}setting_data_t;


void read_monitor_params(void);
void create_mon_mqtt_packet(void);
void read_setting_params(void);
void create_read_setting_mqtt(void);
void parse_stm32_packet(char *str, uint16_t len);

#endif /* INC_WIFI_CONTROL_H_ */
