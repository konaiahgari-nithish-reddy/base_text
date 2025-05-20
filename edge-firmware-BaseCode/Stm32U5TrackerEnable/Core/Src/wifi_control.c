/*
 * wifi_control.c
 *
 *  Created on: Feb 18, 2024
 *      Author: Saurabh Nishad
 *      Company: Nexotron Infotech Pvt. Ltd.
 *      url: www.nexotron.com
 */

#include "wifi_control.h"
#include "MotorPWM.h"
#include "Inclinometer.h"

#define MOTOR_TEST	1

volatile uint8_t wifi_flag = 1;
volatile uint32_t wifi_ctr = 0;

// Variable list
volatile uint8_t rx_buff[RX_BUFF_SIZE] = {0};
volatile uint16_t rx_buff_ctr = 0;
volatile uint8_t json_packet_flag = 0;
volatile uint8_t json_validator = 0;
volatile uint16_t json_packet_size = 0;
uint8_t setting_flag;

time_setting_t mon_time;
date_setting_t mon_date;
sensor_data_t mon_sensor;
location_data_t mon_location;
setting_data_t mem_setting;

spa_angle_t sun_angle;
system_orientation_t tracker_angle;

char buildDate[21];

char nodeID[10] = "SMRTR0000";

extern BOOL gIsMma845Enabled;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart3;
HAL_StatusTypeDef pStatus;
extern SmartTrakOrientation acquisitionAngle;

void read_monitor_params(void) {
	// Read RTC Data
	if (ReadRTCCRegister(DS3232_REG_SECONDS, &mon_time.second)	!= TRUE)
	{
		printf("Failed to Read time data.\n");
	}
	if (ReadRTCCRegister(DS3232_REG_MINUTES, &mon_time.minute)	!= TRUE)
	{
		printf("Failed to Read time data.\n");
	}
	if (ReadRTCCRegister(DS3232_REG_HOURS, &mon_time.hour) != TRUE)
	{
		printf("Failed to Read time data.\n");
	}
	if (ReadRTCCRegister(DS3232_REG_DAY, &mon_date.day) != TRUE)
	{
		printf("Failed to Read time data.\n");
	}
	if (ReadRTCCRegister(DS3232_REG_DATE, &mon_date.date) != TRUE)
	{
		printf("Failed to Read time data.\n");
	}
	if (ReadRTCCRegister(DS3232_REG_MONTH, &mon_date.month) != TRUE)
	{
		printf("Failed to Read time data.\n");
	}
	if (ReadRTCCRegister(DS3232_REG_YEAR, &mon_date.year) != TRUE)
	{
		printf("Failed to Read time data.\n");
	}


	// Read Sensor Data
	if (gIsMma845Enabled == FALSE) {
		LIS2HH12_Init();
		// Register 0x00 Status
		if (ReadInclinometerRegister(LIS2HH12_STATUS, &mon_sensor.status) != TRUE) {
			// Read LIS2HH12_STATUS
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x0B System Mode
		if (ReadInclinometerRegister(SYSMOD_REG, &mon_sensor.sys_mode) != TRUE) {
			// Read SYSMOD_REG
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x0C Interrupt Source
		if (ReadInclinometerRegister(INT_SOURCE_REG, &mon_sensor.int_src) != TRUE) {
			// Read INT_SOURCE_REG
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x0D Who Am I
		if (ReadInclinometerRegister(LIS2HH12_WHO_AM_I, &mon_sensor.who_am_i) != TRUE) {
			// Read LIS2HH12_WHO_AM_I
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x2A Control Reg
		if (ReadInclinometerRegister(LIS2HH12_CTRL1, &mon_sensor.ctrl_1) != TRUE) {
			// Read LIS2HH12_CTRL1
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x2B Control Reg
		if (ReadInclinometerRegister(LIS2HH12_CTRL2, &mon_sensor.ctrl_2) != TRUE) {
			// Read LIS2HH12_CTRL2
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x2C Control Reg
		if (ReadInclinometerRegister(LIS2HH12_CTRL3, &mon_sensor.ctrl_3) != TRUE) {
			// Read LIS2HH12_CTRL3
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x01 Out X MSB
		if (ReadInclinometerRegister(LIS2HH12_OUT_X_H, &mon_sensor.sensor_x_h) == TRUE) {
			// Read LIS2HH12_OUT_X_H
			mon_sensor.sensor_x = (mon_sensor.sensor_x_h << 8);
		}
		// Register 0x02 Out X LSB
		if (ReadInclinometerRegister(LIS2HH12_OUT_X_L, &mon_sensor.sensor_x_l) == TRUE) {
			// Read LIS2HH12_OUT_X_L
			mon_sensor.sensor_x = (mon_sensor.sensor_x | mon_sensor.sensor_x_l);
		}
		// Register 0x03 Out Y MSB
		if (ReadInclinometerRegister(LIS2HH12_OUT_Y_H, &mon_sensor.sensor_y_h) == TRUE) {
			// Read LIS2HH12_OUT_Y_H
			mon_sensor.sensor_y = (mon_sensor.sensor_y_h << 8);
		}
		// Register 0x04 Out Y LSB
		if (ReadInclinometerRegister(LIS2HH12_OUT_Y_L, &mon_sensor.sensor_y_l) == TRUE) {
			// Read LIS2HH12_OUT_Y_L
			mon_sensor.sensor_y = (mon_sensor.sensor_y | mon_sensor.sensor_y_l);
		}
		// Register 0x05 Out Z MSB
		if (ReadInclinometerRegister(LIS2HH12_OUT_Z_H, &mon_sensor.sensor_z_h) == TRUE) {
			// Read LIS2HH12_OUT_Z_H
			mon_sensor.sensor_z = (mon_sensor.sensor_z_h << 8);
		}
		// Register 0x06 Out Z LSB
		if (ReadInclinometerRegister(LIS2HH12_OUT_Z_L, &mon_sensor.sensor_z_l) == TRUE) {
			// Read LIS2HH12_OUT_Z_L
			mon_sensor.sensor_z = (mon_sensor.sensor_z | mon_sensor.sensor_z_l);
		}
	} else {
		MMA845x_Init();

		// Register 0x00 Status
		if (ReadInclinometerRegister(STATUS_00_REG, &mon_sensor.status) != TRUE) {
			// Read LIS2HH12_STATUS
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x0B System Mode
		if (ReadInclinometerRegister(SYSMOD_REG, &mon_sensor.sys_mode) != TRUE) {
			// Read SYSMOD_REG
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x0C Interrupt Source
		if (ReadInclinometerRegister(INT_SOURCE_REG, &mon_sensor.int_src) != TRUE) {
			// Read INT_SOURCE_REG
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x0D Who Am I
		if (ReadInclinometerRegister(WHO_AM_I_REG, &mon_sensor.who_am_i) != TRUE) {
			// Read WHO_AM_I_REG
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x2A Control Reg
		if (ReadInclinometerRegister(CTRL_REG1, &mon_sensor.ctrl_1) != TRUE) {
			// Read CTRL_REG1
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x2B Control Reg
		if (ReadInclinometerRegister(CTRL_REG2, &mon_sensor.ctrl_2) != TRUE) {
			// Read CTRL_REG2
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x2C Control Reg
		if (ReadInclinometerRegister(CTRL_REG3, &mon_sensor.ctrl_3) != TRUE) {
			// Read CTRL_REG3
			printf("Failed to Read sensor data.\n");
		}
		// Register 0x01 Out X MSB
		if (ReadInclinometerRegister(OUT_X_MSB_REG, &mon_sensor.sensor_x_h) == TRUE) {
			// Read OUT_X_MSB_REG
			mon_sensor.sensor_x = (mon_sensor.sensor_x_h << 8);
		}
		// Register 0x02 Out X LSB
		if (ReadInclinometerRegister(OUT_X_LSB_REG, &mon_sensor.sensor_x_l) == TRUE) {
			// Read OUT_X_LSB_REG
			mon_sensor.sensor_x = (mon_sensor.sensor_x | mon_sensor.sensor_x_l);
		}
		// Register 0x03 Out Y MSB
		if (ReadInclinometerRegister(OUT_Y_MSB_REG, &mon_sensor.sensor_y_h) == TRUE) {
			// Read OUT_Y_MSB_REG
			mon_sensor.sensor_y = (mon_sensor.sensor_y_h << 8);
		}
		// Register 0x04 Out Y LSB
		if (ReadInclinometerRegister(OUT_Y_LSB_REG, &mon_sensor.sensor_y_l) == TRUE) {
			// Read OUT_Y_LSB_REG
			mon_sensor.sensor_y = (mon_sensor.sensor_y | mon_sensor.sensor_y_l);
		}
		// Register 0x05 Out Z MSB
		if (ReadInclinometerRegister(OUT_Z_MSB_REG, &mon_sensor.sensor_z_h) == TRUE) {
			// Read OUT_Z_MSB_REG
			mon_sensor.sensor_z = (mon_sensor.sensor_z_h << 8);
		}
		// Register 0x06 Out Z LSB
		if (ReadInclinometerRegister(OUT_Z_LSB_REG, &mon_sensor.sensor_z_l) == TRUE) {
			// Read OUT_Z_LSB_REG
			mon_sensor.sensor_z = (mon_sensor.sensor_z | mon_sensor.sensor_z_l);
		}
	}

	// Read Location Data
	mon_location.latitude.data_float = ptrRAM_SystemParameters->fLatitude;
	mon_location.longitude.data_float = ptrRAM_SystemParameters->fLongitude;
	mon_location.altitude.data_float = ptrRAM_SystemParameters->fAltitude;
	mon_location.timezone.data_float = ptrRAM_SystemParameters->fTimeZone;

	// Read SPA Angle
	sun_angle.spa_azimuth = ptrRTCC_RAM_AppParameters->fSPACalculation_AZ;
	sun_angle.spa_elevation = ptrRTCC_RAM_AppParameters->fSPACalculation_EL;
	sun_angle.setPoint_az = ptrRTCC_RAM_AppParameters->fSetPoint_AZ;
	sun_angle.setPoint_el = ptrRTCC_RAM_AppParameters->fSetPoint_EL;

	// Read Tracker Angle
	tracker_angle.sun_angle = ptrRTCC_RAM_AppParameters->fSPACalculation_AZ;
	tracker_angle.target_angle = acquisitionAngle.fAzimuthTiltAngleDegrees;
	tracker_angle.tracker_angle = pgAngleAverage.fX_Angle;

	// DeadCheck
	tracker_angle.moduleTilt = acquisitionAngle.fModuleTiltAngleDegrees;
	tracker_angle.az = acquisitionAngle.fAzimuth;
	tracker_angle.el = acquisitionAngle.fElevation;
	tracker_angle.spaEL = ptrRTCC_RAM_AppParameters->fSPACalculation_EL;
	tracker_angle.tracker_y_angle = pgAngleAverage.fY_Angle;

	// Get Build Time
	ADDBUILDDateTime((char *)&buildDate);
}


void create_mon_mqtt_packet(void) {
	cJSON *root = cJSON_CreateObject();
	uint32_t uID = *((uint32_t*)0x0BFA0700);
	snprintf(nodeID, 10, "ST%X", (unsigned int)uID);	// NodeID
	cJSON_AddStringToObject(root, "NodeID", nodeID);

	// RTC
	cJSON *rtcObj = cJSON_CreateObject();
	cJSON_AddNumberToObject(rtcObj, "Day", mon_date.day);
	cJSON_AddNumberToObject(rtcObj, "Date", mon_date.date);
	cJSON_AddNumberToObject(rtcObj, "Month", mon_date.month);
	cJSON_AddNumberToObject(rtcObj, "Year", mon_date.year);
	cJSON_AddNumberToObject(rtcObj, "Hour", mon_time.hour);
	cJSON_AddNumberToObject(rtcObj, "Min", mon_time.minute);
	cJSON_AddNumberToObject(rtcObj, "Sec", mon_time.second);
	cJSON_AddItemToObject(root, "RTC", rtcObj);

	// IncSensor
	cJSON *incSensor = cJSON_CreateObject();
	cJSON *registerObj = cJSON_CreateObject();

	cJSON_AddNumberToObject(registerObj, "SENSOR_OUT_X_H", mon_sensor.sensor_x_h);
	cJSON_AddNumberToObject(registerObj, "SENSOR_OUT_X_L", mon_sensor.sensor_x_l);
	cJSON_AddNumberToObject(registerObj, "SENSOR_OUT_Y_H", mon_sensor.sensor_y_h);
	cJSON_AddNumberToObject(registerObj, "SENSOR_OUT_Y_L", mon_sensor.sensor_y_l);
	cJSON_AddNumberToObject(registerObj, "SENSOR_OUT_Z_H", mon_sensor.sensor_z_h);
	cJSON_AddNumberToObject(registerObj, "SENSOR_OUT_Z_L", mon_sensor.sensor_z_l);
#ifdef SENSOR_DATA
	cJSON_AddNumberToObject(registerObj, "Status", mon_sensor.status);
	cJSON_AddNumberToObject(registerObj, "SYSTEM_MODE", mon_sensor.sys_mode);
	cJSON_AddNumberToObject(registerObj, "INT_SOURCE_REG", mon_sensor.int_src);
	cJSON_AddNumberToObject(registerObj, "SENSOR_WHO_AM_I", mon_sensor.who_am_i);
	cJSON_AddNumberToObject(registerObj, "SENSOR_CTRL1", mon_sensor.ctrl_1);
	cJSON_AddNumberToObject(registerObj, "SENSOR_CTRL2", mon_sensor.ctrl_2);
	cJSON_AddNumberToObject(registerObj, "SENSOR_CTRL3", mon_sensor.ctrl_3);
#endif
	cJSON_AddItemToObject(incSensor, "Register", registerObj);

#ifdef LIS_DATA
	// Commenting this packet to parse at server side
	cJSON *valueObj = cJSON_CreateObject();
	cJSON_AddNumberToObject(valueObj, "LIS2HH12_OUT_X", mon_sensor.sensor_x);
	cJSON_AddNumberToObject(valueObj, "LIS2HH12_OUT_Y", mon_sensor.sensor_y);
	cJSON_AddNumberToObject(valueObj, "LIS2HH12_OUT_Z", mon_sensor.sensor_z);
	cJSON_AddItemToObject(incSensor, "Value", valueObj);
#endif
	cJSON_AddItemToObject(root, "IncSensor", incSensor);

	// Sun and Tracker Angle
	cJSON *orientationData = cJSON_CreateObject();
	char angle_buff[10]; // angle buffer
	memset(angle_buff, 0, sizeof(angle_buff));
	sprintf(angle_buff, "%0.4f", tracker_angle.moduleTilt);
	cJSON_AddStringToObject(orientationData, "Target_Tilt", angle_buff);

	memset(angle_buff, 0, sizeof(angle_buff));
	sprintf(angle_buff, "%0.4f", tracker_angle.tracker_angle);
	cJSON_AddStringToObject(orientationData, "Tracker_Angle", angle_buff);

	memset(angle_buff, 0, sizeof(angle_buff));
	sprintf(angle_buff, "%0.4f", tracker_angle.sun_angle);
	cJSON_AddStringToObject(orientationData, "Sun_Angle", angle_buff);
#ifdef TestParams
	memset(angle_buff, 0, sizeof(angle_buff));
	sprintf(angle_buff, "%0.4f", tracker_angle.tracker_y_angle);
	cJSON_AddStringToObject(orientationData, "TrackerY", angle_buff);

	memset(angle_buff, 0, sizeof(angle_buff));
	sprintf(angle_buff, "%0.4f", tracker_angle.az);
	cJSON_AddStringToObject(orientationData, "azimuth", angle_buff);

	memset(angle_buff, 0, sizeof(angle_buff));
	sprintf(angle_buff, "%0.4f", tracker_angle.el);
	cJSON_AddStringToObject(orientationData, "elevation", angle_buff);

	memset(angle_buff, 0, sizeof(angle_buff));
	sprintf(angle_buff, "%0.4f", tracker_angle.moduleTilt);
	cJSON_AddStringToObject(orientationData, "ModuleTilt", angle_buff);

	memset(angle_buff, 0, sizeof(angle_buff));
	sprintf(angle_buff, "%0.4f", tracker_angle.spaEL);
	cJSON_AddStringToObject(orientationData, "SPA_El", angle_buff);
#endif

	cJSON_AddItemToObject(root, "Orientation", orientationData);

	// Location Value
	char buff[10];// Location Value
	cJSON *locationValue = cJSON_CreateObject();
	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mon_location.latitude.data_float);
	cJSON_AddStringToObject(locationValue, "latitude", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mon_location.longitude.data_float);
	cJSON_AddStringToObject(locationValue, "longitude", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mon_location.altitude.data_float);
	cJSON_AddStringToObject(locationValue, "altitude", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mon_location.timezone.data_float);
	cJSON_AddStringToObject(locationValue, "timezone", buff);

	cJSON_AddItemToObject(root, "Location", locationValue);

	// BuildDate
	cJSON_AddStringToObject(root, "BuildDate", buildDate);

	// Convert cJSON to string
	char *jsonString = cJSON_Print(root);

	// Use jsonString as needed
	uint32_t size = strlen(jsonString);
#ifdef DEBUG
	pStatus = HAL_UART_Transmit(&hlpuart1, (uint8_t *)jsonString, size, HAL_MAX_DELAY);
	pStatus = HAL_UART_Transmit(&huart3, (uint8_t *)jsonString, size, HAL_MAX_DELAY);
#endif
	// Free cJSON objects
	cJSON_Delete(root);
}

void read_setting_params(void) {
	// Read Location Data
	mem_setting.latitude.data_float = ptrRAM_SystemParameters->fLatitude;
	mem_setting.longitude.data_float = ptrRAM_SystemParameters->fLongitude;
	mem_setting.altitude.data_float = ptrRAM_SystemParameters->fAltitude;
	mem_setting.timezone.data_float = ptrRAM_SystemParameters->fTimeZone;
	mem_setting.refraction.data_float  =ptrRAM_SystemParameters->fRefraction;
	mem_setting.tracking_mode = ptrRAM_SystemParameters->ucTracking_Mode;

	// Read Azimuth Data
	mem_setting.fAZ_Offset.data_float = ptrRAM_SystemParameters->fAZ_Offset;
	mem_setting.fAZ_SoftLimit_Forward.data_float = ptrRAM_SystemParameters->fAZ_SoftLimit_Forward;
	mem_setting.fAZ_SoftLimit_Reverse.data_float = ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse;
	mem_setting.fAZ_DeadBand.data_float = ptrRAM_SystemParameters->fAZ_DeadBand;
	mem_setting.fAZ_NightStowThreshold.data_float = ptrRAM_SystemParameters->fAZ_NightStowThreshold;
	mem_setting.fAZ_NightStowPosition.data_float = ptrRAM_SystemParameters->fAZ_NightStowPosition;
	mem_setting.fAZ_WindStowPosition.data_float = ptrRAM_SystemParameters->fAZ_WindStowPosition;

	// Read Elevation Settings
	mem_setting.fEL_Offset.data_float = ptrRAM_SystemParameters->fEL_Offset;
	mem_setting.fEL_SoftLimit_Forward.data_float = ptrRAM_SystemParameters->fEL_SoftLimit_Forward;
	mem_setting.fEL_SoftLimit_Reverse.data_float = ptrRAM_SystemParameters->fEL_SoftLimit_Reverse;
	mem_setting.fEL_DeadBand.data_float = ptrRAM_SystemParameters->fEL_DeadBand;
	mem_setting.fEL_NightStowThreshold.data_float = ptrRAM_SystemParameters->fEL_NightStowThreshold;
	mem_setting.fEL_NightStowPosition.data_float = ptrRAM_SystemParameters->fEL_NightStowPosition;
	mem_setting.fEL_WindStowPosition.data_float = ptrRAM_SystemParameters->fEL_WindStowPosition;

	// Read Single Axis Settings
	mem_setting.bBacktrackingEnabled = ptrRAM_SystemParameters->bBacktrackingEnabled;
	mem_setting.fPanelShadowStartAngleDegrees.data_float = ptrRAM_SystemParameters->fPanelShadowStartAngleDegrees;
	mem_setting.fSunShadowStartAngleDegrees.data_float = ptrRAM_SystemParameters->fSunShadowStartAngleDegrees;
	mem_setting.fSunShadowStartHeight.data_float = ptrRAM_SystemParameters->fSunShadowStartHeight;
	mem_setting.fSingle_SoftLimit_Forward.data_float = ptrRAM_SystemParameters->fSingle_SoftLimit_Forward;
	mem_setting.fSingle_SoftLimit_Reverse.data_float = ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse;
	mem_setting.fSingle_start_date.data_float = ptrRAM_SystemParameters->fSingle_start_date;
	mem_setting.fSingle_stop_days.data_float = ptrRAM_SystemParameters->fSingle_stop_days;
}

void create_read_setting_mqtt(void) {
	char buff[10];
	cJSON *root = cJSON_CreateObject();
	uint32_t uID = *((uint32_t*)0x0BFA0700);
	snprintf(nodeID, 10, "ST%X", (unsigned int)uID);	// NodeID
	cJSON_AddStringToObject(root, "NodeID", nodeID);

	cJSON_AddStringToObject(root, "Res", "read");

	// Location Value
	cJSON *locationValue = cJSON_CreateObject();
	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.latitude.data_float);
	cJSON_AddStringToObject(locationValue, "latitude", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.longitude.data_float);
	cJSON_AddStringToObject(locationValue, "longitude", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.altitude.data_float);
	cJSON_AddStringToObject(locationValue, "altitude", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.timezone.data_float);
	cJSON_AddStringToObject(locationValue, "timezone", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.refraction.data_float);
	cJSON_AddStringToObject(locationValue, "refraction", buff);


	cJSON_AddNumberToObject(locationValue, "tracking_mode", mem_setting.tracking_mode);
	cJSON_AddItemToObject(root, "Location", locationValue);


	// Azimuth Value
	cJSON *azimuthValue = cJSON_CreateObject();
	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fAZ_Offset.data_float);
	cJSON_AddStringToObject(azimuthValue, "Offset", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fAZ_SoftLimit_Reverse.data_float);
	cJSON_AddStringToObject(azimuthValue, "SoftLimit_Reverse", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fAZ_SoftLimit_Forward.data_float);
	cJSON_AddStringToObject(azimuthValue, "SoftLimit_Forward", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fAZ_DeadBand.data_float);
	cJSON_AddStringToObject(azimuthValue, "DeadBand", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fAZ_NightStowThreshold.data_float);
	cJSON_AddStringToObject(azimuthValue, "NightStowThreshold", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fAZ_NightStowPosition.data_float);
	cJSON_AddStringToObject(azimuthValue, "NightStowPosition", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fAZ_WindStowPosition.data_float);
	cJSON_AddStringToObject(azimuthValue, "WindStowPosition", buff);
	cJSON_AddItemToObject(root, "Azimuth", azimuthValue);

	// Elevation Value
	cJSON *elevationValue = cJSON_CreateObject();
	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fEL_Offset.data_float);
	cJSON_AddStringToObject(elevationValue, "Offset", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fEL_SoftLimit_Reverse.data_float);
	cJSON_AddStringToObject(elevationValue, "SoftLimit_Reverse", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fEL_SoftLimit_Forward.data_float);
	cJSON_AddStringToObject(elevationValue, "SoftLimit_Forward", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fEL_DeadBand.data_float);
	cJSON_AddStringToObject(elevationValue, "DeadBand", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fEL_NightStowThreshold.data_float);
	cJSON_AddStringToObject(elevationValue, "NightStowThreshold", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fEL_NightStowPosition.data_float);
	cJSON_AddStringToObject(elevationValue, "NightStowPosition", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fEL_WindStowPosition.data_float);
	cJSON_AddStringToObject(elevationValue, "WindStowPosition", buff);
	cJSON_AddItemToObject(root, "Elevation", elevationValue);

	// Single-Axis Value
	cJSON *singleAxisValue = cJSON_CreateObject();

	cJSON_AddNumberToObject(singleAxisValue, "BacktrackingEnabled", mem_setting.bBacktrackingEnabled);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fPanelShadowStartAngleDegrees.data_float);
	cJSON_AddStringToObject(singleAxisValue, "PanelShadowStartAngleDegrees", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fSunShadowStartAngleDegrees.data_float);
	cJSON_AddStringToObject(singleAxisValue, "SunShadowStartAngleDegrees", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fSunShadowStartHeight.data_float);
	cJSON_AddStringToObject(singleAxisValue, "SunShadowStartHeight", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fSingle_SoftLimit_Reverse.data_float);
	cJSON_AddStringToObject(singleAxisValue, "Single_SoftLimit_Reverse", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fSingle_SoftLimit_Forward.data_float);
	cJSON_AddStringToObject(singleAxisValue, "Single_SoftLimit_Forward", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fSingle_start_date.data_float);
	cJSON_AddStringToObject(singleAxisValue, "Single_start_date", buff);

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%0.4f", mem_setting.fSingle_stop_days.data_float);
	cJSON_AddStringToObject(singleAxisValue, "Single_stop_days", buff);
	cJSON_AddItemToObject(root, "SingleAxis", singleAxisValue);

	// Convert cJSON to string
	char *jsonString = cJSON_Print(root);

	// Use jsonString as needed
	uint32_t size = strlen(jsonString);
#ifdef DEBUG
	if((huart3.gState == HAL_UART_STATE_READY) && (hlpuart1.gState == HAL_UART_STATE_READY)) {
		pStatus = HAL_UART_Transmit(&hlpuart1, (uint8_t *)jsonString, size, HAL_MAX_DELAY);
		pStatus = HAL_UART_Transmit(&huart3, (uint8_t *)jsonString, size, HAL_MAX_DELAY);
	}
#endif
	// Free cJSON objects
	cJSON_Delete(root);
}


void parse_stm32_packet(char *str, uint16_t len) {
	if (!strcmp(str, "{read_id}")) {
		uint32_t uID = *((uint32_t*)0x0BFA0700);
		snprintf(nodeID, 10, "ST%X", (unsigned int)uID);	// NodeID
		HAL_UART_Transmit(&huart3, (uint8_t *)nodeID, sizeof(nodeID), HAL_MAX_DELAY);
		return;
	}
    cJSON *json = cJSON_Parse(str);
    // Check if parsing was successful
    if (json == NULL) {
    	__NOP();
    	return;
    }

    // Access specific values in the JSON structure
    cJSON *location = cJSON_GetObjectItem(json, "Location");
    cJSON *azimuth = cJSON_GetObjectItem(json, "Azimuth");
    cJSON *elevation = cJSON_GetObjectItem(json, "Elevation");
    cJSON *singleaxis = cJSON_GetObjectItem(json, "SingleAxis");
    cJSON *cmd = cJSON_GetObjectItem(json, "CMD");
#if MOTOR_TEST == 1
    cJSON *motortest = cJSON_GetObjectItem(json, "TestMotor");
#endif
    cJSON *rtc = cJSON_GetObjectItem(json, "RTC");


    if (location != NULL) {
		// Access specific values under "Location Value"
		cJSON *latitude = cJSON_GetObjectItem(location, "latitude");
		cJSON *longitude = cJSON_GetObjectItem(location, "longitude");
		cJSON *altitude = cJSON_GetObjectItem(location, "altitude");
		cJSON *timezone = cJSON_GetObjectItem(location, "timezone");
		cJSON *refraction = cJSON_GetObjectItem(location, "refraction");
		cJSON *tracking = cJSON_GetObjectItem(location, "tracking_mode");

		// Location
		if (latitude != NULL) {
			ptrRAM_SystemParameters->fLatitude = atof(latitude->valuestring);
		}

		if (longitude != NULL) {
			ptrRAM_SystemParameters->fLongitude = atof(longitude->valuestring);
		}

		if (altitude != NULL) {
			ptrRAM_SystemParameters->fAltitude = atof(altitude->valuestring);
		}

		if (timezone != NULL) {
			ptrRAM_SystemParameters->fTimeZone = atof(timezone->valuestring);
		}

		if (refraction != NULL) {
			ptrRAM_SystemParameters->fRefraction = atof(refraction->valuestring);
		}

		if (tracking != NULL) {
			ptrRAM_SystemParameters->ucTracking_Mode = tracking->valueint;
		}
		WriteFlashParameterTable();
		setting_flag = 1;
    }


    if (azimuth != NULL) {
		// Azimuth
		cJSON *az_offset = cJSON_GetObjectItem(azimuth, "Offset");
		cJSON *az_SoftLimit_Reverse = cJSON_GetObjectItem(azimuth, "SoftLimit_Reverse");
		cJSON *az_SoftLimit_Forward = cJSON_GetObjectItem(azimuth, "SoftLimit_Forward");
		cJSON *az_DeadBand = cJSON_GetObjectItem(azimuth, "DeadBand");
		cJSON *az_NightStowThreshold = cJSON_GetObjectItem(azimuth, "NightStowThreshold");
		cJSON *az_NightStowPosition = cJSON_GetObjectItem(azimuth, "NightStowPosition");
		cJSON *az_WindStowPosition = cJSON_GetObjectItem(azimuth, "WindStowPosition");
		if (az_offset != NULL) {
			ptrRAM_SystemParameters->fAZ_Offset = atof(az_offset->valuestring);
		}

		if (az_SoftLimit_Reverse != NULL) {
			ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse = atof(az_SoftLimit_Reverse->valuestring);
		}

		if (az_SoftLimit_Forward != NULL) {
			ptrRAM_SystemParameters->fAZ_SoftLimit_Forward = atof(az_SoftLimit_Forward->valuestring);
		}

		if (az_DeadBand != NULL) {
			ptrRAM_SystemParameters->fAZ_DeadBand = atof(az_DeadBand->valuestring);
		}

		if (az_NightStowThreshold != NULL) {
			ptrRAM_SystemParameters->fAZ_NightStowThreshold = atof(az_NightStowThreshold->valuestring);
		}

		if (az_NightStowPosition != NULL) {
			ptrRAM_SystemParameters->fAZ_NightStowPosition = atof(az_NightStowPosition->valuestring);
		}

		if (az_WindStowPosition != NULL) {
			ptrRAM_SystemParameters->fAZ_WindStowPosition = atof(az_WindStowPosition->valuestring);
		}
		WriteFlashParameterTable();
		setting_flag = 1;
    }


    if (elevation != NULL) {
		// Elevation
		cJSON *el_offset = cJSON_GetObjectItem(elevation, "Offset");
		cJSON *el_SoftLimit_Reverse = cJSON_GetObjectItem(elevation, "SoftLimit_Reverse");
		cJSON *el_SoftLimit_Forward = cJSON_GetObjectItem(elevation, "SoftLimit_Forward");
		cJSON *el_DeadBand = cJSON_GetObjectItem(elevation, "DeadBand");
		cJSON *el_NightStowThreshold = cJSON_GetObjectItem(elevation, "NightStowThreshold");
		cJSON *el_NightStowPosition = cJSON_GetObjectItem(elevation, "NightStowPosition");
		cJSON *el_WindStowPosition = cJSON_GetObjectItem(elevation, "WindStowPosition");
		if (el_offset != NULL) {
			ptrRAM_SystemParameters->fEL_Offset = atof(el_offset->valuestring);
		}

		if (el_SoftLimit_Reverse != NULL) {
			ptrRAM_SystemParameters->fEL_SoftLimit_Reverse = atof(el_SoftLimit_Reverse->valuestring);
		}

		if (el_SoftLimit_Forward != NULL) {
			ptrRAM_SystemParameters->fEL_SoftLimit_Forward = atof(el_SoftLimit_Forward->valuestring);
		}

		if (el_DeadBand != NULL) {
			ptrRAM_SystemParameters->fEL_DeadBand = atof(el_DeadBand->valuestring);
		}

		if (el_NightStowThreshold != NULL) {
			ptrRAM_SystemParameters->fEL_NightStowThreshold = atof(el_NightStowThreshold->valuestring);
		}

		if (el_NightStowPosition != NULL) {
			ptrRAM_SystemParameters->fEL_NightStowPosition = atof(el_NightStowPosition->valuestring);
		}

		if (el_WindStowPosition != NULL) {
			ptrRAM_SystemParameters->fEL_WindStowPosition = atof(el_WindStowPosition->valuestring);
		}

		WriteFlashParameterTable();
		setting_flag = 1;
    }

    if (singleaxis != NULL) {
		// SingleAxis
		cJSON *BacktrackingEnabled = cJSON_GetObjectItem(singleaxis, "BacktrackingEnabled");
		cJSON *PanelShadowStartAngleDegrees = cJSON_GetObjectItem(singleaxis, "PanelShadowStartAngleDegrees");
		cJSON *SunShadowStartAngleDegrees = cJSON_GetObjectItem(singleaxis, "SunShadowStartAngleDegrees");
		cJSON *SunShadowStartHeight = cJSON_GetObjectItem(singleaxis, "SunShadowStartHeight");
		cJSON *Single_SoftLimit_Reverse = cJSON_GetObjectItem(singleaxis, "Single_SoftLimit_Reverse");
		cJSON *Single_SoftLimit_Forward = cJSON_GetObjectItem(singleaxis, "Single_SoftLimit_Forward");
		cJSON *Single_start_date = cJSON_GetObjectItem(singleaxis, "Single_start_date");
		cJSON *Single_stop_days = cJSON_GetObjectItem(singleaxis, "Single_stop_days");
		if (BacktrackingEnabled != NULL) {
			ptrRAM_SystemParameters->bBacktrackingEnabled = BacktrackingEnabled->valueint;
		}

		if (PanelShadowStartAngleDegrees != NULL) {
			ptrRAM_SystemParameters->fPanelShadowStartAngleDegrees = atof(PanelShadowStartAngleDegrees->valuestring);
		}

		if (SunShadowStartAngleDegrees != NULL) {
			ptrRAM_SystemParameters->fSunShadowStartAngleDegrees = atof(SunShadowStartAngleDegrees->valuestring);
		}

		if (SunShadowStartHeight != NULL) {
			ptrRAM_SystemParameters->fSunShadowStartHeight = atof(SunShadowStartHeight->valuestring);
		}

		if (Single_SoftLimit_Reverse != NULL) {
			ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse = atof(Single_SoftLimit_Reverse->valuestring);
		}

		if (Single_SoftLimit_Forward != NULL) {
			ptrRAM_SystemParameters->fSingle_SoftLimit_Forward = atof(Single_SoftLimit_Forward->valuestring);
		}

		if (Single_start_date != NULL) {
			ptrRAM_SystemParameters->fSingle_start_date = atof(Single_start_date->valuestring);
		}

		if (Single_stop_days != NULL) {
			ptrRAM_SystemParameters->fSingle_stop_days = atof(Single_stop_days->valuestring);
		}

		WriteFlashParameterTable();
		setting_flag = 1;
    }

    if (rtc != NULL) {
		// SingleAxis
		cJSON *Day = cJSON_GetObjectItem(rtc, "Day");
		cJSON *Date = cJSON_GetObjectItem(rtc, "Date");
		cJSON *Month = cJSON_GetObjectItem(rtc, "Month");
		cJSON *Year = cJSON_GetObjectItem(rtc, "Year");
		cJSON *Hour = cJSON_GetObjectItem(rtc, "Hour");
		cJSON *Minute = cJSON_GetObjectItem(rtc, "Min");
		cJSON *Second = cJSON_GetObjectItem(rtc, "Sec");
		if (Day != NULL) {
			if (WriteRTCCRegister(DS3232_REG_DAY, Day->valueint)) {
				printf("Failed to Write Day.\n");
			}
		}

		if (Date != NULL) {
			if (WriteRTCCRegister(DS3232_REG_DATE, Date->valueint)) {
				printf("Failed to Write Date.\n");
			}
		}

		if (Month != NULL) {
			if (WriteRTCCRegister(DS3232_REG_MONTH, Month->valueint)) {
				printf("Failed to Write Month.\n");
			}
		}

		if (Year != NULL) {
			if (WriteRTCCRegister(DS3232_REG_YEAR, Year->valueint)) {
				printf("Failed to Write Year.\n");
			}
		}

		if (Hour != NULL) {
			if (WriteRTCCRegister(DS3232_REG_HOURS, Hour->valueint)) {
				printf("Failed to Write Hour.\n");
			}
		}

		if (Minute != NULL) {
			if (WriteRTCCRegister(DS3232_REG_MINUTES, Minute->valueint)) {
				printf("Failed to Write Minutes.\n");
			}
		}

		if (Second != NULL) {
			if (WriteRTCCRegister(DS3232_REG_SECONDS, Second->valueint)) {
				printf("Failed to Write Secons.\n");
			}
		}
	}

    if (cmd != NULL)
    {
    	if (!strcmp(cmd->valuestring, "read")) {
			read_setting_params();
			create_read_setting_mqtt();
		}
    }

#if MOTOR_TEST == 1
    if (motortest != NULL)
    {
    	if (!strcmp(motortest->valuestring, "L")) {
    		PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_STOPPED);
    		PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_REVERSE);
		}
    	if (!strcmp(motortest->valuestring, "R")) {
    		PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_STOPPED);
    		PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_FORWARD);
		}
    	if (!strcmp(motortest->valuestring, "S")) {
    		PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_STOPPED);
		}
    }
#endif

    // Clean up cJSON structure
    cJSON_Delete(json);
}
