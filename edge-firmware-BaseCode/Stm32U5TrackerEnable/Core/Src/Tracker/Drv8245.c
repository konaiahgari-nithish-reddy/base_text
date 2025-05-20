#include "stm32u5xx_hal.h"
#include "PeripheralCallbacks.h"
#include "main.h"
#include "Drv8245.h"
#include "dwt_delay.h"

extern TIM_HandleTypeDef htim2;
void delay_us(uint32_t us);
/*void WakeUpDrv8245(void)
{
	HAL_GPIO_WritePin(GPIOA, DRIVE_DRVOFF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, DRIVE_nSLEEP_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	while(HAL_GPIO_ReadPin(GPIOA, DRIVE_nFAULT_Pin) != GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, DRIVE_nSLEEP_Pin, GPIO_PIN_RESET);
//	DWT_Delay(10);
//	while(HAL_GPIO_ReadPin(GPIOA, DRIVE_nFAULT_Pin) != GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, DRIVE_nSLEEP_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, DRIVE_DRVOFF_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, DRIVE_EN_IN1_TIM_CH2_Pin, GPIO_PIN_SET);
}*/

void WakeUpDrv8245(void)
  {


  	HAL_GPIO_WritePin(GPIOA, DRIVE_DRVOFF_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(GPIOA, DRIVE_nSLEEP_Pin, GPIO_PIN_SET);
  	HAL_Delay(1);
//  	while(HAL_GPIO_ReadPin(GPIOA, DRIVE_nFAULT_Pin) != GPIO_PIN_RESET);	Saurabh
  	if (HAL_GPIO_ReadPin(GPIOA, DRIVE_nFAULT_Pin) != GPIO_PIN_RESET) {

  	}
  	HAL_GPIO_WritePin(GPIOA, DRIVE_nSLEEP_Pin, GPIO_PIN_RESET);
  	DWT_Delay(10);
  	//while(HAL_GPIO_ReadPin(GPIOA, DRIVE_nFAULT_Pin) != GPIO_PIN_SET);
  	HAL_GPIO_WritePin(GPIOA, DRIVE_nSLEEP_Pin, GPIO_PIN_SET);
  	HAL_Delay(10);
  	HAL_GPIO_WritePin(GPIOA, DRIVE_DRVOFF_Pin, GPIO_PIN_RESET);
  	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, DRIVE_EN_IN1_TIM_CH2_Pin, GPIO_PIN_SET);

  }

/*void SetPins(void)
{
	if (HAL_GPIO_ReadPin(GPIOA, DRIVE_DRVOFF_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(GPIOA, DRIVE_DRVOFF_Pin, GPIO_PIN_RESET);

	if (HAL_GPIO_ReadPin(GPIOA, DRIVE_nSLEEP_Pin) == GPIO_PIN_RESET)
		HAL_GPIO_WritePin(GPIOA, DRIVE_nSLEEP_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, DRIVE_PH_IN2_TIM_CH1_Pin, GPIO_PIN_SET);
}*/

void SetPins(void)
{
	if (HAL_GPIO_ReadPin(GPIOA, DRIVE_DRVOFF_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(GPIOA, DRIVE_DRVOFF_Pin, GPIO_PIN_RESET);

	if (HAL_GPIO_ReadPin(GPIOA, DRIVE_nSLEEP_Pin) == GPIO_PIN_RESET)
		HAL_GPIO_WritePin(GPIOA, DRIVE_nSLEEP_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, DRIVE_PH_IN2_TIM_CH1_Pin, GPIO_PIN_SET);

}

void EnableDrv8245(void)
{
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Reverse or opposite direction (PH/IN2)
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Forward direction (EN/IN1)

	//HAL_GPIO_WritePin(GPIOA, DRIVE_EN_IN1_TIM_CH2_Pin, GPIO_PIN_RESET);
}
void SetMotor(uint8_t Direction)
{
	  if(Direction == 1){
		  //HAL_GPIO_WritePin(GPIOA, DRIVE_PH_IN2_TIM_CH1_Pin, GPIO_PIN_SET);
		  //HAL_GPIO_WritePin(GPIOA, DRIVE_EN_IN1_TIM_CH2_Pin, GPIO_PIN_RESET);
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Reverse or opposite direction (PH/IN2)
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Forward direction (EN/IN1)
	  }
	  else{
		  //HAL_GPIO_WritePin(GPIOA, DRIVE_PH_IN2_TIM_CH1_Pin, GPIO_PIN_RESET);
		  //HAL_GPIO_WritePin(GPIOA, DRIVE_EN_IN1_TIM_CH2_Pin, GPIO_PIN_SET);
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); // Forward direction (EN/IN1)
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Reverse or opposite direction (PH/IN2)
	  }
}

/*
void SetPH(void)
{
	HAL_GPIO_WritePin(GPIOA, Drive_PH_IN2_Pin, GPIO_PIN_SET);
}

void ResetPH(void)
{
	HAL_GPIO_WritePin(GPIOA, Drive_PH_IN2_Pin, GPIO_PIN_RESET);
}

void EnableDrv8245(void)
{
	HAL_GPIO_WritePin(GPIOA, Drive_EN_IN1_Pin, GPIO_PIN_SET);
}
*/

/*void EnableDrv8245(void)
{
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Reverse or opposite direction (PH/IN2)
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Forward direction (EN/IN1)

	  HAL_GPIO_WritePin(GPIOA, DRIVE_EN_IN1_TIM_CH2_Pin, GPIO_PIN_SET);
}*/

/*void SetMotor(uint8_t Direction)
{
	if(Direction == 1)
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
	else
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 25);
}*/

void SetDutyCycleForward(uint32_t DutyCycle)
{
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Reverse or opposite direction (PH/IN2)
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Forward direction (EN/IN1)
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, DutyCycle);
//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, DutyCycle);
}

void SetDutyCycleReverse(uint32_t DutyCycle)
{
//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, DutyCycle);

	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); // Forward direction (EN/IN1)
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Reverse or opposite direction (PH/IN2)
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, DutyCycle);
}


void StopMotor(uint32_t DutyCycle)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, DutyCycle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, DutyCycle);
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Reverse or opposite direction (PH/IN2)
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); // Forward direction (EN/IN1)
//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, DutyCycle);
//	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, DutyCycle);
}
