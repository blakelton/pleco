/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "i2c.h"
#include "semphr.h"
#include "liquidcrystal_i2c.h"
#include "tim.h"
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t adcBuffer[2]; // [0] used for pot, [1] for battery
uint16_t potReadValue;
uint16_t batteryReadValue;
uint16_t tachReadValue;
extern LiquidCrystal_I2C lcd;


/* USER CODE END Variables */

/* Definitions for FanControlTask */
osThreadId_t FanControlTaskHandle;
const osThreadAttr_t FanControlTask_attributes = {
  .name = "FanControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MonitorADCTask */
osThreadId_t MonitorADCTaskHandle;
const osThreadAttr_t MonitorADCTask_attributes = {
  .name = "MonitorADCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for MonitorTachTask */
osThreadId_t MonitorTachTaskHandle;
const osThreadAttr_t MonitorTachTask_attributes = {
  .name = "MonitorTachTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UpdateLCDTask */
osThreadId_t UpdateLCDTaskHandle;
const osThreadAttr_t UpdateLCDTask_attributes = {
  .name = "UpdateLCDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for dataMutex */
osMutexId_t dataMutexHandle;
const osMutexAttr_t dataMutex_attributes = {
  .name = "dataMutex"
};
/* Definitions for adcConvSem */
osSemaphoreId_t adcConvSemHandle;
const osSemaphoreAttr_t adcConvSem_attributes = {
  .name = "adcConvSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartFanControlTask(void *argument);
void StartMonitorADCTask(void *argument);
void StartMonitorTachTask(void *argument);
void StartUpdateLCDTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of dataMutex */
  dataMutexHandle = osMutexNew(&dataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of adcConvSem */
  adcConvSemHandle = osSemaphoreNew(1, 1, &adcConvSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of FanControlTask */
  FanControlTaskHandle = osThreadNew(StartFanControlTask, NULL, &FanControlTask_attributes);

  /* creation of MonitorADCTask */
  MonitorADCTaskHandle = osThreadNew(StartMonitorADCTask, NULL, &MonitorADCTask_attributes);

  /* creation of MonitorTachTask */
  MonitorTachTaskHandle = osThreadNew(StartMonitorTachTask, NULL, &MonitorTachTask_attributes);

  /* creation of UpdateLCDTask */
  UpdateLCDTaskHandle = osThreadNew(StartUpdateLCDTask, NULL, &UpdateLCDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartFanControlTask */
/**
  * @brief  Function implementing the FanControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartFanControlTask */
void StartFanControlTask(void *argument)
{
	/* USER CODE BEGIN StartFanControlTask */
	const uint32_t pwmMaxValue = __HAL_TIM_GET_AUTORELOAD(&htim1); // Max value for the PWM duty cycle (timer auto-reload value)

	// Start the PWM output on the desired channel (ensure this is done once)
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	for(;;)
	{
		// Acquire the mutex to safely access potReadValue
		osMutexAcquire(dataMutexHandle, osWaitForever);
		uint16_t potValue = potReadValue; // Get the latest potentiometer value (0-100%)
		osMutexRelease(dataMutexHandle);

		// Convert the percentage to a duty cycle value
		uint32_t dutyCycle = (uint32_t)((potValue / 100.0f) * pwmMaxValue);

		// Set the PWM duty cycle
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyCycle);

		osDelay(100); // Adjust as necessary
	}
	/* USER CODE END StartFanControlTask */

}

/* USER CODE BEGIN Header_StartMonitorADCTask */
/**
* @brief Function implementing the MonitorADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorADCTask */
void StartMonitorADCTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorADCTask */
  float potVoltage_mv = 0.0;
  uint16_t potPercentage = 0;
  float batteryVoltage_mv = 0.0;
  /* Infinite loop */
  for(;;)
  {
	  //Start ADC DMA Conversion
	  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 2) == HAL_OK)
	  {
		  // Wait for conversion to complete
		  if(xSemaphoreTake(adcConvSemHandle, pdMS_TO_TICKS(100)) == pdTRUE)
		  {
			  // Conversion is complete, buffer data is ready
			  //potVoltage_mv = ((float)adcBuffer[0] / 4095.0f) * 3600.0f;
			  potVoltage_mv = ((float)adcBuffer[0] / 4095.0f) * 3300.0f; // Read potentiometer voltage
			  potPercentage = round((potVoltage_mv / 3200) * 100);
			  if(potPercentage > 100){potPercentage = 100;}
			  if(potPercentage < 0){potPercentage = 0;}

			  batteryVoltage_mv = ((float)adcBuffer[1] / 4095.0f) * 3300.0f * BATTERY_SCALING_FACTOR;
			  if(osMutexAcquire(dataMutexHandle, osWaitForever) == osOK)
			  {
				  // Variables are ready for updates
				  potReadValue = potPercentage;
				  batteryReadValue = batteryVoltage_mv;
				  osMutexRelease(dataMutexHandle);
			  }
		  }
		  else
		  {
			  //ADC Conversion did not complete in time.
		      // TODO: Handle error if needed
		  }
	  }
	  else
	  {
		  // ADC DMA failed to start
		  // TODO: Handle error if needed
	  }

    osDelay(100);
  }
  /* USER CODE END StartMonitorADCTask */
}

/* USER CODE BEGIN Header_StartMonitorTachTask */
/**
* @brief Function implementing the MonitorTachTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTachTask */
void StartMonitorTachTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorTachTask */
  /* Infinite loop */
  for(;;)
  {


    osDelay(1);
  }
  /* USER CODE END StartMonitorTachTask */
}

/* USER CODE BEGIN Header_StartUpdateLCDTask */
/**
* @brief Function implementing the UpdateLCDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUpdateLCDTask */
void StartUpdateLCDTask(void *argument)
{
  /* USER CODE BEGIN StartUpdateLCDTask */
  char line1[17];
  char line2[17];
  uint16_t currentFanRPM = 0;
  float currentBatteryVoltage = 0;
  uint16_t batteryLifeRemaining = 0;
  uint16_t targetFanRPM = 0;

  for(;;)
  {
	// Read shared variables
	osMutexAcquire(dataMutexHandle, osWaitForever);
	currentFanRPM = tachReadValue;
	currentBatteryVoltage = batteryReadValue;
	targetFanRPM = potReadValue;
	osMutexRelease(dataMutexHandle);

	// Calculate battery life - based on 12v 3s lipo battery
	batteryLifeRemaining = round(((currentBatteryVoltage - 9000) / 3600) * 100);
	currentBatteryVoltage = currentBatteryVoltage / 1000; //Convert mv to v for easy read
	currentBatteryVoltage = (float)((int)(currentBatteryVoltage * 10)) / 10.0f; // trim to tenth place

	// Update LCD with new data
	snprintf(line1, sizeof(line1), "RPM: %5u/%5u", currentFanRPM, targetFanRPM);
	lcd_setCursor(&lcd, 0, 0);
	lcd_print(&lcd, line1);
	snprintf(line2, sizeof(line2), "Power: %.1f/%u%%", currentBatteryVoltage, batteryLifeRemaining);

	lcd_setCursor(&lcd, 0, 1);
	lcd_print(&lcd, line2);

	osDelay(100); // Adjust as needed
  }
/* USER CODE END StartTaskLCDUpdateTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

