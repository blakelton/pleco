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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FanControlTask */
osThreadId_t FanControlTaskHandle;
const osThreadAttr_t FanControlTask_attributes = {
  .name = "FanControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCDUpdateTask */
osThreadId_t LCDUpdateTaskHandle;
const osThreadAttr_t LCDUpdateTask_attributes = {
  .name = "LCDUpdateTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RPMCalcTask */
osThreadId_t RPMCalcTaskHandle;
const osThreadAttr_t RPMCalcTask_attributes = {
  .name = "RPMCalcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for BatterySenseTas */
osThreadId_t BatterySenseTasHandle;
const osThreadAttr_t BatterySenseTas_attributes = {
  .name = "BatterySenseTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartFanControlTask(void *argument);
void StartLCDUpdateTask(void *argument);
void StartRPMCalcTask(void *argument);
void StartBatterySenseTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of FanControlTask */
  FanControlTaskHandle = osThreadNew(StartFanControlTask, NULL, &FanControlTask_attributes);

  /* creation of LCDUpdateTask */
  LCDUpdateTaskHandle = osThreadNew(StartLCDUpdateTask, NULL, &LCDUpdateTask_attributes);

  /* creation of RPMCalcTask */
  RPMCalcTaskHandle = osThreadNew(StartRPMCalcTask, NULL, &RPMCalcTask_attributes);

  /* creation of BatterySenseTas */
  BatterySenseTasHandle = osThreadNew(StartBatterySenseTask, NULL, &BatterySenseTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartFanControlTask */
/**
* @brief Function implementing the FanControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFanControlTask */
void StartFanControlTask(void *argument)
{
  /* USER CODE BEGIN StartFanControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartFanControlTask */
}

/* USER CODE BEGIN Header_StartLCDUpdateTask */
/**
* @brief Function implementing the LCDUpdateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDUpdateTask */
void StartLCDUpdateTask(void *argument)
{
  /* USER CODE BEGIN StartLCDUpdateTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLCDUpdateTask */
}

/* USER CODE BEGIN Header_StartRPMCalcTask */
/**
* @brief Function implementing the RPMCalcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRPMCalcTask */
void StartRPMCalcTask(void *argument)
{
  /* USER CODE BEGIN StartRPMCalcTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRPMCalcTask */
}

/* USER CODE BEGIN Header_StartBatterySenseTask */
/**
* @brief Function implementing the BatterySenseTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBatterySenseTask */
void StartBatterySenseTask(void *argument)
{
  /* USER CODE BEGIN StartBatterySenseTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBatterySenseTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

