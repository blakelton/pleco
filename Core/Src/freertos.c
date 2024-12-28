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
#include <stdlib.h>
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
uint32_t localFanPeriod = 0;
#define CAPTURE_LOG_SIZE 1000
volatile unsigned int captureDutyCycleLogIndex = 0;
volatile uint8_t captureDutyCycleLogOverflow = 0;        // Flag to indicate buffer overflow
volatile float captureDutyCycleLog[CAPTURE_LOG_SIZE];
volatile unsigned int captureErrorLogIndex = 0;
volatile uint8_t captureErrorLogOverflow = 0;        // Flag to indicate buffer overflow
volatile float captureErrorLog[CAPTURE_LOG_SIZE];
// Rate‐limiting parameter: how many timer counts we allow the duty cycle to change per iteration
float maxDeltaPerLoop = 10000.0f; // adjust as needed

// Max RPM - for DBTB0428B2G - manually measured using external tach;
float maxRPM = 22000.0f;

// used for PID Tuning
float percentOnTarget = 0.0f;
// Best Scoring PID Values - Score: 38
volatile float kP = 66.0f;
volatile float kI = 7.3727f;
volatile float kD = 0.00159f;
float bestScore = -1.0f;
float score = 0.0f;
float testSetpoint = 0.7f; // 70% or 15400 RPM for PID tuning
float testTime = 20.0f; // 15 seconds each test


/* USER CODE END Variables */
/* Definitions for FanControlTask */
osThreadId_t FanControlTaskHandle;
const osThreadAttr_t FanControlTask_attributes = {
  .name = "FanControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MonitorADCTask */
osThreadId_t MonitorADCTaskHandle;
const osThreadAttr_t MonitorADCTask_attributes = {
  .name = "MonitorADCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MonitorTachTask */
osThreadId_t MonitorTachTaskHandle;
const osThreadAttr_t MonitorTachTask_attributes = {
  .name = "MonitorTachTask",
  .stack_size = 256 * 4,
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
/* Definitions for tachMonSem */
osSemaphoreId_t tachMonSemHandle;
const osSemaphoreAttr_t tachMonSem_attributes = {
  .name = "tachMonSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void SetPIDGains(float kp, float ki, float kd)
{
    kP = kp;
    kI = ki;
    kD = kd;
}

/**
  * @brief  Runs a mini PID loop at the specified (testKp, testKi, testKd)
  *         for the specified duration. It drives the fan (PWM) using
  *         scaledDuty calls, measures performance, then returns a "score"
  *         (e.g., fraction of time on target).
  *
  * @param  testKp, testKi, testKd : Gains to apply temporarily
  * @param  testTimeSeconds        : How long (in seconds) to run the test
  * @param  setPointRPM            : Desired target RPM for the test
  * @return float                  : The measured performance score
  */
float RunPIDTest(float testKp, float testKi, float testKd)
{
    // 1) Save original gains so we can restore later
    float originalKp = kP;
    float originalKi = kI;
    float originalKd = kD;

    // 2) Apply the test gains
    SetPIDGains(testKp, testKi, testKd);

    /************************************************************
     * 3) WARM-UP PHASE
     ************************************************************/
    const TickType_t warmUpTicks = pdMS_TO_TICKS(2000); // e.g., 3 seconds
    TickType_t startTicks = xTaskGetTickCount();
    TickType_t warmUpEnd  = startTicks + warmUpTicks;

    const TickType_t stepDelay = pdMS_TO_TICKS(10); // 10 ms loop
    float dT = 0.01f; // 10 ms in float seconds

    float pidIntegral    = 0.0f;
    float previousError  = 0.0f;
    float pidOutput      = 0.0f;
    float scaledDuty     = 0.0f;

    // Keep track of the previous duty cycle for "soft clamp" & rate limit
    float lastDuty       = 0.0f;

    while (xTaskGetTickCount() < warmUpEnd)
    {
        // 3A) Read current RPM
        osMutexAcquire(dataMutexHandle, portMAX_DELAY);
        uint16_t currRPM = tachReadValue;
        osMutexRelease(dataMutexHandle);

        // 3B) Compute error
        float error = testSetpoint - currRPM;

        // 3C) Basic PID
        pidIntegral       += (error * dT);
        float pidDerivative = (error - previousError) / dT;
        pidOutput          = (kP * error) + (kI * pidIntegral) + (kD * pidDerivative);
        previousError      = error;

        // 3D) Convert to duty cycle
        float pwmMaxValue = (float)__HAL_TIM_GET_AUTORELOAD(&htim1);
        scaledDuty = (pidOutput / maxRPM) * pwmMaxValue;

        // 3E) "Soft clamp" approach for negative outputs
        if (scaledDuty < 0.0f)
        {
            float overshootMagnitude  = -scaledDuty;  // how negative we went
            float reductionFraction   = 0.80f;         // tweak as needed
            scaledDuty = lastDuty - (overshootMagnitude * reductionFraction);

            if (scaledDuty < 0.0f)
                scaledDuty = 0.0f;
        }

        // 3F) Also clamp if above max
        if (scaledDuty > pwmMaxValue)
            scaledDuty = pwmMaxValue;

        // >>>> NEW RATE‐LIMITING BLOCK <<<<
        float delta = scaledDuty - lastDuty;
        if (delta >  maxDeltaPerLoop)  delta =  maxDeltaPerLoop;
        if (delta < -maxDeltaPerLoop)  delta = -maxDeltaPerLoop;
        scaledDuty = lastDuty + delta;
        // (Now clamp again if it drifted below 0 or above max)
        if (scaledDuty < 0.0f)         scaledDuty = 0.0f;
        if (scaledDuty > pwmMaxValue)  scaledDuty = pwmMaxValue;
        // <<<< END RATE‐LIMITING BLOCK >>>>

        // 3G) Apply the scaled duty
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)scaledDuty);

        // 3H) Save duty for next iteration
        lastDuty = scaledDuty;

        // 3I) Wait a bit
        vTaskDelay(stepDelay);
    }

    /************************************************************
     * 4) MEASUREMENT PHASE (accumulate error metrics)
     ************************************************************/
    uint32_t onTargetCount    = 0;
    uint32_t offTargetCount   = 0;
    float absoluteErrorSum     = 0.0f;

    float threshold           = 300.0f;  // ±300 RPM
    TickType_t measureTicks   = (TickType_t)(testTime * 1000 / portTICK_PERIOD_MS);
    TickType_t measureStart   = xTaskGetTickCount();
    TickType_t measureEnd     = measureStart + measureTicks;

    // Optionally reset integrator at start of measurement
    // pidIntegral = 0.0f;

    while (xTaskGetTickCount() < measureEnd)
    {
        // 4A) Read current RPM
        osMutexAcquire(dataMutexHandle, portMAX_DELAY);
        uint16_t currRPM = tachReadValue;
        osMutexRelease(dataMutexHandle);

        // 4B) Compute error
        float error = testSetpoint - currRPM;
        absoluteErrorSum += fabsf(error) * dT;

        // 4C) On-target or off-target
        if (fabsf(error) <= threshold)
            onTargetCount++;
        else
            offTargetCount++;

        // 4D) PID calculations
        pidIntegral       += (error * dT);
        float pidDerivative = (error - previousError) / dT;
        pidOutput          = (kP * error) + (kI * pidIntegral) + (kD * pidDerivative);
        previousError      = error;

        // Log the current capture value for analyzing tach data
        captureErrorLog[captureDutyCycleLogIndex] = error;  // Write to the buffer
        captureErrorLogIndex++;
        if (captureErrorLogIndex >= CAPTURE_LOG_SIZE)
        {
            captureErrorLogIndex = 0;
            captureErrorLogOverflow = 1; // Indicate overflow
        }

        // 4E) Convert to duty cycle
        float pwmMaxValue = (float)__HAL_TIM_GET_AUTORELOAD(&htim1);
        scaledDuty        = (pidOutput / maxRPM) * pwmMaxValue;

        // 4F) "Soft clamp" again for negative
        if (scaledDuty < 0.0f)
        {
            float overshootMagnitude  = -scaledDuty;
            float reductionFraction   = 0.5f;
            scaledDuty = lastDuty - (overshootMagnitude * reductionFraction);

            if (scaledDuty < 0.0f)
                scaledDuty = 0.0f;
        }

        if (scaledDuty > pwmMaxValue)
            scaledDuty = pwmMaxValue;

        // >>>> RATE‐LIMITING AGAIN <<<<
        float delta = scaledDuty - lastDuty;
        if (delta >  maxDeltaPerLoop)  delta =  maxDeltaPerLoop;
        if (delta < -maxDeltaPerLoop)  delta = -maxDeltaPerLoop;
        scaledDuty = lastDuty + delta;
        // final clamp
        if (scaledDuty < 0.0f)         scaledDuty = 0.0f;
        if (scaledDuty > pwmMaxValue)  scaledDuty = pwmMaxValue;
        // <<<< END RATE‐LIMITING BLOCK >>>>

		 // Log the current capture value for analyzing tach data
       captureDutyCycleLog[captureDutyCycleLogIndex] = scaledDuty;  // Write to the buffer
       captureDutyCycleLogIndex++;                            // Increment the index

		if (captureDutyCycleLogIndex >= CAPTURE_LOG_SIZE)      // Handle buffer overflow
		{
			captureDutyCycleLogIndex = 0;                      // Reset index (circular buffer)
			captureDutyCycleLogOverflow = 1;                   // Indicate overflow
		}

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)scaledDuty);
        lastDuty = scaledDuty;

        vTaskDelay(stepDelay);
    }

    /************************************************************
     * 5) Convert the error metrics into a final "score"
     ************************************************************/
    float totalSamples = (float)(onTargetCount + offTargetCount);
    float fractionOnTarget = 0.0f;
    if (totalSamples > 0.0f)
        fractionOnTarget = (float)onTargetCount / totalSamples;

    float penalty = 1.0f + absoluteErrorSum / 100000.0f;
    float localScore = fractionOnTarget / penalty;

    /************************************************************
     * 6) Restore original gains
     ************************************************************/
    SetPIDGains(originalKp, originalKi, originalKd);

    return localScore;
}


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

  /* creation of tachMonSem */
  tachMonSemHandle = osSemaphoreNew(1, 1, &tachMonSem_attributes);

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

	// PID variables
	float previousError = 0.0f;
	float dT = 0.01f; // Assumes this tasks runs every 10ms
	float setPoint = 0.0f;
	float error = 0.0f;
	float pidIntegral = 0.0f;
	float pidDerivative = 0.0f;
	float pidOutput = 0.0f;
	float scaledDuty = 0.0f;
	float lastDuty       = 0.0f; // Keeps track of the previous duty cycle for "soft clamp" & rate limit
	uint16_t potValue = 0; // Get the latest potentiometer value (0-100%)
	uint16_t currentRPM = 0;


	// used for PID tuning (15400)
	testSetpoint = 0.7f * maxRPM;

	/*
	// OPTIONAL AUTO-TUNING PROCEDURE: comment out if not used
	// Arrays of Kp, Ki, Kd to try:
	// Kp to try
	float possibleKp[] = {10.0f,15.0f,66.0f,304.5f};
	// Ki to try
	//float possibleKi[] = {0.0f};6.5536f,
	float possibleKi[] = {0.4096f, 0.8192f};
	//float possibleKd[] = {0.0f};
	float possibleKd[] = {0.0008f, 0.0016f, 0.0032f, 0.0064f, 0.0128f, 0.0256f};

	float bestKp   = kP;
	float bestKi   = kI;
	float bestKd   = kD;

	// Loop over possible gains:
	for (size_t i=0; i< (sizeof(possibleKp)/sizeof(possibleKp[0])); i++)
	{
		for (size_t j=0; j< (sizeof(possibleKi)/sizeof(possibleKi[0])); j++)
		{
			for (size_t k=0; k< (sizeof(possibleKd)/sizeof(possibleKd[0])); k++)
			{
			  float kpVal = possibleKp[i];
			  float kiVal = possibleKi[j];
			  float kdVal = possibleKd[k];

			  score = RunPIDTest(kpVal, kiVal, kdVal);
			  // Compare:
			  if (score > bestScore)
			  {
				  bestScore = score;
				  bestKp    = kpVal;
				  bestKi    = kiVal;
				  bestKd    = kdVal;
			  }
			}
		}
	}

	// When tuning place a break point here to see best values
	SetPIDGains(bestKp, bestKi, bestKd);
	// *** (END of auto-tuning procedure) ****/

	for(;;)  // Main fan control loop
	{
		// Acquire the mutex to safely access potReadValue, tachReadValue
		osMutexAcquire(dataMutexHandle, osWaitForever);
		potValue   = potReadValue;   // e.g. 0–100
		currentRPM = tachReadValue;  // measured from tach
		osMutexRelease(dataMutexHandle);

		// Convert potValue% to a target RPM in [0, maxRPM]
		setPoint = (potValue / 100.0f) * maxRPM;

		// Calculate PID Error
		error = setPoint - currentRPM;

		// PID Calculations
		pidIntegral   += error * dT;
		pidDerivative  = (error - previousError) / dT;
		pidOutput      = (kP * error) + (kI * pidIntegral) + (kD * pidDerivative);

		previousError  = error; // store for next iteration

		// Convert pidOutput to duty cycle
		scaledDuty = (pidOutput / maxRPM) * (float)pwmMaxValue;

		// ============= 1) “Soft clamp” approach for negative outputs =============
		if (scaledDuty < 0.0f)
		{
		  float overshootMagnitude = -scaledDuty;   // how negative
		  float reductionFraction   = 0.80f;         // tweak as needed
		  scaledDuty = lastDuty - (overshootMagnitude * reductionFraction);

		  if (scaledDuty < 0.0f)
			scaledDuty = 0.0f;
		}

		// ============= 2) Hard clamp if above max =============
		if (scaledDuty > pwmMaxValue) scaledDuty = pwmMaxValue;

		// ============= 3) Rate‐limiting code =============
		float delta = scaledDuty - lastDuty;
		if (delta >  maxDeltaPerLoop)   delta =  maxDeltaPerLoop;
		if (delta < -maxDeltaPerLoop)   delta = -maxDeltaPerLoop;
		scaledDuty = lastDuty + delta;

		// Final clamp again if it ended up outside [0, pwmMaxValue]
		if (scaledDuty < 0.0f) scaledDuty = 0.0f;
		if (scaledDuty > pwmMaxValue) scaledDuty = pwmMaxValue;

		// 4) Apply the final scaled duty
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)scaledDuty);

		// 5) Save for next loop iteration
		lastDuty = scaledDuty;

		// Delay ~10 ms
		osDelay(pdMS_TO_TICKS(10));
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
	// Timer frequency in Hz (1Mhz with 63 prescaler)
	const float timerFreq = 1000000.0f;
	float localFrequency = 0.0f;
	const float pulsesPerRev = 2.0f;
	uint16_t rpm = 0;
	/* Infinite loop */
	for(;;)
	{
		// Get the current time and the last update time first
		TickType_t currentTime = xTaskGetTickCount();

		// Check how long since the last pulse
		TickType_t timeSinceLastPulse = currentTime - lastTachUpdateTime;

		// If more than 1000ms have passed without a new pulse, fan is effectively stopped
		if (timeSinceLastPulse > pdMS_TO_TICKS(1000))
		{
			rpm = 0;
			// Update shared variable for RPM
			if(osMutexAcquire(dataMutexHandle, osWaitForever) == osOK)
			{
				tachReadValue = rpm;
				osMutexRelease(dataMutexHandle);
			}
		}
		else
		{
			// Check semaphore if read task is available from tach callback
			if(xSemaphoreTake(tachMonSemHandle, pdMS_TO_TICKS(100)) == pdTRUE)
			{
				// Capture event has occurred - compute new RPM reading
				localFanPeriod = fanPeriodTicks;
				if(localFanPeriod > 0)
				{
					// Calculate RPM
					localFrequency = timerFreq / (float)localFanPeriod;
					rpm = (uint16_t)(localFrequency * ((float)60 / pulsesPerRev));
				}
				else
				{
					// If not data - RPM is likely zero
					rpm = 0;
				}

				//Store the RPM as a shared variable
				if(osMutexAcquire(dataMutexHandle, osWaitForever) == osOK)
				{
					tachReadValue = rpm;
					osMutexRelease(dataMutexHandle);
				}
			}
		}


	osDelay(pdMS_TO_TICKS(5));
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

  lcd_clear(&lcd);

  for(;;)
  {
	// Read shared variables
	osMutexAcquire(dataMutexHandle, osWaitForever);
	currentFanRPM = tachReadValue;
	currentBatteryVoltage = batteryReadValue;
	//targetFanRPM = potReadValue;
	// Convert the potValue to a target RPM
	targetFanRPM = (potReadValue / 100.0f) * maxRPM;
	osMutexRelease(dataMutexHandle);

	// Calculate battery life - based on 12v 3s lipo battery
	batteryLifeRemaining = round(((currentBatteryVoltage - 9000) / 3600) * 100);
	currentBatteryVoltage = currentBatteryVoltage / 1000; //Convert mv to v for easy read
	currentBatteryVoltage = (float)((int)(currentBatteryVoltage * 10)) / 10.0f; // trim to tenth place

	/* Use for PID tuning - comment out when not tuning
	uint16_t localBestScore = roundf(bestScore * 100);
	uint16_t localScore = roundf(score * 100);
	snprintf(line1, sizeof(line1), "%5u-S:%d/BS:%d", currentFanRPM, localScore, localBestScore);
	lcd_setCursor(&lcd, 0, 0);
	lcd_print(&lcd, line1);
	snprintf(line2, sizeof(line2), "p%.0fi%.3fd%.3f", kP, kI, kD);
	lcd_setCursor(&lcd, 0, 1);
	lcd_print(&lcd, line2);
	// End PID Tuning output */

	/* Normal Operation output */
	// Update LCD with new data
	snprintf(line1, sizeof(line1), "RPM: %5u/%5u", currentFanRPM, targetFanRPM);
	lcd_setCursor(&lcd, 0, 0);
	lcd_print(&lcd, line1);
	snprintf(line2, sizeof(line2), "Power: %.1fv/%2u%%", currentBatteryVoltage, batteryLifeRemaining);
	lcd_setCursor(&lcd, 0, 1);
	lcd_print(&lcd, line2);
	/* End Normal Operation output */

	osDelay(25 ); // Adjust as needed
  }
  /* USER CODE END StartUpdateLCDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

