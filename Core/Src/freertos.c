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
#include <stdbool.h>

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
#define CAPTURE_LOG_SIZE 1000
#define CAPTURE_BATTERY_LOG_SIZE 50 // Size of array for moving average
#define POT_HYSTERESIS_THRESHOLD 50 // 50 mv
#define POT_LOW_PASS_ALPHA 1.0f	//Responsiveness at max (range 0.0 - 1.0 higher is faster)
#define BATTERY_LOW_THRESHOLD 10.5
#define MAX_BATTERY_VOLTAGE 12.6 // 100%
#define MAX_ADC_READ 3090 // 100% ADC READ (for battery logging initialization)
#define ADC_NOISE_AVERAGE 52 // Measured average of noise when RPMS are High vs Low - adjust as needed

extern LiquidCrystal_I2C lcd;

volatile uint16_t adcBuffer[2]; // [0] used for pot, [1] for battery
volatile uint16_t potReadValue;
volatile uint16_t potPercentage;
volatile uint16_t tachReadValue;
volatile uint32_t localFanPeriod = 0;

// Logs for battery / ADC / duty cycle / PID error, etc.
volatile unsigned int captureADCLogIndex = 0;
volatile float captureADCLog[CAPTURE_BATTERY_LOG_SIZE];
volatile uint8_t captureADCLogOverflow = 0;

volatile unsigned int captureBatteryLogIndex = 0;
volatile float captureBatteryLog[CAPTURE_BATTERY_LOG_SIZE];

volatile unsigned int captureDutyCycleLogIndex = 0;
volatile uint8_t captureDutyCycleLogOverflow = 0;        // Flag to indicate buffer overflow
volatile float captureDutyCycleLog[CAPTURE_LOG_SIZE];

volatile unsigned int captureErrorLogIndex = 0;
volatile uint8_t captureErrorLogOverflow = 0;        // Flag to indicate buffer overflow
volatile float captureErrorLog[CAPTURE_LOG_SIZE];

// Shared flag for low battery voltage
volatile bool batteryLow = false;

// Rate‐limiting parameter: how many timer counts we allow the duty cycle to change per iteration
volatile float maxDeltaPerLoop = 10000.0f; // adjust to clamp over/undershoot

// Max RPM - for DBTB0428B2G - manually measured using external tach;
volatile float maxRPM = 23000.0f;

// used for PID Tuning
volatile float bestScore = -1.0f;
volatile float score = 0.0f;
volatile float testSetpoint = 0.7f; // 70% or 15400 RPM for PID tuning
volatile float testTime = 20.0f; // 15 seconds each test
// Best Scoring PID Values - Score: 38
volatile float kP = 66.0f;
volatile float kI = 7.3727f;
volatile float kD = 0.00159f;

// Piecewise samples for calculating battery life
typedef struct{
	float adcValue;
	float voltageValue;
	float batteryPercent;
} BatteryPoint;

// Empirically derived samples
static const BatteryPoint batteryTable[] = {
	//  voltage,   ADC,        approx. %
	{ 3079.90f, 12.50f,  97.0f },
	{ 3055.96f, 12.45f,  95.0f },
	{ 3046.58f, 12.40f,  93.0f },
	{ 3033.42f, 12.35f,  91.0f },
	{ 3022.80f, 12.30f,  89.0f },
	{ 3004.28f, 12.25f,  88.0f },
	{ 3000.84f, 12.20f,  85.0f },
	{ 2979.90f, 12.15f,  83.0f },
	{ 2968.90f, 12.10f,  81.0f },
	{ 2965.18f, 12.05f,  78.0f },
	{ 2943.48f, 12.00f,  76.0f },
	{ 2937.22f, 11.95f,  74.0f },
	{ 2922.16f, 11.90f,  72.0f },
	{ 2912.56f, 11.85f,  69.0f },
	{ 2901.84f, 11.80f,  67.0f },
	{ 2886.44f, 11.75f,  64.0f },
	{ 2878.52f, 11.70f,  61.0f },
	{ 2864.88f, 11.65f,  58.0f },
	{ 2846.66f, 11.60f,  53.0f },
	{ 2833.86f, 11.55f,  49.0f },
	{ 2822.10f, 11.50f,  44.0f },
	{ 2815.10f, 11.45f,  38.0f },
	{ 2802.82f, 11.40f,  32.0f },
	{ 2789.94f, 11.35f,  28.0f },
	{ 2773.90f, 11.30f,  25.0f },
	{ 2763.30f, 11.25f,  21.0f },
	{ 2752.16f, 11.20f,  18.0f },
	{ 2741.18f, 11.15f,  16.0f },
	{ 2729.64f, 11.05f,  11.0f },
	{ 2704.96f, 11.00f,   9.0f },
	{ 2696.06f, 10.95f,   8.0f },
	{ 2677.54f, 10.90f,   8.0f },
	{ 2670.00f, 10.85f,   7.0f },
	{ 2653.00f, 10.80f,   7.0f },
	{ 2643.52f, 10.75f,   6.0f },
	{ 2637.84f, 10.70f,   6.0f },
	{ 2604.76f, 10.60f,   6.0f },
	{ 2587.14f, 10.50f,   5.0f },
	{ 2563.54f, 10.40f,   4.0f }
};
static const int batteryTableCount = sizeof(batteryTable) / sizeof(batteryTable[0]);

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
  .stack_size = 256 * 4,
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

/**
  * @brief  Initializes the battery logging array with a default ADC value
  *         (e.g. MAX_ADC_READ), helping avoid false "low battery" on startup.
  *         Called once at system init if needed.
  */
void InitializeBatteryMonitoringLog()
{
	for (int i = 0; i < CAPTURE_BATTERY_LOG_SIZE; i++)
	{
		captureBatteryLog[i] = MAX_ADC_READ;
	}
}


/**
  * @brief  Applies hysteresis + low-pass filtering on the raw pot ADC reading.
  *         Returns a smoothed millivoltage. Minimizes jitter but stays responsive.
  *
  * @param  newReading   Raw ADC reading: range [0..4095].
  * @retval float        Filtered reading in millivolts [0..3300].
  */
float ProcessPotentiometerReading(float newReading)
{
	static float potLastAcceptedValue = 0.0f;
	static float potFilteredValue = 0.0f;
	newReading = (newReading / 4095.0f) * 3300.0f;

	// Apply Hysteresis to ignore small changes
	if(fabs(newReading - potLastAcceptedValue) > POT_HYSTERESIS_THRESHOLD)
	{
		potLastAcceptedValue = newReading;
	}

	// Apply Low-pass filter for smoothing
	potFilteredValue = potFilteredValue + POT_LOW_PASS_ALPHA * (potLastAcceptedValue - potFilteredValue);
	return potFilteredValue;
}


/**
  * @brief  Averages the battery ADC readings from captureBatteryLog to produce a stable value.
  *
  * @retval float   The moving-average ADC reading (in raw ADC counts).
  */
float GetBatteryReadingADC()
{
	static int count = 0;
	count = count < CAPTURE_BATTERY_LOG_SIZE ? count + 1 : CAPTURE_BATTERY_LOG_SIZE; // Increment up to buffer size

	float sum = 0.0f;
	for(int i = 0; i < count; i++)
	{
		sum += captureBatteryLog[i];
	}
	float currentACDReading = sum / count;
	return currentACDReading;
}

/**
  * @brief  Converts the average ADC reading into an approximate battery voltage (Volts),
  *         using piecewise linear interpolation. Reference batteryTable for samples.
  *
  * @retval float  Approximate battery voltage (e.g. 12.45).
  */
float GetBatteryReadingVolts()
{
	float adcValue = GetBatteryReadingADC();
	// If the measured ADC is >= max voltage, clamp:
	if(adcValue >= batteryTable[0].adcValue)
	{
		return (float)MAX_BATTERY_VOLTAGE; // return max 12.6 volts
	}

	// If the measured ADC is <= min voltage, clamp:
	if(adcValue <= batteryTable[batteryTableCount - 1].adcValue)
	{
		return batteryTable[batteryTableCount -1].voltageValue;
	}

	// Run through samples to determine slope and intersect
	for(int i = 0; i < batteryTableCount - 1; i++)
	{
		float adcHigh = batteryTable[i].adcValue;
		float adcLow = batteryTable[i+1].adcValue;

		if(adcValue <= adcHigh && adcValue >=adcLow)
		{
			// Correct segment located from the table
			// Linearly interpolate between adcHigh and adcLow
			float voltageHigh = batteryTable[i].voltageValue;
			float voltageLow = batteryTable[i+1].voltageValue;

			// Fraction along the segment
			float fraction = (adcValue - adcLow) / (adcHigh - adcLow);

			// Interpolate
			float estimatedVoltage = voltageLow + fraction * (voltageHigh - voltageLow);

			return estimatedVoltage;
		}
	}
	// should never get here due to clamping - avoids compile warnings
	return batteryTable[batteryTableCount - 1].voltageValue;
}



/**
  * @brief  Converts the average ADC reading into an integer percentage [0..100] by
  *         linearly interpolating the raw ADC reading in batteryTable.

  * @retval uint8_t  The battery percentage, 0..100.
  */
uint8_t GetBatteryLifePercent()
{
	float adcValue = GetBatteryReadingADC();
		// If the measured ADC is >= max percentage, clamp:
		if(adcValue >= batteryTable[0].adcValue)
		{
			return 100; // return 100% battery life
		}

		// If the measured ADC is <= min voltage, clamp:
		if(adcValue <= batteryTable[batteryTableCount - 1].adcValue)
		{
			return batteryTable[batteryTableCount -1].batteryPercent;
		}

		// Run through samples to determine slope and intersect
		for(int i = 0; i < batteryTableCount - 1; i++)
		{
			float adcHigh = batteryTable[i].adcValue;
			float adcLow = batteryTable[i+1].adcValue;

			if(adcValue <= adcHigh && adcValue >=adcLow)
			{
				// Correct segment located from the table
				// Linearly interpolate between adcHigh and adcLow
				float percentHigh = batteryTable[i].batteryPercent;
				float percentLow = batteryTable[i+1].batteryPercent;

				// Fraction along the segment
				float fraction = (adcValue - adcLow) / (adcHigh - adcLow);

				// Interpolate
				uint8_t batteryPercentage = (int)round(percentLow + fraction * (percentHigh - percentLow));

				return batteryPercentage;
			}
		}
		// should never get here due to clamping - avoids compile warnings
		return batteryTable[batteryTableCount - 1].voltageValue;
}

/**
  * @brief  Returns the battery reading in millivolts, derived from GetBatteryReadingVolts().
  *
  * @retval float  The battery voltage in millivolts (e.g., 12540.0).
  */
float GetBatteryReadingMillivolts()
{
	float volts = GetBatteryReadingVolts();
	return (volts * 1000.0f);  // returns ~12570.0
}

/**
  * @brief  Logs a new raw battery ADC reading into captureBatteryLog for averaging, then
  *         checks if the battery voltage is below BATTERY_LOW_THRESHOLD to set batteryLow.
  *
  * @param  newReading: The raw ADC reading for battery channel [0..4095].
  */
void ProcessBatteryReading(float newReading)
{
	captureBatteryLog[captureBatteryLogIndex] = newReading;
	captureBatteryLogIndex = (captureBatteryLogIndex +1 ) % CAPTURE_BATTERY_LOG_SIZE;
	// If Battery is low throttle power down
	if(GetBatteryReadingVolts() < BATTERY_LOW_THRESHOLD)
	{
		// Indicate battery low to other tasking
		if(osMutexAcquire(dataMutexHandle, osWaitForever) == osOK)
		{
			batteryLow = true;
			osMutexRelease(dataMutexHandle);
		}
	}
	else
	{
		// Indicate battery not yet low to other tasking
		if(osMutexAcquire(dataMutexHandle, osWaitForever) == osOK)
		{
			batteryLow = false;
			osMutexRelease(dataMutexHandle);
		}
	}
}

/**
  * @brief  Updates the globally used PID gains kP, kI, kD for fan control.
  *
  * @param  kp, ki, kd : Gains used in the PID control loop
  */
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
  *
  * @retval float: Score achieved by this these PID values - score represents time spent
  *    		  	   at set point within threshold
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

	/*
	// OPTIONAL AUTO-TUNING PROCEDURE: comment out if not used
	// used for PID tuning (15400)
	testSetpoint = 0.7f * maxRPM;
	// Arrays of Kp, Ki, Kd to try:
	// Kp to try
	float possibleKp[] = {10.0f,15.0f,66.0f,304.5f};
	// Ki to try
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
		potValue   = potPercentage;   // e.g. 0–100
		currentRPM = tachReadValue;  // measured from tach
		osMutexRelease(dataMutexHandle);

		// If Battery is low throttle power down
		if(batteryLow)
		{
			// Set Duty cycle to zero
			scaledDuty = 0.0f;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			// Skip all PID logic
			lastDuty = 0.0f;
			osDelay(pdMS_TO_TICKS(10));
			continue;
		}

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
  float batteryVoltageReading = 0.0f;
  uint16_t localPotPercentage = 0;

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
			  // Get the potentiometer return voltage
			  potVoltage_mv = (float)adcBuffer[0]; // Read potentiometer voltage
			  // Get the battery reading voltage
			  batteryVoltageReading = (float)adcBuffer[1];
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

	  // Process Potentiometer Reading
	  potVoltage_mv = ProcessPotentiometerReading(potVoltage_mv);
	  // Derive potentiometer turn percentage
	  localPotPercentage = round((potVoltage_mv / 3200) * 100);
	  if(localPotPercentage > 100){localPotPercentage = 100;}
	  if(localPotPercentage < 0){localPotPercentage = 0;}

	  // Apply offset for observed noise in the system
	  float calculatedNoiseOffset =  ADC_NOISE_AVERAGE * (1.0f - ((float)localPotPercentage / 100));
	  batteryVoltageReading += calculatedNoiseOffset;

	  // Store Pot Values
	  if(osMutexAcquire(dataMutexHandle, osWaitForever) == osOK)
	  {
		  // Variables are ready for updates
		  potReadValue = potVoltage_mv;
		  potPercentage = localPotPercentage;
		  osMutexRelease(dataMutexHandle);
	  }

	  // Process battery readings
	  ProcessBatteryReading(batteryVoltageReading);

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
	uint16_t targetFanRPM = 0;
	// For recharge / power throttle
	TickType_t lastBlinkTime = 0;
	bool blinkState = false;
	bool isBatteryLow = false;

	lcd_clear(&lcd);

	for(;;)
	{
		// Acquire blink state - blinking every 500 ms
		TickType_t now = xTaskGetTickCount();
		if((now - lastBlinkTime) >= pdMS_TO_TICKS(500))
		{
			blinkState = !blinkState;
			lastBlinkTime = now;
		}

		// Read shared variables
		osMutexAcquire(dataMutexHandle, osWaitForever);
		currentFanRPM = tachReadValue;
		isBatteryLow = batteryLow;
		// Convert the potValue to a target RPM
		targetFanRPM = (potPercentage / 100.0f) * maxRPM;
		osMutexRelease(dataMutexHandle);

		if(isBatteryLow)
		{
			//Line 1: "Power Low: XX.Xv"
			lcd_clear(&lcd); // clear each line
			snprintf(line1, sizeof(line1), "Power Low: %.2fv", GetBatteryReadingVolts());
			lcd_setCursor(&lcd,0,0);
			lcd_print(&lcd, line1);

			//Line 2: blink "PLEASE RECHARGE!"
			lcd_setCursor(&lcd,0,1);
			if(blinkState)
			{
				snprintf(line2,sizeof(line2), "PLEASE RECHARGE!");
				lcd_print(&lcd, line2);
			}
			else
			{
				//Just print blank for blinky affect ( • ᴗ - )
				lcd_print(&lcd, "                ");
			}

			// skip the rest of the loop
			osDelay(25);
			continue;
		}

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
		snprintf(line2, sizeof(line2), "Power:%.2fv/%2u%%", GetBatteryReadingVolts(), GetBatteryLifePercent());
		lcd_setCursor(&lcd, 0, 1);
		lcd_print(&lcd, line2);
		/* End Normal Operation output */

		osDelay(25); // Adjust as needed
	}
  /* USER CODE END StartUpdateLCDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

