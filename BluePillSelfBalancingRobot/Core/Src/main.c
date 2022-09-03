/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*MPU6050*/
#define MPU_DEVICE_ADDRESS 				(0xD0)

#define MPU_WHO_AM_I_REG				(0x75)
#define MPU_WHO_AM_I_SUCCESS_VALUE 		(0x68)

#define MPU_POWER_MANAGEMENT_ONE_REG	(0x6B)
#define MPU_WAKE_UP_VALUE				(uint8_t)0x0

#define MPU_SAMPLE_RATE_DIV_REG			(0x19)
#define MPU_DATA_RATE					(uint8_t)0x7

#define MPU_GYRO_CONFIG_REG				(0x1B)
#define MPU_GYRO_CONFIG_VALUE			(uint8_t)0x0

#define MPU_ACCEL_CONFIG_REG			(0x1c)
#define MPU_ACCEL_CONFIG_VALUE			(uint8_t)0x0

#define MPU_ACCEL_DATA_START 			(0x3B)
#define MPU_ACCEL_DATA_CONVERSION_FACT	(16384.0f)

#define MPU_GYRO_DATA_START				(0x43)
#define MPU_GYRO_DATA_CONVERSION_FACT	(131.0f)

#define MPU_FILTERING_CONFIG_REG		(0x1A)
#define MPU_FILTERING_CONFIG_VALUE 		(0x03)

#define GYRO_TRAVELED_1MS_FACT			(1.0f/(1000.0f * MPU_GYRO_DATA_CONVERSION_FACT))


#define RAD_TO_DEG						(57.295779513082320876798154814105f)
/*STEPPER-MOTORS*/
#define DIRECTION_PORT_LEFT				GPIOA
#define DIRECTION_PIN_LEFT				GPIO_PIN_3

#define STEP_PORT_LEFT					GPIOA
#define STEP_PIN_LEFT					GPIO_PIN_4

#define DIRECTION_PORT_RIGHT			GPIOA
#define DIRECTION_PIN_RIGHT				GPIO_PIN_1

#define STEP_PORT_RIGHT					GPIOA
#define STEP_PIN_RIGHT					GPIO_PIN_2

#define STEPPER_DELAY_MIN 			(400u)
#define STEPPER_DELAY_MAX			(10000u)
#define MAX_ANGLE_FOR_REACTION		(30.0f)

#define STEPPER_SPEED_STEP			((STEPPER_DELAY_MAX - STEPPER_DELAY_MIN) / 100u)

#define STEPPER_ERROR_STEP			(100u / MAX_ANGLE_FOR_REACTION)

#define PID_OUTPUT_RATE				(7000.0f - (-7000.0f))
#define PID_STEP_RATE				(100 / 7000.0f)
/*LCD*/
#define LCD_WRITE_DATA					(0x80)

#define LCD_CURSOR_ROW_ONE_COLUMN_ONE 	(0x00)
#define LCD_CURSOR_ROW_TWO_COLUMN_ONE	(0x40)

/*PID*/
#define ACCEL_BALANCE_VALUE 			(14900)
#define RAD_PER_SEC_TRAVELED_4MS		(0.000031f)

#define MAXIMUM_CONTROLLER_VALUE 		(400)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/*MPU6050*/
uint8_t errorBlink500ms = 0;

float accelData[3] = {0};
float gyroData[3] = {0};
uint8_t accelRaw[6] = {0};
int16_t accelRawValues[3] = {0};
uint8_t gyroRaw[6] = {0};
int16_t gyroRawValues[3] = {0};

volatile float angleFromAccel = 0.0f;
volatile float gyroAngle = 0.0f;

float gyroCalibrationValuePitch = 0.0f;
float gyroCalibrationValueYaw 	= 0.0f;

/*PID*/
float Kp = 1;
float Kd = 0;
float Ki = 0;

uint8_t balance = 0;

volatile float selfBalancePIDSetpoint = 0.0f;
volatile float errorTempPID = 0.0f;
volatile float integralMemoryPID = 0.0f;
volatile float setpointPID = 0.0f;
volatile float gyroInput = 0.0f;
volatile float outputPIDValue = 0.0f;
volatile float derivativeErrorPreviousPID = 0.0f;

volatile float leftStepperOutputPID = 0.0f;
volatile float rightStepperOutputPID = 0.0f;

volatile int32_t leftStepperOutput = 0;
volatile int32_t rightStepperOutput = 0;;

volatile int32_t leftStepperOutputMemory = 0;
volatile int32_t rightStepperOutputMemory = 0;
volatile uint32_t leftStepperOutputCount = 0;
volatile uint32_t rightStepperOutputCount = 0;

volatile int32_t leftStepperPulseControl = 0;
volatile int32_t rightStepperPulseControl = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void microDelay (uint16_t delay);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/*MPU6050*/
uint8_t MPU6050_Init(void);
void MPU6050_ReadAccelRaw();
void MPU6050_ConvertRawToGravityConstantValues();
void MPU6050_GetAccelData();
void MPU6050_ReadGyroRaw();
void MPU6050_ConvertRawToDegPerSecValues();
void MPU6050_GetGyroData();
void MPU6050_GyroInitializationCalibration();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  lcd_init();
  lcd_send_cmd(LCD_WRITE_DATA|LCD_CURSOR_ROW_ONE_COLUMN_ONE);
  lcd_send_string("Initialized!");
  lcd_send_cmd(LCD_WRITE_DATA|LCD_CURSOR_ROW_TWO_COLUMN_ONE);
  lcd_send_string("LCD");
  HAL_Delay(1000);
  uint8_t success = MPU6050_Init();
  if(!success){
	  lcd_send_cmd(LCD_WRITE_DATA|LCD_CURSOR_ROW_ONE_COLUMN_ONE);
	  lcd_send_string("Init Failed");
	  lcd_send_cmd(LCD_WRITE_DATA|LCD_CURSOR_ROW_TWO_COLUMN_ONE);
	  lcd_send_string("MPU6050 (-1)");
	  errorBlink500ms = 1;

  } else{
	  lcd_send_cmd(LCD_WRITE_DATA|LCD_CURSOR_ROW_ONE_COLUMN_ONE);
	  lcd_send_string("Initialized!");
	  lcd_send_cmd(LCD_WRITE_DATA|LCD_CURSOR_ROW_TWO_COLUMN_ONE);
	  lcd_send_string("MPU6050");
	  HAL_Delay(1000);
	  lcd_clear();
	  MPU6050_GyroInitializationCalibration();
	  lcd_clear();

	  HAL_TIM_Base_Start_IT(&htim2);
	  HAL_TIM_Base_Start_IT(&htim3);
	}
		//microDelay(83000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(errorBlink500ms){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_Delay(500);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		  HAL_Delay(500);

	  } else{
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_Delay(4000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		  HAL_Delay(4000);
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 144-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 79;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//REFACTOR EVERYTHING HERE
	if (htim->Instance == TIM2) {
		//4ms main looping
		 //__HAL_TIM_SET_COUNTER(&htim1, 0);
		 MPU6050_GetAccelData();
		 accelRawValues[2] -= ACCEL_BALANCE_VALUE;
		 if(accelRawValues[2] > 8200){
			 accelRawValues[2]  = 8200;
		 }
		 if(accelRawValues[2] < -8200){
			 accelRawValues[2] = -8200;
		 }

		 angleFromAccel = asin((float)accelRawValues[2]/8200.0) * RAD_TO_DEG;

		 if(balance == 0 && angleFromAccel > -0.5f && angleFromAccel < 0.5f){
			 balance = 1;
			 gyroAngle = angleFromAccel;
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			 //errorBlink500ms = 0;
		 }

		 MPU6050_GetGyroData();
		 gyroRawValues[1] -= gyroCalibrationValuePitch;
		 gyroAngle += gyroRawValues[1] * RAD_PER_SEC_TRAVELED_4MS;

		 gyroRawValues[2] -= gyroCalibrationValueYaw;

		 gyroAngle = gyroAngle * 0.9995 + angleFromAccel * 0.0005;

		 /*PID calculations*/

		 errorTempPID = gyroAngle - selfBalancePIDSetpoint - setpointPID;
		 if (outputPIDValue > 10  || outputPIDValue < -10){
			 errorTempPID += outputPIDValue * 0.015;
		 }

		 integralMemoryPID += Ki * errorTempPID;
		 if(integralMemoryPID > MAXIMUM_CONTROLLER_VALUE){
			 integralMemoryPID = MAXIMUM_CONTROLLER_VALUE;
		 } else if (integralMemoryPID < -MAXIMUM_CONTROLLER_VALUE){
			 integralMemoryPID = -MAXIMUM_CONTROLLER_VALUE;
		 }

		 outputPIDValue = Kp * errorTempPID + integralMemoryPID + Kd * (errorTempPID - derivativeErrorPreviousPID);
		 if(outputPIDValue > MAXIMUM_CONTROLLER_VALUE){
			 outputPIDValue = MAXIMUM_CONTROLLER_VALUE;
		 } else if(outputPIDValue < -MAXIMUM_CONTROLLER_VALUE){
			 outputPIDValue = -MAXIMUM_CONTROLLER_VALUE;
		 }

		 derivativeErrorPreviousPID = errorTempPID;

		 if(outputPIDValue < 5 && outputPIDValue > -5){
			 outputPIDValue = 0;
		 }

		 if(gyroAngle > MAX_ANGLE_FOR_REACTION || gyroAngle < -MAX_ANGLE_FOR_REACTION || balance == 0 ){
			 outputPIDValue = 0;
			 balance = 0;
			 integralMemoryPID = 0;
			 selfBalancePIDSetpoint = 0;
		 }

		 leftStepperOutputPID = outputPIDValue;
		 rightStepperOutputPID = outputPIDValue;

		 /*Controller Calculations*/
		 /*TODO implement controller controls*/

		 if(setpointPID > 0.5){
			 setpointPID -=0.05;
		 } else if(setpointPID < -0.5){
			 setpointPID += 0.05;
		 } else{
			 setpointPID = 0;
		 }

		 if (setpointPID == 0){
			 if(outputPIDValue < 0 ){
				 selfBalancePIDSetpoint += 0.0015;
			 }
			 if (outputPIDValue > 0){
				 selfBalancePIDSetpoint -= 0.0015;
			 }
		 }

		 /*Motor Step Calculations*/
		 /*Compensating for non-linear behaviour of stepper motors to get a linear speed response*/
		 if(leftStepperOutputPID > 0){
			 leftStepperOutputPID = 405 - (1/(leftStepperOutputPID + 9)) * 5500;
		 } else if(leftStepperOutputPID  < 0){
			 leftStepperOutputPID = -405 - (1/(leftStepperOutputPID - 9)) * 5500;
		 }

		 if(rightStepperOutputPID > 0){
			 rightStepperOutputPID = 405 - (1/(rightStepperOutputPID + 9)) * 5500;
		 } else if (rightStepperOutputPID < 0){
			 rightStepperOutputPID =  -405 - (1/(rightStepperOutputPID - 9)) * 5500;
		 }

		 /*Calculate step time*/
		 if(leftStepperOutputPID > 0){
			 leftStepperOutput = MAXIMUM_CONTROLLER_VALUE - leftStepperOutputPID;
		 } else if(leftStepperOutputPID < 0){
			 leftStepperOutput = -MAXIMUM_CONTROLLER_VALUE - leftStepperOutputPID;
		 } else{
			 leftStepperOutput = 0;
		 }

		 if(rightStepperOutputPID > 0){
			 rightStepperOutput = MAXIMUM_CONTROLLER_VALUE - rightStepperOutputPID;
		 } else if(rightStepperOutputPID < 0){
			 rightStepperOutput = -MAXIMUM_CONTROLLER_VALUE - rightStepperOutputPID;
		 } else{
			 rightStepperOutput = 0;
		 }

		 leftStepperPulseControl = leftStepperOutput;
		 rightStepperPulseControl = rightStepperOutput;

  	} else if(htim->Instance == TIM3){
  		/*Step pulse generation - every 20us*/
  		//Right Stepper
  		rightStepperOutputCount++;
  		if(rightStepperOutputCount > rightStepperOutputMemory){
  			rightStepperOutputMemory = rightStepperPulseControl;
  			rightStepperOutputCount = 0;
  			if(rightStepperOutputMemory < 0){
  				GPIOA->BSRR |= (DIRECTION_PIN_RIGHT);
  				//HAL_GPIO_WritePin(DIRECTION_PORT_RIGHT, DIRECTION_PIN_RIGHT, GPIO_PIN_SET);
  				rightStepperOutputMemory *= -1;
  			} else{
  				GPIOA->BSRR |= (DIRECTION_PIN_RIGHT) << 16;
  				//HAL_GPIO_WritePin(DIRECTION_PORT_RIGHT, DIRECTION_PIN_RIGHT, GPIO_PIN_RESET);
  			}
  		} else if(rightStepperOutputCount == 1){
  			//HAL_GPIO_WritePin(STEP_PORT_RIGHT, STEP_PIN_RIGHT, GPIO_PIN_SET);
  			GPIOA->BSRR |= (STEP_PIN_RIGHT);
  		} else if(rightStepperOutputCount == 2){
  			//HAL_GPIO_WritePin(STEP_PORT_RIGHT, STEP_PIN_RIGHT, GPIO_PIN_RESET);
  			GPIOA->BSRR |= (STEP_PIN_RIGHT) << 16;
  		}
  		// Left Stepper
		leftStepperOutputCount++;
		if(leftStepperOutputCount > leftStepperOutputMemory){
			leftStepperOutputMemory = leftStepperPulseControl;
			leftStepperOutputCount = 0;
			if(leftStepperOutputMemory < 0){
				GPIOA->BSRR |= (DIRECTION_PIN_LEFT)  << 16;
				//HAL_GPIO_WritePin(DIRECTION_PORT_LEFT, DIRECTION_PIN_LEFT, GPIO_PIN_RESET);
				leftStepperOutputMemory *= -1;
			} else{
				GPIOA->BSRR |= (DIRECTION_PIN_LEFT);
				//HAL_GPIO_WritePin(DIRECTION_PORT_LEFT, DIRECTION_PIN_LEFT, GPIO_PIN_SET);
			}
		} else if(leftStepperOutputCount == 1){
			//HAL_GPIO_WritePin(STEP_PORT_LEFT, STEP_PIN_LEFT, GPIO_PIN_SET);
			GPIOA->BSRR |= (STEP_PIN_LEFT);
		} else if(leftStepperOutputCount == 2){
			//HAL_GPIO_WritePin(STEP_PORT_LEFT, STEP_PIN_LEFT, GPIO_PIN_RESET);
			GPIOA->BSRR |= (STEP_PIN_LEFT) << 16;
		}
  	}
}

/*MPU6050*/
uint8_t MPU6050_Init(void){
	uint8_t dataBuffer;
	//WHO AM I check
	HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS, MPU_WHO_AM_I_REG, 1, &dataBuffer, 1, 1000);
	if (dataBuffer != MPU_WHO_AM_I_SUCCESS_VALUE){
		for(int i = 0; i<3; i++){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_Delay(1000);
		}

		return 0u;

	} else{
		//Write all 0's to Power Management 1 register to wake the sensor up
		dataBuffer = MPU_WAKE_UP_VALUE;
		HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, MPU_POWER_MANAGEMENT_ONE_REG, 1, &dataBuffer, 1, 1000);

		//Set Data Rate
		dataBuffer = MPU_DATA_RATE;
		HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, MPU_SAMPLE_RATE_DIV_REG, 1, &dataBuffer, 1, 1000);

		//Configure GYRO (+- 250 deg/s)
		dataBuffer = MPU_GYRO_CONFIG_VALUE;
		HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, MPU_GYRO_CONFIG_REG, 1, &dataBuffer, 1, 1000);

		//Configure ACCEL (+- 2g)
		dataBuffer = MPU_ACCEL_CONFIG_VALUE;
		HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, MPU_ACCEL_CONFIG_REG, 1, &dataBuffer, 1, 1000);

		//Improve raw data by setting up filtering
		dataBuffer = MPU_FILTERING_CONFIG_VALUE;
		HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, MPU_FILTERING_CONFIG_REG, 1, &dataBuffer, 1, 1000);
		return 1u;

	}
}
void MPU6050_ReadAccelRaw(){

	//Read Accel Data for all 3 axis (2 bytes per axis)

	HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS, MPU_ACCEL_DATA_START, 1, accelRaw, 6, 1000);


	accelRawValues[0] = (((uint16_t)accelRaw[0] << 8u) | accelRaw[1]);
	accelRawValues[1] = (((uint16_t)accelRaw[2] << 8u) | accelRaw[3]);
	accelRawValues[2] = (((uint16_t)accelRaw[4] << 8u) | accelRaw[5]);

}

void MPU6050_ConvertRawToGravityConstantValues(){

	accelData[0] = accelRawValues[0] / MPU_ACCEL_DATA_CONVERSION_FACT;
	accelData[1] = accelRawValues[1] / MPU_ACCEL_DATA_CONVERSION_FACT;
	accelData[2] = accelRawValues[2] / MPU_ACCEL_DATA_CONVERSION_FACT;

}

void MPU6050_GetAccelData(){
	MPU6050_ReadAccelRaw();
	MPU6050_ConvertRawToGravityConstantValues();

}

void MPU6050_ReadGyroRaw(){
	HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS, MPU_GYRO_DATA_START, 1, gyroRaw, 6, 1000);


	gyroRawValues[0] = (((uint16_t)gyroRaw[0] << 8u) | gyroRaw[1]);
	gyroRawValues[1] = (((uint16_t)gyroRaw[2] << 8u) | gyroRaw[3]);
	gyroRawValues[2] = (((uint16_t)gyroRaw[4] << 8u) | gyroRaw[5]);

}

void MPU6050_ConvertRawToDegPerSecValues(){
	gyroData[0] = gyroRawValues[0] / MPU_GYRO_DATA_CONVERSION_FACT;
	gyroData[1] = gyroRawValues[1] / MPU_GYRO_DATA_CONVERSION_FACT;
	gyroData[2] = gyroRawValues[2] / MPU_GYRO_DATA_CONVERSION_FACT;

}

void MPU6050_GetGyroData(){

	MPU6050_ReadGyroRaw();
	MPU6050_ConvertRawToDegPerSecValues();

}


void MPU6050_GyroInitializationCalibration(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	int calCounter = 0;
	lcd_send_cmd (0x80|0x00);
	lcd_send_string("Calibrating Gyro");
	lcd_send_cmd (0x80|0x40);
	for(calCounter = 0; calCounter < 1000; calCounter++){
		if(calCounter % 100 == 0){
			lcd_send_string(".");
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		}
		MPU6050_GetGyroData();
		gyroCalibrationValueYaw += gyroRawValues[2];
		gyroCalibrationValuePitch += gyroRawValues[1];
		microDelay(3000);
	}


	gyroCalibrationValuePitch /= 1000;
	gyroCalibrationValueYaw /=  1000;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

}

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay){
	  //wait until timer counter goes to desired delay
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
