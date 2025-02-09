/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "stdint.h"
#include "b_l475e_iot01a2_bus.h"
#include "lsm6dsl.h"
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
LSM6DSL_Object_t MovementDetector;
volatile uint32_t DrdyUpdate_1;
volatile uint32_t DrdyUpdate_2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MEMS_Init(void);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DrdyUpdate_1 = 0;
  DrdyUpdate_2 = 0;
  MEMS_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  if (DrdyUpdate_1 != 0) {
	      DrdyUpdate_1 = 0;
	      LSM6DSL_Axes_t axes;
	      LSM6DSL_ACC_GetAxes(&MovementDetector, &axes);
	      printf("Acc-Axes-X:%5d,   Acc-Axes-Y:%5d,   Acc-Axes-Z:%5d\r\n",
	             (int) axes.x, (int) axes.y, (int) axes.z);
	      HAL_Delay(30);
	  }

/*	Gyroscope Function; before Activation please enable it in USER CODE 4

	  if (DrdyUpdate_2 != 0) {
	      DrdyUpdate_2 = 0;
	      LSM6DSL_Axes_t axes;
	      LSM6DSL_GYRO_GetAxes(&MovementDetector, &axes);
	      printf("GYRO-Axes-P:%5d,   GYRO-Axes-R:%5d,   GYRO-Axes-Y:%5d\r\n",
	             (int) axes.x, (int) axes.y, (int) axes.z);
	      HAL_Delay(30);
	  }
*/

	  }
  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM6DSL_INT1_EXTI11_Pin */
  GPIO_InitStruct.Pin = LSM6DSL_INT1_EXTI11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void MEMS_Init(void)

{

  LSM6DSL_IO_t IO;
  uint8_t ID;
  LSM6DSL_AxesRaw_t axes_raw;

  /* Link I2C functions to the LSM6DSL driver */
  IO.BusType     = LSM6DSL_I2C_BUS;
  IO.Address     = LSM6DSL_I2C_ADD_L;
  IO.Init        = BSP_I2C2_Init;
  IO.DeInit      = BSP_I2C2_DeInit;
  IO.ReadReg     = BSP_I2C2_ReadReg;
  IO.WriteReg    = BSP_I2C2_WriteReg;
  IO.GetTick     = BSP_GetTick;

  LSM6DSL_RegisterBusIO(&MovementDetector, &IO);

  /* Read the LSM6DSL WHO_AM_I register */
  LSM6DSL_ReadID(&MovementDetector, &ID);

  if (ID != LSM6DSL_ID) {

    Error_Handler();

  }

  /* Initialize the LSM6DSL sensor */
  LSM6DSL_Init(&MovementDetector);

  /* Configure the LSM6DSL accelerometer (ODR, scale and interrupt) */
  LSM6DSL_ACC_SetOutputDataRate(&MovementDetector, 1.6f); /* 26 Hz */
  LSM6DSL_ACC_SetFullScale(&MovementDetector, 4);          /* [-4000mg; +4000mg] */
  LSM6DSL_ACC_Set_INT1_DRDY(&MovementDetector, ENABLE);    /* Enable DRDY */
  LSM6DSL_ACC_GetAxesRaw(&MovementDetector, &axes_raw);        /* Clear DRDY */

  /* Configure the LSM6DSL Gyroscope (ODR, scale and interrupt) */
  LSM6DSL_GYRO_SetOutputDataRate(&MovementDetector, 12.5f); /* 52 Hz */
  LSM6DSL_GYRO_SetFullScale(&MovementDetector, 250);        /* [-250dps; 250dps] */
  LSM6DSL_GYRO_Set_INT1_DRDY(&MovementDetector, ENABLE);    /* Enable DRDY */
  LSM6DSL_GYRO_GetAxesRaw(&MovementDetector, &axes_raw);        /* Clear DRDY */

  /* Start the LSM6DSL accelerometer */
  LSM6DSL_ACC_Enable(&MovementDetector);

  /* Start the LSM6DSL Gyroscope */
  //LSM6DSL_GYRO_Enable(&MovementDetector);

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

        if (GPIO_Pin == GPIO_PIN_11) {
            DrdyUpdate_1++;
            DrdyUpdate_2++;
        }

}



int _write(int fd, char *ptr, int len)
{
    while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        // Wait for the previous transmission to complete
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
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
