/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "can_it.h"
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

/* USER CODE BEGIN PV */
extern uint8_t RXData[8];

typedef struct GYRO_DATA
{
	int8_t
	acc_x1,acc_x2,
	acc_y1,acc_y2,
	acc_z1,acc_z2,
	gyr_x1,gyr_x2,
	gyr_y1,gyr_y2,
	gyr_z1,gyr_z2,
	ang_x1,ang_x2,
	ang_y1,ang_y2,
	ang_z1,ang_z2;

	int16_t
	acc_X,acc_Y,acc_Z,
	gyr_X,gyr_Y,gyr_Z,
	ang_X,ang_Y,ang_Z;
} GYRO_DATA;

GYRO_DATA CAN_VALUE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  CAN_RX_Header_defunc();
  CAN_TX_Header_defunc();
  CAN_Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(CAN_RXH.StdId==1)
	  	  {
	  		  if(RXData[1]==0x33) //acc
	  		  	  {
	  		  		 CAN_VALUE.acc_x1=RXData[2];
	  		  		 CAN_VALUE.acc_x2=RXData[3];
	  		  		 CAN_VALUE.acc_y1=RXData[4];
	  		  		 CAN_VALUE.acc_y2=RXData[5];
	  		  		 CAN_VALUE.acc_z1=RXData[6];
	  		  		 CAN_VALUE.acc_z2=RXData[7];

	  				 CAN_VALUE.acc_X = (CAN_VALUE.acc_x2 << 8) + (uint8_t)CAN_VALUE.acc_x1;
	  			     CAN_VALUE.acc_Y = (CAN_VALUE.acc_y2 << 8) + (uint8_t)CAN_VALUE.acc_y1;
	  			  	 CAN_VALUE.acc_Z = (CAN_VALUE.acc_z2 << 8) + (uint8_t)CAN_VALUE.acc_z1;

	  		  		 /*CAN_VALUE.acc_X = (CAN_VALUE.acc_x2 << 8) | CAN_VALUE.acc_x1;
	  		  		 CAN_VALUE.acc_Y = (CAN_VALUE.acc_y2 << 8) | CAN_VALUE.acc_y1;
	  		  		 CAN_VALUE.acc_Z = (CAN_VALUE.acc_z2 << 8) | CAN_VALUE.acc_z1;*/
	  		  	  }
	  		  else if(RXData[1]==0x34) // gyro
	  		  	  {
	  			     CAN_VALUE.gyr_x1=RXData[2];
	  				 CAN_VALUE.gyr_x2=RXData[3];
	  				 CAN_VALUE.gyr_y1=RXData[4];
	  				 CAN_VALUE.gyr_y2=RXData[5];
	  				 CAN_VALUE.gyr_z1=RXData[6];
	  				 CAN_VALUE.gyr_z2=RXData[7];

	  				 CAN_VALUE.gyr_X = (CAN_VALUE.gyr_x2 << 8) + (uint8_t)CAN_VALUE.gyr_x1;
	  				 CAN_VALUE.gyr_Y = (CAN_VALUE.gyr_y2 << 8) + (uint8_t)CAN_VALUE.gyr_y1;
	  				 CAN_VALUE.gyr_Z = (CAN_VALUE.gyr_z2 << 8) + (uint8_t)CAN_VALUE.gyr_z1;

	  		  		 /*CAN_VALUE.gyr_X = (CAN_VALUE.gyr_x2 << 8) | CAN_VALUE.gyr_x1;
	  		  		 CAN_VALUE.gyr_Y = (CAN_VALUE.gyr_y2 << 8) | CAN_VALUE.gyr_y1;
	  		  		 CAN_VALUE.gyr_Z = (CAN_VALUE.gyr_z2 << 8) | CAN_VALUE.gyr_z1;*/
	  		  	  }
	  		  else if(RXData[1]==0x35) // ang
	  		  	  {
	  			     CAN_VALUE.ang_x1=RXData[2];
	  				 CAN_VALUE.ang_x2=RXData[3];
	  				 CAN_VALUE.ang_y1=RXData[4];
	  				 CAN_VALUE.ang_y2=RXData[5];
	  				 CAN_VALUE.ang_z1=RXData[6];
	  				 CAN_VALUE.ang_z2=RXData[7];

	  			  	CAN_VALUE.ang_X = (CAN_VALUE.ang_x2 << 8) + (uint8_t)CAN_VALUE.ang_x1;
	  		  		CAN_VALUE.ang_Y = (CAN_VALUE.ang_y2 << 8) + (uint8_t)CAN_VALUE.ang_y1;
	  		  		CAN_VALUE.ang_Z = (CAN_VALUE.ang_z2 << 8) + (uint8_t)CAN_VALUE.ang_z1;
	  		  		/*CAN_VALUE.ang_X = (CAN_VALUE.ang_x2 << 8) | CAN_VALUE.ang_x1;
	  			     CAN_VALUE.ang_Y = (CAN_VALUE.ang_y2 << 8) | CAN_VALUE.ang_y1;
	  		  		 tmp = (CAN_VALUE.ang_z2 << 8);
	  		  		 tmp2 = (uint8_t)CAN_VALUE.ang_z1;
	  		  		 CAN_VALUE.ang_Z = tmp + tmp2;*/
	  		  	  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

/* USER CODE BEGIN 4 */

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
