/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
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
	GPIO_PinState init = RESET;
	GPIO_PinState shutdown = RESET;

	char name[] = "\t\t 62010948\r\n\t\t Sirawit Sukwattanavit\r\n";
	char quit[] = "\t\t Quit";
	char unknown[] = "\t\t Unknown Command\r\n";
	char newline[] = "\r\n";

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //No.1 Sending string
//	  char str[] = "Hello, World!!\r\n";
//	  //char a = 'a';
//	  //while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)== RESET){}
//	  HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 1000);
//	  //HAL_UART_Transmit(&huart1, (uint8_t*) &a, 1, 1000);
//	  HAL_Delay(500);


	  //No.2 Sending character A
//	  char ch1 = 'A';
//	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)== RESET){} // Check if Transmit complete
//	  HAL_UART_Transmit(&huart1, (uint8_t*) &ch1, 1, 1000);
//	  HAL_Delay(500);


	  //No.3 Input character
//	  char inp[] = "\r\nInput => ";
//	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)== RESET){}
//	  HAL_UART_Transmit(&huart1, (uint8_t*) inp, strlen(inp), 1000);
//
//	  char ch1;
//	  //Waiting Receive...
//	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)== RESET) {}
//	  HAL_UART_Receive(&huart1, (uint8_t*) &ch1, 1, 1000);
//
//	  //Send what Receive ->
//	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)== RESET){}
//	  HAL_UART_Transmit(&huart1, (uint8_t*) &ch1 , 1, 1000);
//
//	  if (ch1 == 'q'){
//		  char quit[] = "\r\nQUIT";
//		  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)== RESET){}
//		  HAL_UART_Transmit(&huart1, (uint8_t*) quit, strlen(quit), 1000);
//		  break;
//	  }


	  //No.4
	  char display[] = "Display Blinking LED PASS (1, 2)\r\nDisplay Group Members PRESS m\r\nQuit PRESS q";
	  char inp[] = "\tInput => ";

	  //Initiate
	  if(init == GPIO_PIN_RESET){
		  HAL_UART_Transmit(&huart1, (uint8_t*) display, strlen(display), 1000);
		  init = GPIO_PIN_SET;
	  }

	  //Sending Input
	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)== RESET){}
	  HAL_UART_Transmit(&huart1, (uint8_t*) inp, strlen(inp), 1000);

	  char ch1;
	  //Waiting Receive...
	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)== RESET) {}
	  HAL_UART_Receive(&huart1, (uint8_t*) &ch1, 1, 1000);

	  //Sending what receive to terminal
	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)== RESET){}
	  HAL_UART_Transmit(&huart1, (uint8_t*) &ch1, 1, 1000);

	  //Newline
	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)== RESET){}
	  HAL_UART_Transmit(&huart1, (uint8_t*) newline, strlen(newline) , 1000);

	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)== RESET){}
	  switch (ch1){
	  case '1':
		  for(int i = 0; i<3 ;i++){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
			  HAL_Delay(300);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);
			  HAL_Delay(300);
		  }
		  break;
	  case '2':
		  for(int i = 0; i<3 ;i++){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);
			  HAL_Delay(300);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);
			  HAL_Delay(300);
		  }
		  break;
	  case 'm':
		  HAL_UART_Transmit(&huart1, (uint8_t*) name, strlen(name), 1000);
		  break;
	  case 'q':
		  shutdown = 1;
		  HAL_UART_Transmit(&huart1, (uint8_t*) quit, strlen(quit), 1000);
		  break;
	  default:
		  HAL_UART_Transmit(&huart1, (uint8_t*) unknown, strlen(unknown), 1000);
	  }

	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)== RESET){}
	  HAL_UART_Transmit(&huart1, (uint8_t*) newline, strlen(newline) , 1000);

	  //Quit
	  if(shutdown == 1){
		  break;
	  }




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
