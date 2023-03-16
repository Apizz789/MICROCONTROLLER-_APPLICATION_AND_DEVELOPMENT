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
#include "adc.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "snow_tiger.h"
#include "pop.h"
#include "string.h"

#include "stdbool.h"
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
uint32_t color = 0x0000;
uint32_t value1 = 0x0000;
uint32_t value2 = 0x0000;
uint32_t value3 = 0x0000;

I2C_HandleTypeDef hi2c1; // not use
I2C_HandleTypeDef hi2c3; // not use

bool pressButton1 = false;
bool pressButton2 = false;
bool pressButton3 = false;
bool pressButton4 = false;

bool isPressButton1 = false;
bool isPressButton2 = false;
bool isPressButton3 = false;
bool isPressButton4 = false;

bool changeState = false;

float prevDuty = 0;

uint8_t mode = 0; // not use
uint8_t num = 0;

uint32_t now = 0;		//may use
uint32_t prenow = 0;	//may use
uint32_t sec = 0;		//may use
uint32_t counting = 3001;

float dutyCycleScreen = 0.0;

float dutyCycleR = 0.0;
float dutyCycleG = 0.0;
float dutyCycleB = 0.0;

char Temp_Buffer_text[50];

//ADC

volatile uint32_t adc_val = 0;

void displayHEX(uint32_t value){

	char hexString[100];

	sprintf(hexString,"ADC1_CH13 0x%08X Vin = %.2f V RawValue = %d\r\n", value, (float)value*3.3/4096.0, value);
	HAL_UART_Transmit(&huart1, (uint8_t*) hexString, strlen(hexString), 100);
	HAL_Delay(50);

}

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
  MX_TIM1_Init();
  MX_GPIO_Init();
  MX_RNG_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();//initial driver setup to drive ili9341


  HAL_ADC_Start(&hadc1);

  GPIOE -> BSRR = 0x00007F80 << num;	// off all

  void ADC_PWM(){

  		//ADC
  		  while(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){} // skip
  		  adc_val = HAL_ADC_GetValue(&hadc1); // get value of adc
  		  //displayHEX(adc_val);

	    //PWM
  		  dutyCycleScreen = ((adc_val/4095.0) * 0.55) + 0.25; // for 25% - 80%  // 0.8 is interval
  		  //No. 2
  		  htim2.Instance -> CCR1 = (100-1) * dutyCycleScreen;

  		 float displayNum = 0;


  		if(dutyCycleScreen >= 0.25 && dutyCycleScreen < 0.30){
  			displayNum = 0.25;
  		}else if(dutyCycleScreen >= 0.30 && dutyCycleScreen < 0.35){
  			displayNum = 0.30;
  		}else if(dutyCycleScreen >= 0.35 && dutyCycleScreen < 0.40){
  			displayNum = 0.35;
  		}else if(dutyCycleScreen >= 0.40 && dutyCycleScreen < 0.45){
  			displayNum = 0.40;
 		}else if(dutyCycleScreen >= 0.45 && dutyCycleScreen < 0.50){
 			displayNum = 0.45;
 		}else if(dutyCycleScreen >= 0.50 && dutyCycleScreen < 0.55){
 			displayNum = 0.50;
 		}else if(dutyCycleScreen >= 0.55 && dutyCycleScreen < 0.60){
 			displayNum = 0.55;
 		}else if(dutyCycleScreen >= 0.60 && dutyCycleScreen < 0.65){
 			displayNum = 0.60;
 		}else if(dutyCycleScreen >= 0.65 && dutyCycleScreen < 0.68){
 			displayNum = 0.65;
 		}else if(dutyCycleScreen >= 0.68 && dutyCycleScreen < 0.75){
 			displayNum = 0.70;
 		}else if(dutyCycleScreen >= 0.75 && dutyCycleScreen < 0.79){
 			displayNum = 0.75;
 		}else if(dutyCycleScreen >= 0.79){
 			displayNum = 0.80;
 		}

  		if(prevDuty != displayNum){
  			changeState = true;
  			prevDuty = displayNum;
  		}

  		sprintf(Temp_Buffer_text, "%d", (uint8_t) (displayNum*100));
  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
  		  if(displayNum >= 0.20 && displayNum <= 0.50){
  			  if(changeState == true){
  	  			ILI9341_Draw_Filled_Rectangle_Coord(0, 0, 320, 240, GREEN);
  	  			ILI9341_Draw_Text(Temp_Buffer_text, 320/2-20, 240/2-20, YELLOW, 2, GREEN);
  	  			changeState = false;
  			  }

  		  }else if(displayNum >= 0.55 && displayNum < 0.70){
  			  if(changeState == true){
  	  			ILI9341_Draw_Filled_Rectangle_Coord(0, 0, 320, 240, YELLOW);
  	  			ILI9341_Draw_Text(Temp_Buffer_text,  320/2-20, 240/2-20, BLUE, 2, YELLOW);
  				changeState = false;
  			  }

  		  }else if(displayNum >= 0.75 && displayNum <= 0.90){
  			ILI9341_Draw_Filled_Rectangle_Coord(0, 0, 320, 240, RED);
  			if(sec % 2 == 0){
  				sprintf(Temp_Buffer_text, "%d", (uint8_t) (displayNum*100));
  				ILI9341_Draw_Text(Temp_Buffer_text,  320/2-20, 240/2-20, CYAN, 2, RED);
  			}else{
  				sprintf(Temp_Buffer_text, "  ", (uint8_t) (displayNum*100));
  	  			ILI9341_Draw_Text(Temp_Buffer_text,  320/2-20, 240/2-20, CYAN, 2, RED);
  			}
  		  }

  		  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);



  		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  		  htim3.Instance -> CCR1 = (1000-1) * dutyCycleScreen;

  		  /*
  		     float dutyCycleB = 0.0;

  		   	 htim2.Instance -> CCR1 = (100-1) * dutyCycleB;
	  	  	 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  			 HAL_Delay(1000);
  		  	 // HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

  		   */
  	}

  void readButton(){
	pressButton1 = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10); // pressButton1 is "true" when press, is "false" when not press
	pressButton2 = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12); // pressButton1 is "true" when press, is "false" when not press
	pressButton3 = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14); // pressButton1 is "true" when press, is "false" when not press
	pressButton4 = !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_2); // pressButton1 is "true" when press, is "false" when not press

//	char hexString[100];
//	sprintf(hexString,"%d %d %d %d\r\n",pressButton1, pressButton2, pressButton3, pressButton4);
//	HAL_UART_Transmit(&huart1, (uint8_t*) hexString, strlen(hexString), 100);

  }

  void ledPack(){

		// on when 0 (RESET)
		// off when 1 (SET)
		uint8_t on = 0;
		uint8_t off = 1;

		//Basic WritePin
	//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, off);
	//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, off);
	//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, off);
	//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, off);
	//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, off);
	//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, off);
	//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, off);
	//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, off);

//		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7)

					    //RESET SET
//		GPIOE -> BSRR = 0x00000080 << num;	// off
//		GPIOE -> BSRR = 0x00400000 << num;	// on
//		HAL_Delay(500);
//		num++;
//		if(num == 9){ // on follow off 9 times
//			num = 0;
//		}

		//Step 1
//		GPIOE -> BSRR = 0x78000000 << num;	// on
//		HAL_Delay(500);
//		GPIOE -> BSRR = 0x00007F80 << num;	// off
//		HAL_Delay(500);
//
//		//Step 2
//		GPIOE -> BSRR = 0x07800000 << num;	// on
//		HAL_Delay(500);
//		GPIOE -> BSRR = 0x00007F80 << num;	// off
//		HAL_Delay(500);
//
//		//Step 3
//		GPIOE -> BSRR = 0x60000000 << num;	// on
//		HAL_Delay(500);
//		GPIOE -> BSRR = 0x00007F80 << num;	// off
//		HAL_Delay(500);
//
//		//Step 4
//		GPIOE -> BSRR = 0x01800000 << num;	// on
//		HAL_Delay(500);
//		GPIOE -> BSRR = 0x00007F80 << num;	// off
//		HAL_Delay(500);








		/*

		//Light Off		//RESET SET
		GPIOC -> BSRR = (0x00000700);
		HAL_Delay(500);

		//Light On
		GPIOC -> BSRR = (0x04000000 >> n);
		HAL_Delay(500);

		 */
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//		 update or read value
		ADC_PWM();



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
