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

uint8_t mode = 1; // not use
uint8_t num = 0;

uint32_t now = 0;		//may use
uint32_t prenow = 0;	//may use
uint32_t sec = 0;		//may use

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


	/*
	char ch1;
	//Waiting Receive...
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)== RESET) {}
	HAL_UART_Receive(&huart1, (uint8_t*) &ch1, 1, 1000);

	switch (ch1){
	  case '1':
	  	  break;
	  default:

	}
	*/
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


  void ADC_PWM(){

  		//ADC
  		  while(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){} // skip
  		  adc_val = HAL_ADC_GetValue(&hadc1); // get value of adc
  		  //displayHEX(adc_val);

	    //PWM
  		  dutyCycleScreen = ((adc_val/4095.0) * 0.8) + 0.2; // for 20% - 100%  // 0.8 is interval
  		  //No. 2
  		  htim2.Instance -> CCR1 = (100-1) * dutyCycleScreen;


  		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

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

					    //RESET SET
		GPIOE -> BSRR = 0x00000080 << num;
		GPIOE -> BSRR = 0x00400000 << num;
		HAL_Delay(300);
		num++;
		if(num == 9){ // on follow off 9 times
			num = 0;
		}


		/*

		//Light Off		//RESET SET
		GPIOC -> BSRR = (0x00000700);
		HAL_Delay(500);

		//Light On
		GPIOC -> BSRR = (0x04000000 >> n);
		HAL_Delay(500);

		 */
  }

  void ledRGB(){

	  //static
//	  dutyCycleR =0.5;
	  dutyCycleG =0.5;
	  dutyCycleB =0.5;

	  htim3.Instance -> CCR1 = (100-1) * dutyCycleR;
	  htim3.Instance -> CCR2 = (100-1) * dutyCycleG;
	  htim3.Instance -> CCR3 = (100-1) * dutyCycleB;

	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

//	  //Waiting Receive...
//	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)== RESET) {
//		  pwmR = ((GPIOB-> IDR & GPIO_PIN_3) >> 3);
//		  pwmG = ((GPIOA-> IDR & GPIO_PIN_5) >> 5)+1;
//		  pwmB = ((GPIOB-> IDR & GPIO_PIN_4) >> 4)+2;
//	  }
//	  HAL_UART_Receive(&huart1, (uint8_t*) &ch, 1, 1000);

//	  switch(ch){
//	  case 'r':
//		  dutyCycleR += 0.2;
//		  if(dutyCycleR > 1.0){
//			  dutyCycleR = 0.0;
//		  }
//		  break;
//	  case 'g':
//		  dutyCycleG += 0.2;
//		  if(dutyCycleG > 1.0){
//			  dutyCycleG = 0.0;
//		  }
//		  break;
//	  case 'b':
//		  dutyCycleB += 0.2;
//		  if(dutyCycleB > 1.0){
//			  dutyCycleB = 0.0;
//		  }
//		  break;
//
//	  }

	  dutyCycleR += 0.2;
	  if(dutyCycleR > 1.0){
		  dutyCycleR = 0.0;
	  }

//	  It doesn't need to have stop functions
//	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	//Initialise screen
	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);


	//Print something here ...
	//...

	uint32_t maxWidth = 320;
	uint32_t maxHeight = 240;


	sprintf(Temp_Buffer_text, "This is a test");
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 40, BLACK, 2, WHITE);


	//Start While Touch
	while(1){

		// update or read value
		ADC_PWM();
		readButton();
		ledPack();
		ledRGB();

		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);

		if(TP_Touchpad_Pressed())
		{

			uint16_t x_pos = 0;
			uint16_t y_pos = 0;


			//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_SET);

			uint16_t position_array[2];

			if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK)
			{
				x_pos = position_array[0];
				y_pos = position_array[1];
				//ILI9341_Draw_Filled_Circle(x_pos, y_pos, 2, BLACK);


				if(x_pos > 0 && x_pos < 240 && y_pos > 0  && y_pos < 320/2-50){
					mode = 1;
					break;
				}

				//Check x_pos y_pos for mapping

//				ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//				char counter_buff[30];
//				sprintf(counter_buff, "POS X: %.3d", x_pos);
//				ILI9341_Draw_Text(counter_buff, 10, 80, BLACK, 2, WHITE);
//				sprintf(counter_buff, "POS Y: %.3d", y_pos);
//				ILI9341_Draw_Text(counter_buff, 10, 120, BLACK, 2, WHITE);


				//All Update when pressed
				//...
			}
		}
		//All Update when not pressed
		//...


		//Counting
//  	  	now = HAL_GetTick();
//  	  	if (now - prenow >= 1000){
//				sec++;
//				prenow = now;
//  	  	}
//
//  	  	if(sec == 5){
//  	  		sec = 0;
//  	  		mode = 1;
//  	  		break;
//  	  	}

	}
	//End While Touch




//----------------------------------------------------------COUNTING MULTIPLE SEGMENTS
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Counting multiple segments at once", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		for(uint16_t i = 0; i <= 10; i++)
//	  		{
//	  		sprintf(Temp_Buffer_text, "Counting: %d", i);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 30, BLUE, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 50, RED, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 70, GREEN, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 90, BLACK, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 110, BLUE, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 130, RED, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 150, GREEN, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 170, WHITE, 2, BLACK);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 190, BLUE, 2, BLACK);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 210, RED, 2, BLACK);
//	  		}
//
//	  		HAL_Delay(1000);

//----------------------------------------------------------COUNTING SINGLE SEGMENT
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Counting single segment", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint16_t i = 0; i <= 100; i++)
//	  		{
//	  		sprintf(Temp_Buffer_text, "Counting: %d", i);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 3, WHITE);
//	  		}
//
//	  		HAL_Delay(1000);

//----------------------------------------------------------ALIGNMENT TEST
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Rectangle alignment check", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		ILI9341_Draw_Hollow_Rectangle_Coord(50, 50, 100, 100, BLACK);
//	  		ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 50, 50, BLACK);
//	  		ILI9341_Draw_Hollow_Rectangle_Coord(10, 10, 19, 19, BLACK);
//	  		HAL_Delay(1000);

//----------------------------------------------------------LINES EXAMPLE

		//Line and Horizontal and Vertical

//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Horizontal and Vertical lines", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 30000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Horizontal_Line(xr, yr, radiusr, colourr);
//	  			ILI9341_Draw_Vertical_Line(xr, yr, radiusr, colourr);
//	  		}
//
//	  		HAL_Delay(1000);

//----------------------------------------------------------HOLLOW CIRCLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Circles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		for(uint32_t i = 0; i < 3000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Hollow_Circle(xr, yr, radiusr*2, colourr);
//	  		}
//	  		HAL_Delay(1000);

//----------------------------------------------------------FILLED CIRCLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Filled Circles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 1000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Filled_Circle(xr, yr, radiusr/2, colourr);
//	  		}
//	  		HAL_Delay(1000);

//----------------------------------------------------------HOLLOW RECTANGLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Rectangles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 20000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Hollow_Rectangle_Coord(xr, yr, xr+radiusr, yr+radiusr, colourr);
//	  		}
//	  		HAL_Delay(1000);

//----------------------------------------------------------FILLED RECTANGLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Filled Rectangles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 20000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Rectangle(xr, yr, radiusr, radiusr, colourr);
//	  		}
//	  		HAL_Delay(1000);

//----------------------------------------------------------INDIVIDUAL PIXEL EXAMPLE

		//Transition

//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Slow draw by selecting", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("and addressing pixels", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		x = 0;
//	  		y = 0;
//	  		while (y < 240)
//	  		{
//	  		while ((x < 320) && (y < 240))
//	  		{
//
//	  			if(x % 2)
//	  			{
//	  				ILI9341_Draw_Pixel(x, y, BLACK);
//	  			}
//
//	  			x++;
//	  		}
//
//	  			y++;
//	  			x = 0;
//	  		}
//
//	  		x = 0;
//	  		y = 0;
//
//
//	  		while (y < 240)
//	  		{
//	  		while ((x < 320) && (y < 240))
//	  		{
//
//	  			if(y % 2)
//	  			{
//	  				ILI9341_Draw_Pixel(x, y, BLACK);
//	  			}
//
//	  			x++;
//	  		}
//
//	  			y++;
//	  			x = 0;
//	  		}
//	  		HAL_Delay(2000);

//----------------------------------------------------------INDIVIDUAL PIXEL EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Random position and colour", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("500000 pixels", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		for(uint32_t i = 0; i < 500000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			uint16_t color = HAL_RNG_GetRandomNumber(&hrng);
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			ILI9341_Draw_Pixel(xr, yr, color);
//	  		}
//	  		HAL_Delay(2000);

//----------------------------------------------------------565 COLOUR EXAMPLE, Grayscale
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Colour gradient", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Grayscale", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//
//
//	  		for(uint16_t i = 0; i <= (320); i++)
//	  		{
//	  			uint16_t Red = 0;
//	  			uint16_t Green = 0;
//	  			uint16_t Blue = 0;
//
//	  			Red = i/(10);
//	  			Red <<= 11;
//	  			Green = i/(5);
//	  			Green <<= 5;
//	  			Blue = i/(10);
//
//
//
//	  			uint16_t RGB_color = Red + Green + Blue;
//	  			ILI9341_Draw_Rectangle(i, x, 1, 240, RGB_color);
//
//	  		}
//	  		HAL_Delay(2000);

//----------------------------------------------------------IMAGE EXAMPLE, Snow Tiger
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("RGB Picture", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("TIGER", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Draw_Image((const char*)snow_tiger, SCREEN_VERTICAL_2);
//	  		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//	  		HAL_Delay(5000);


//----------------------------------------------------------TOUCHSCREEN EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Touchscreen", 10, 10, BLACK, 2, WHITE);
//	  		ILI9341_Draw_Text("Touch to draw", 10, 30, BLACK, 2, WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//
//
//	  		while(1)
//	  		{
//	  			//HAL_Delay(50);
//
//	  			if(TP_Touchpad_Pressed())
//	          {
//
//	  					uint16_t x_pos = 0;
//	  					uint16_t y_pos = 0;
//
//
//	  					HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_SET);
//
//	  					uint16_t position_array[2];
//
//	  					if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK)
//	  					{
//	  					x_pos = position_array[0];
//	  					y_pos = position_array[1];
//	  					ILI9341_Draw_Filled_Circle(x_pos, y_pos, 2, BLACK);
//
//	  					ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  					char counter_buff[30];
//	  					sprintf(counter_buff, "POS X: %.3d", x_pos);
//	  					ILI9341_Draw_Text(counter_buff, 10, 80, BLACK, 2, WHITE);
//	  					sprintf(counter_buff, "POS Y: %.3d", y_pos);
//	  					ILI9341_Draw_Text(counter_buff, 10, 120, BLACK, 2, WHITE);
//	  					ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//	  					}
//
//	  					ILI9341_Draw_Pixel(x_pos, y_pos, BLACK);
//
//	          }
//	  			else
//	  			{
//	  				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);
//	  			}
//
//	  		}



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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
