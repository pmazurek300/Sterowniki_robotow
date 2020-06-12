/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

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
volatile float Hcsr04_Distance_tmp;
uint8_t Received;
uint8_t flag;
int pwm_duty;
volatile int H_sum;
uint8_t test = 0x0F;
uint8_t result = 0;
uint8_t out=0;
#define EEPROM_ADDRESS_W 0xA0 //zapis
//#define EEPROM_ADDRES_R 0xA1 //odczyt
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim3){
	uint16_t time;
	char buff[25];
	uint8_t len;

	time = __HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_2) -__HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_1);
	if(time < 23615) {
		Hcsr04_Distance_tmp = (float)time / 2.0 * 0.0343;
		len = sprintf(buff,"%.2f ",Hcsr04_Distance_tmp);
		for(int i =0; i<strlen(buff);i++){
			H_sum +=buff[i];
		}
		H_sum = H_sum %37;
		len = sprintf(buff,"%.2f #%d\r\n" ,Hcsr04_Distance_tmp,H_sum);
		HAL_UART_Transmit(&huart2, (uint8_t*)buff, len,15);
	}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
if(htim == &htim2){
	if(flag==1){
	if(out) {
	        out = 0;
	        HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port, Step_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
	    }
	    else {
	        out = 1;
	        HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port, Step_STEPPER_LOWER_Pin, GPIO_PIN_SET);
	    }
	}
}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	flag=1;
	HAL_UART_Receive_DMA(&huart2, &Received, 1);
 }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	flag=0;
	H_sum = 0;
	pwm_duty=0;
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_UART_Receive_DMA(&huart2, &Received, 1);

  HAL_I2C_Mem_Write(&hi2c1, 0xa0, 0x10, 1, (uint8_t*)&test, sizeof(test), HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, 0xa0, 0x10, 1, (uint8_t*)&result, sizeof(result), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(Dir_STEPPER_LOWER_GPIO_Port, Dir_STEPPER_LOWER_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flag == 1){
		  switch (atoi(&Received)){
		  case 0: // do przodu
			  if(Hcsr04_Distance_tmp >= 50){
				  HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin, GPIO_PIN_RESET);

				  if(pwm_duty <3000){
					  pwm_duty +=1;
				  }
				  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm_duty);
				  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_duty);
				  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_duty);
			  }
			  else{
				  HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin, GPIO_PIN_RESET);

				  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
				  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
				  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			  }
			  break;

		  case 1: // w lewo
			  HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin, GPIO_PIN_SET);

			  if(pwm_duty <3000){
				  pwm_duty +=1;
			  }
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_duty);
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_duty);
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm_duty);
			  break;

		  case 2: //  w prawo
			  HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin, GPIO_PIN_RESET);

			  if(pwm_duty <3000){
				  pwm_duty +=1;
			  }
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_duty);
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_duty);
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm_duty);
			  break;

		  case 3: // do tylu
			  HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin, GPIO_PIN_SET);

			  if(pwm_duty <3000){
				  pwm_duty +=1;
			  }
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_duty);
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_duty);
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm_duty);
			  break;
		  case 4:
			  HAL_GPIO_WritePin(En_STEPPER_LOWER_GPIO_Port, En_STEPPER_LOWER_Pin, GPIO_PIN_SET);
			  break;
		  default:
			  HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin, GPIO_PIN_RESET);

			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
			  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);

			  HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port, Step_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(En_STEPPER_LOWER_GPIO_Port, En_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
			  pwm_duty=0;
			  flag = 0;
			  break;
		  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//  HAL_Delay(10);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
