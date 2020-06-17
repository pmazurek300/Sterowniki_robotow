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
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct Vector3D {
	float x;
	float y;
	float z;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define L1 15.00
#define L2 23.00
#define podstawa 15.00
#define M_PI 3.14159265358979323846
#define EEPROM_ADDRESS_W 0xA0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Hcsr04_Distance_tmp;
int H_sum;

char buff[20];
uint8_t Received[15];
uint8_t flag;

uint8_t flaga_dolnego_stepp;
uint8_t out_dolnego_stepp = 0;

uint8_t flaga_gornego_stepp;
uint8_t out_gornego_stepp = 0;

uint8_t flaga_servo_joint;
uint8_t flaga_servo_effector;

int pwm_duty;
int pwm_duty_servo_joint;
int pwm_duty_servo_effector;
uint16_t step_dolnego;
uint16_t step_gornego;

float a;
float b;
float s;
char * copy;
char *newline;
char tmp[16];
struct Vector3D p[3];
uint16_t zapis[3];
uint16_t odczyt[3];

float alfa = 0.00;
float beta = 0.00;
float sigma = 0.00;

uint16_t step_dolnego_fabrik;
uint16_t step_gornego_fabrik;
uint8_t flaga_fabrik_dolny;
uint8_t flaga_fabrik_gorny;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Fabrik(struct Vector3D target) {
	alfa = (step_dolnego % 12900) * 360.00 / 12900.00;
	beta = step_gornego * 360.00 / 200.00;
	sigma = (pwm_duty_servo_joint - 250.00) * 180.00 / 1000.00 + 90.00;

	p[1].x = cosf(alfa * M_PI / 180.00) * cosf(beta * M_PI / 180.00) * L1;
	p[1].y = sinf(alfa * M_PI / 180.0) * cosf(beta * M_PI / 180.0) * L1;
	p[1].z = podstawa + cosf((90.0 - beta) * M_PI / 180.0) * L1;
	p[2].x = cosf(alfa * M_PI / 180.0)
			* (cosf(beta * M_PI / 180.0) * L1
					+ sinf((beta + sigma - 90.0) * M_PI / 180.0) * L2);
	p[2].y = sinf(alfa * M_PI / 180.0)
			* (cosf(beta * M_PI / 180.0) * L1
					+ sinf((beta + sigma - 90.0) * M_PI / 180.0) * L2);
	p[2].z = podstawa + cosf((90.0 - beta) * M_PI / 180.0) * L1
			- cosf((beta + sigma - 90.0) * M_PI / 180.0) * L2;
	float R[3];
	float Lambda[3];

	float D[2];
	D[0] = sqrt(
			pow((p[1].x - p[0].x), 2) + pow((p[1].y - p[0].y), 2)
					+ pow((p[1].z - p[0].z), 2));
	D[1] = sqrt(
			pow((p[2].x - p[1].x), 2) + pow((p[2].y - p[1].y), 2)
					+ pow((p[2].z - p[1].z), 2));

	float Dist = sqrt(
			pow((p[0].x - target.x), 2) + pow((p[0].y - target.y), 2)
					+ pow((p[0].z - target.z), 2));

	struct Vector3D B;

	float DifA;

	float Tol = 0.000001;

	if (Dist > D[0] + D[1]) {
		for (int i = 0; i < 3; i++) {
			R[i] = sqrt(
					pow((target.x - p[i].x), 2) + pow((target.y - p[i].y), 2)
							+ pow((target.z - p[i].z), 2));
			Lambda[i] = D[i] / R[i];

			if (i > 0) {
				p[i].x = (1 - Lambda[i - 1]) * p[i - 1].x
						+ Lambda[i - 1] * target.x;
				p[i].y = (1 - Lambda[i - 1]) * p[i - 1].y
						+ Lambda[i - 1] * target.y;
				p[i].z = (1 - Lambda[i - 1]) * p[i - 1].z
						+ Lambda[i - 1] * target.z;
			}
		}
	} else {
		B.x = p[0].x;
		B.y = p[0].y;
		B.z = p[0].z;

		DifA = sqrt(
				pow((p[2].x - target.x), 2) + pow((p[2].y - target.y), 2)
						+ pow((p[2].z - target.z), 2));
		while (DifA > Tol) {
			float tmp = DifA;

			p[2].x = target.x;
			p[2].y = target.y;
			p[2].z = target.z;

			for (int i = 1; i > -1; i--) {
				R[i] = sqrt(
						pow((p[i + 1].x - p[i].x), 2)
								+ pow((p[i + 1].y - p[i].y), 2)
								+ pow((p[i + 1].z - p[i].z), 2));
				Lambda[i] = D[i] / R[i];
				p[i].x = (1 - Lambda[i]) * p[i + 1].x + Lambda[i] * p[i].x;
				p[i].y = (1 - Lambda[i]) * p[i + 1].y + Lambda[i] * p[i].y;
				p[i].z = (1 - Lambda[i]) * p[i + 1].z + Lambda[i] * p[i].z;
			}

			p[0].x = B.x;
			p[0].y = B.y;
			p[0].z = B.z;

			for (int i = 0; i < 2; i++) {
				R[i] = sqrt(
						pow((p[i + 1].x - p[i].x), 2)
								+ pow((p[i + 1].y - p[i].y), 2)
								+ pow((p[i + 1].z - p[i].z), 2));
				Lambda[i] = D[i] / R[i];
				p[i + 1].x = (1 - Lambda[i]) * p[i].x + Lambda[i] * p[i + 1].x;
				p[i + 1].y = (1 - Lambda[i]) * p[i].y + Lambda[i] * p[i + 1].y;
				p[i + 1].z = (1 - Lambda[i]) * p[i].z + Lambda[i] * p[i + 1].z;
			}

			DifA = sqrt(
					pow((p[2].x - target.x), 2) + pow((p[2].y - target.y), 2)
							+ pow((p[2].z - target.z), 2));
			if (tmp == DifA) {
				break;
			}
		}
	}
	if (target.x > 0 && target.y > 0) {
		alfa = atanf(p[2].y / p[2].x) * 180 / M_PI;
	} else if (target.x < 0 && target.y > 0) {
		alfa = -atanf(p[2].y / p[2].x) * 180 / M_PI + 90.0;
	} else if (target.x < 0 && target.y < 0) {
		alfa = atanf(p[2].y / p[2].x) * 180 / M_PI + 180.0;
	} else if (target.x > 0 && target.y < 0) {
		alfa = -atanf(p[2].y / p[2].x) * 180 / M_PI + 270.0;
	} else if (target.x == 0 && target.y == 0) {
		alfa = 0.0;
	} else if (target.x > 0 && target.y == 0) {
		alfa = 0.0;
	} else if (target.x == 0 && target.y > 0) {
		alfa = 90.0;
	} else if (target.x < 0 && target.y == 0) {
		alfa = 180.0;
	} else if (target.x == 0 && target.y < 0) {
		alfa = 270.0;
	}
	beta = acosf(sqrt(pow(p[1].x, 2) + pow(p[1].y, 2)) / L1) * 180 / M_PI;
		sigma = asinf(sqrt(pow(p[1].x, 2) + pow(p[1].y, 2)) / L1) * 180 / M_PI + asinf((sqrt(pow(p[2].x, 2) + pow(p[2].y, 2)) - sqrt(pow(p[1].x, 2) + pow(p[1].y, 2))) / L2) * 180 / M_PI;

	step_dolnego_fabrik = alfa * 12900.00 / 360.00;
	step_gornego_fabrik = beta * 200.00 / 360.00;
	pwm_duty_servo_joint = sigma * 1000.00 / 180.00 + 250.00;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty_servo_joint);

	if (step_dolnego > step_dolnego_fabrik) {
		HAL_GPIO_WritePin(Dir_STEPPER_LOWER_GPIO_Port, Dir_STEPPER_LOWER_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(En_STEPPER_LOWER_GPIO_Port, En_STEPPER_LOWER_Pin,
				GPIO_PIN_SET);
		flaga_fabrik_dolny = 1;
	} else if (step_dolnego < step_dolnego_fabrik) {
		HAL_GPIO_WritePin(Dir_STEPPER_LOWER_GPIO_Port, Dir_STEPPER_LOWER_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(En_STEPPER_LOWER_GPIO_Port, En_STEPPER_LOWER_Pin,
				GPIO_PIN_SET);
		flaga_fabrik_dolny = 2;
	}

	if (step_gornego > step_gornego_fabrik) {
		HAL_GPIO_WritePin(Dir_STEPPER_UPPER_GPIO_Port, Dir_STEPPER_UPPER_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(En_STEPPER_UPPER_GPIO_Port, En_STEPPER_UPPER_Pin,
				GPIO_PIN_SET);
		flaga_fabrik_gorny = 1;
	} else if (step_gornego < step_gornego_fabrik) {
		HAL_GPIO_WritePin(Dir_STEPPER_UPPER_GPIO_Port, Dir_STEPPER_UPPER_Pin,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(En_STEPPER_UPPER_GPIO_Port, En_STEPPER_UPPER_Pin,
				GPIO_PIN_SET);
		flaga_fabrik_gorny = 2;
	}

}

int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 50);
	return len;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		uint16_t time;
		H_sum = 0;

		time = __HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_2)
				- __HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_1);
		if (time < 23615) {
			Hcsr04_Distance_tmp = (float) time / 2.0 * 0.0343;
			sprintf(buff, "X %.2f ", Hcsr04_Distance_tmp);
			for (int i = 0; i < strlen(buff); i++) {
				H_sum += buff[i];
			}
			H_sum = H_sum % 37;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		if (flag == 1 && flaga_dolnego_stepp == 1 && step_dolnego < 25800) {
			if (out_dolnego_stepp) {
				out_dolnego_stepp = 0;
				HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port,
				Step_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
			} else {
				step_dolnego++;
				out_dolnego_stepp = 1;
				HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port,
				Step_STEPPER_LOWER_Pin, GPIO_PIN_SET);
			}
		} else if (flag == 1 && flaga_dolnego_stepp == 2 && step_dolnego > 0) {
			if (out_dolnego_stepp) {
				out_dolnego_stepp = 0;
				HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port,
				Step_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
			} else {
				step_dolnego--;
				out_dolnego_stepp = 1;
				HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port,
				Step_STEPPER_LOWER_Pin, GPIO_PIN_SET);
			}
		} else if (flaga_fabrik_dolny == 1) {
			if (out_dolnego_stepp) {
				out_dolnego_stepp = 0;
				HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port,
				Step_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
			} else {
				step_dolnego--;
				out_dolnego_stepp = 1;
				HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port,
				Step_STEPPER_LOWER_Pin, GPIO_PIN_SET);
			}
			if (step_dolnego == step_dolnego_fabrik) {
				flaga_fabrik_dolny = 0;
				HAL_GPIO_WritePin(En_STEPPER_LOWER_GPIO_Port,
				En_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
			}
		} else if (flaga_fabrik_dolny == 2) {
			if (out_dolnego_stepp) {
				out_dolnego_stepp = 0;
				HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port,
				Step_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
			} else {
				step_dolnego++;
				out_dolnego_stepp = 1;
				HAL_GPIO_WritePin(Step_STEPPER_LOWER_GPIO_Port,
				Step_STEPPER_LOWER_Pin, GPIO_PIN_SET);
			}
			if (step_dolnego == step_dolnego_fabrik) {
				flaga_fabrik_dolny = 0;
				HAL_GPIO_WritePin(En_STEPPER_LOWER_GPIO_Port,
				En_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
			}
		}
	}
	if (htim == &htim11) {
		if (flag == 1 && flaga_gornego_stepp == 1 && step_gornego > 0) {
			if (out_gornego_stepp) {
				out_gornego_stepp = 0;
				HAL_GPIO_WritePin(Step_STEPPER_UPPER_GPIO_Port,
				Step_STEPPER_UPPER_Pin, GPIO_PIN_RESET);
			} else {
				out_gornego_stepp = 1;
				step_gornego--;
				HAL_GPIO_WritePin(Step_STEPPER_UPPER_GPIO_Port,
				Step_STEPPER_UPPER_Pin, GPIO_PIN_SET);

			}
		} else if (flag == 1 && flaga_gornego_stepp == 2
				&& step_gornego < 116) {
			if (out_gornego_stepp) {
				out_gornego_stepp = 0;
				HAL_GPIO_WritePin(Step_STEPPER_UPPER_GPIO_Port,
				Step_STEPPER_UPPER_Pin, GPIO_PIN_RESET);
			} else {
				out_gornego_stepp = 1;
				step_gornego++;
				HAL_GPIO_WritePin(Step_STEPPER_UPPER_GPIO_Port,
				Step_STEPPER_UPPER_Pin, GPIO_PIN_SET);
			}
		}else if (flaga_fabrik_gorny == 1) {
			if (out_gornego_stepp) {
				out_gornego_stepp = 0;
				HAL_GPIO_WritePin(Step_STEPPER_UPPER_GPIO_Port,
				Step_STEPPER_UPPER_Pin, GPIO_PIN_RESET);
			} else {
				step_gornego--;
				out_gornego_stepp = 1;
				HAL_GPIO_WritePin(Step_STEPPER_UPPER_GPIO_Port,
				Step_STEPPER_UPPER_Pin, GPIO_PIN_SET);
			}
			if (step_gornego == step_gornego_fabrik) {
				flaga_fabrik_gorny = 0;
			}
		} else if (flaga_fabrik_gorny == 2) {
			if (out_gornego_stepp) {
				out_gornego_stepp = 0;
				HAL_GPIO_WritePin(Step_STEPPER_UPPER_GPIO_Port,
				Step_STEPPER_UPPER_Pin, GPIO_PIN_RESET);
			} else {
				step_gornego++;
				out_gornego_stepp = 1;
				HAL_GPIO_WritePin(Step_STEPPER_UPPER_GPIO_Port,
				Step_STEPPER_UPPER_Pin, GPIO_PIN_SET);
			}
			if (step_gornego == step_gornego_fabrik) {
				flaga_fabrik_gorny = 0;
			}
		}
	}
	if (htim == &htim10) {
		printf("X %.2f #%d %d %d %f %f %f %s \r\n", Hcsr04_Distance_tmp, H_sum,
				step_dolnego, step_gornego, a, b, s,tmp);
//		printf("X %s 2 \r\n",tmp);

	}
	if (htim == &htim9) {
		if (flag == 1 && flaga_servo_joint == 1
				&& pwm_duty_servo_joint < 1250) {
			pwm_duty_servo_joint++;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty_servo_joint);
		} else if (flag == 1 && flaga_servo_joint == 2
				&& pwm_duty_servo_joint > 250) {
			pwm_duty_servo_joint--;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty_servo_joint);
		} else if (flag == 1 && flaga_servo_effector == 1
				&& pwm_duty_servo_effector < 1250) {
			pwm_duty_servo_effector += 2;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
					pwm_duty_servo_effector);
		} else if (flag == 1 && flaga_servo_effector == 2
				&& pwm_duty_servo_effector > 250) {
			pwm_duty_servo_effector -= 2;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
					pwm_duty_servo_effector);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	flag = 1;
	HAL_UART_Receive_DMA(&huart2, Received, 15);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	flag = 0;
	H_sum = 0;
	pwm_duty = 0;
	flaga_dolnego_stepp = 0;
	flaga_gornego_stepp = 0;
	pwm_duty_servo_joint = 250;
	pwm_duty_servo_effector = 250;
	flaga_servo_joint = 0;
	flaga_servo_effector = 0;
	step_dolnego = 12900;
	step_gornego = 0;
	struct Vector3D target;
	step_dolnego_fabrik = 0;
	step_gornego_fabrik = 0;
	flaga_fabrik_dolny = 0;
	char *token;


	p[0].x = 0;
	p[0].y = 0;
	p[0].z = podstawa;
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
	MX_TIM1_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM11_Init();
	MX_TIM9_Init();
	MX_TIM10_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim11);

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	HAL_UART_Receive_DMA(&huart2, Received, 15);
	HAL_I2C_Mem_Read(&hi2c1, 0xa0, 0x10, 1, (uint16_t*) &odczyt, sizeof(odczyt),
			50);
	pwm_duty_servo_joint = odczyt[1];
	step_dolnego = odczyt[0];
	step_gornego = odczyt[2];
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty_servo_joint);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (flag == 1) {
			token =strtok(Received, " ");
			switch (atoi(token)) {
			case 0: // do przodu
				if (Hcsr04_Distance_tmp >= 50) {
					HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin,
							GPIO_PIN_RESET);

					if (pwm_duty < 3000) {
						pwm_duty += 1;
					}
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_duty);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_duty);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_duty);
				} else {
					HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin,
							GPIO_PIN_RESET);

					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
				}
				break;

			case 1: // w lewo
				HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin, GPIO_PIN_SET);

				if (pwm_duty < 3000) {
					pwm_duty += 1;
				}
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_duty);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_duty);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_duty);
				break;

			case 2: //  w prawo
				HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin, GPIO_PIN_RESET);

				if (pwm_duty < 3000) {
					pwm_duty += 1;
				}
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_duty);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_duty);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_duty);
				break;

			case 3: // do tylu
				HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin, GPIO_PIN_SET);

				if (pwm_duty < 3000) {
					pwm_duty += 1;
				}
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_duty);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_duty);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_duty);
				break;

			case 4:
				flaga_dolnego_stepp = 1;
				HAL_GPIO_WritePin(Dir_STEPPER_LOWER_GPIO_Port,
				Dir_STEPPER_LOWER_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(En_STEPPER_LOWER_GPIO_Port,
				En_STEPPER_LOWER_Pin, GPIO_PIN_SET);
				break;

			case 5:
				flaga_dolnego_stepp = 2;
				HAL_GPIO_WritePin(Dir_STEPPER_LOWER_GPIO_Port,
				Dir_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(En_STEPPER_LOWER_GPIO_Port,
				En_STEPPER_LOWER_Pin, GPIO_PIN_SET);
				break;

			case 6:

				break;

			case 7:

				break;

			case 8:

				break;

			case 9:

				break;

			case 10:

				break;

			case 11:

				break;

			case 12:

				break;

			case 13:


				break;

			default:
				HAL_GPIO_WritePin(Dc_IN2_GPIO_Port, Dc_IN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Dc_IN1_GPIO_Port, Dc_IN1_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
				flaga_servo_joint = 0;
				flaga_servo_effector = 0;
				pwm_duty = 0;
				flag = 0;
				flaga_dolnego_stepp = 0;
				flaga_gornego_stepp = 0;
				break;
			}
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
			//  HAL_Delay(10);
		}
		if(flaga_dolnego_stepp ==0 && flaga_fabrik_dolny ==0){
			HAL_GPIO_WritePin(En_STEPPER_LOWER_GPIO_Port,
			En_STEPPER_LOWER_Pin, GPIO_PIN_RESET);
		}

			zapis[0] = step_dolnego;
			zapis[1] = pwm_duty_servo_joint;
			zapis[2] = step_gornego;
			HAL_I2C_Mem_Write(&hi2c1, 0xa0, 0x10, 1, (uint16_t*) &zapis,
					sizeof(zapis), 50);
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
