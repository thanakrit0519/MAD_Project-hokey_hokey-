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
#include "math.h"
#include "snow_tiger.h"
#include "img_view.h"
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
/* Private variables ---------------------------------------------------------*/
uint16_t adc_val;
uint32_t count;

HAL_StatusTypeDef status;

void doMotor();
uint8_t playState = 0;
uint8_t isDrawButton = 0;

uint8_t stepsPerRevolution = 0;
uint16_t delayStep = 500;
uint32_t prevActionMotor = 0;
uint8_t isHigh = 0;
uint16_t Hp = 1;
uint16_t lastHp = 0;
uint32_t countPlayTime = 1;
uint32_t lastCountPlayTime = 0;
uint32_t lastCountTime = 0;

uint32_t debounceSw = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

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
	MX_USART3_UART_Init();
	MX_SPI5_Init();
	MX_TIM1_Init();
	MX_RNG_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	ILI9341_Init(); //initial driver setup to drive ili9341
	ILI9341_Fill_Screen(WHITE);
	HAL_TIM_Base_Start_IT(&htim2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (playState == 0) {
			state0();
		} else if (playState == 1) {
			state1();
		} else if (playState == 2) {
			state2();
		} else if (playState == 3) {
			state3();
		} else if (playState == 4) {
			state4();
			play();
		} else if (playState == 5) {
			state5();
			play();
		} else if (playState == 6) {
			state6();
		}

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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 200;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void doMotor() {
	if (count - prevActionMotor > delayStep && stepsPerRevolution != 0) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, isHigh);
		isHigh = abs(isHigh - 1);
		prevActionMotor = count;
		stepsPerRevolution--;
	} else if (stepsPerRevolution == 0) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
	}
}

void play() {
	HAL_ADC_Start(&hadc1);
	while (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)
		;
	adc_val = HAL_ADC_GetValue(&hadc1);
	delayStep = (int) (((float) adc_val / 4095) * 2000) + 500;
	uint8_t readSw1 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);
	uint8_t readSw2 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);
	uint8_t readLimitSw1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	uint8_t readLimitSw2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	if (readSw1 == 0 && readLimitSw1 != 1) {
		stepsPerRevolution = 8;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
	}
	if (readSw2 == 0 && readLimitSw2 != 1) {
		stepsPerRevolution = 8;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
	}
	doMotor();
}

void state0() {
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
	ILI9341_Draw_Text("Hokey Hokey", 65, 35, BLACK, 3, WHITE);
	if (isDrawButton == 0) {
		ILI9341_Draw_Rectangle(60, 70, 200, 120, RED);
		ILI9341_Draw_Text("PLAY", 105, 110, WHITE, 5, RED);
		isDrawButton = 1;
	}
	if (TP_Touchpad_Pressed()) {
		uint16_t x_pos = 0;
		uint16_t y_pos = 0;
		uint16_t position_array[2];

		if (TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK) {
			y_pos = 240 - position_array[0];
			x_pos = position_array[1];
			if (y_pos >= 70 && y_pos <= 190 && x_pos >= 60 && x_pos <= 260) {
				playState = 1;
				isDrawButton = 0;
				HAL_Delay(400);
				ILI9341_Fill_Screen(WHITE);
			}
		}

	}
}
void state1() {
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
	ILI9341_Draw_Text("Select Mode", 65, 35, BLACK, 3, WHITE);
	if (isDrawButton == 0) {
		ILI9341_Draw_Rectangle(55, 80, 210, 50, RED);
		ILI9341_Draw_Text("No time to limit", 67, 97, WHITE, 2, RED);
		ILI9341_Draw_Rectangle(55, 145, 210, 50, RED);
		ILI9341_Draw_Text("limit time", 102, 162, WHITE, 2, RED);
		isDrawButton = 1;
	}
	if (TP_Touchpad_Pressed()) {
		uint16_t x_pos = 0;
		uint16_t y_pos = 0;
		uint16_t position_array[2];

		if (TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK) {
			y_pos = 240 - position_array[0];
			x_pos = position_array[1];
			if (y_pos >= 80 && y_pos <= 130 && x_pos >= 55 && x_pos <= 265) {
				playState = 2;
				isDrawButton = 0;
				HAL_Delay(400);
				ILI9341_Fill_Screen(WHITE);
			}
			if (y_pos >= 145 && y_pos <= 195 && x_pos >= 55 && x_pos <= 265) {
				playState = 3;
				isDrawButton = 0;
				HAL_Delay(400);
				ILI9341_Fill_Screen(WHITE);
			}
		}

	}
}

void state2() {
	if (isDrawButton == 0) {
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
		ILI9341_Draw_Text("Select max hp", 30, 35, BLACK, 3, WHITE);
		ILI9341_Draw_Rectangle(110, 180, 100, 40, RED);
		ILI9341_Draw_Text("start", 117, 187, WHITE, 3, RED);

		ILI9341_Draw_Filled_Circle(70, 115, 30, RED);
		ILI9341_Draw_Text("down", 48, 108, WHITE, 2, RED);

		ILI9341_Draw_Filled_Circle(250, 115, 30, RED);
		ILI9341_Draw_Text("up", 243, 108, WHITE, 2, RED);
		isDrawButton = 1;
	}
	if (lastHp != Hp) {
		char str[20];
		ILI9341_Draw_Rectangle(120, 75, 80, 80, BLACK);
		sprintf(str, "%d", Hp);
		uint8_t x = 0;
		if (strlen(str) == 1) {
			ILI9341_Draw_Text("0", 138, 100, WHITE, 4, BLACK);
			x = 27;
		}
		ILI9341_Draw_Text(str, 138 + x, 100, WHITE, 4, BLACK);
		lastHp = Hp;
	}
	if (TP_Touchpad_Pressed()) {
		uint16_t x_pos = 0;
		uint16_t y_pos = 0;
		uint16_t position_array[2];

		if (TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK) {
			y_pos = 240 - position_array[0];
			x_pos = position_array[1];
			if (y_pos >= 180 && y_pos <= 220 && x_pos >= 110 && x_pos <= 210) {
				playState = 4;
				isDrawButton = 0;
				lastHp = -1;
				char str[20];
				sprintf(str, "1 %d", Hp);
				while (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC) == RESET) {
				}
				HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);
				ILI9341_Fill_Screen(WHITE);
			}
			if (sqrt(pow(x_pos - 70, 2) + pow(y_pos - 115, 2)) <= 30) {
				if (Hp > 1) {
					Hp--;
					HAL_Delay(100);
				}
			}
			if (sqrt(pow(x_pos - 250, 2) + pow(y_pos - 115, 2)) <= 30) {

				if (Hp < 99) {
					Hp++;
					HAL_Delay(100);
				}
			}
		}

	}
}
void state3() {
	if (isDrawButton == 0) {
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
		ILI9341_Draw_Text("Select time to end", 40, 35, BLACK, 2, WHITE);
		ILI9341_Draw_Rectangle(110, 180, 100, 40, RED);
		ILI9341_Draw_Text("start", 117, 187, WHITE, 3, RED);

		ILI9341_Draw_Filled_Circle(55, 115, 30, RED);
		ILI9341_Draw_Text("down", 33, 108, WHITE, 2, RED);

		ILI9341_Draw_Filled_Circle(250, 115, 30, RED);
		ILI9341_Draw_Text("up", 243, 108, WHITE, 2, RED);
		isDrawButton = 1;
	}
	if (lastCountPlayTime != countPlayTime) {
		char str[20];
		ILI9341_Draw_Rectangle(93, 75, 122, 80, BLACK);
		sprintf(str, "%d", countPlayTime);
		uint8_t x = 0;
		if (strlen(str) == 1) {
			ILI9341_Draw_Text("0", 105, 100, WHITE, 4, BLACK);
			x = 27;
		}
		ILI9341_Draw_Text(str, 105 + x, 100, WHITE, 4, BLACK);
		ILI9341_Draw_Text("min", 160, 105, WHITE, 3, BLACK);
		lastCountPlayTime = countPlayTime;
	}
	if (TP_Touchpad_Pressed()) {
		uint16_t x_pos = 0;
		uint16_t y_pos = 0;
		uint16_t position_array[2];

		if (TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK) {
			y_pos = 240 - position_array[0];
			x_pos = position_array[1];
			if (y_pos >= 180 && y_pos <= 220 && x_pos >= 110 && x_pos <= 210) {
				playState = 5;
				isDrawButton = 0;
				Hp = 0;
				lastHp = -1;
				countPlayTime = countPlayTime * 60;
				char str[20];
				sprintf(str, "2 %d", countPlayTime);
				while (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC) == RESET) {
				}
				HAL_UART_Transmit(&huart2, (uint8_t*) str, strlen(str), 100);
				ILI9341_Fill_Screen(WHITE);
			}
			if (sqrt(pow(x_pos - 55, 2) + pow(y_pos - 115, 2)) <= 30) {
				if (countPlayTime > 1) {
					countPlayTime--;
					HAL_Delay(100);
				}
			}
			if (sqrt(pow(x_pos - 250, 2) + pow(y_pos - 115, 2)) <= 30) {

				if (countPlayTime < 99) {
					countPlayTime++;
					HAL_Delay(100);
				}
			}
		}

	}
}
void state4() {
	if (isDrawButton == 0) {
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
		ILI9341_Draw_Text("your hp", 100, 35, BLACK, 3, WHITE);
		isDrawButton = 1;
	}
	if (lastHp != Hp) {
		char str[20];
		ILI9341_Draw_Rectangle(120, 75, 80, 80, BLACK);
		sprintf(str, "%d", Hp);
		uint8_t x = 0;
		if (strlen(str) == 1) {
			ILI9341_Draw_Text("0", 138, 100, WHITE, 4, BLACK);
			x = 27;
		}
		ILI9341_Draw_Text(str, 138 + x, 100, WHITE, 4, BLACK);
		lastHp = Hp;
	}
	if (Hp <= 0) {
		ILI9341_Fill_Screen(WHITE);
		playState = 6;
	}
}
void state5() {
	if (isDrawButton == 0) {
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
		ILI9341_Draw_Text("time", 125, 35, BLACK, 3, WHITE);
		ILI9341_Draw_Text("your score", 70, 135, BLACK, 3, WHITE);
		isDrawButton = 1;
	}
	if (lastCountPlayTime != countPlayTime) {
		uint8_t min = countPlayTime / 60;
		uint8_t sec = countPlayTime - (((int) (countPlayTime / 60)) * 60);
		char str[20];
		ILI9341_Draw_Rectangle(75, 60, 170, 40, BLACK);
		sprintf(str, "%d", min);
		uint8_t x = 0;
		if (strlen(str) == 1) {
			ILI9341_Draw_Text("0", 108, 62, WHITE, 4, BLACK);
			x = 27;
		}
		ILI9341_Draw_Text(str, 108 + x, 62, WHITE, 4, BLACK);
		ILI9341_Draw_Text(":", 162, 62, WHITE, 4, BLACK);
		sprintf(str, "%d", sec);
		x = 0;
		if (strlen(str) == 1) {
			ILI9341_Draw_Text("0", 173, 62, WHITE, 4, BLACK);
			x = 27;
		}
		ILI9341_Draw_Text(str, 173 + x, 62, WHITE, 4, BLACK);
		lastCountPlayTime = countPlayTime;
	}
	if (count - lastCountTime > 100000 && countPlayTime > 0) {
		countPlayTime--;
		lastCountTime = count;
	}
	if (lastHp != Hp) {
		char str[20];
		ILI9341_Draw_Rectangle(120, 160, 80, 40, BLACK);
		sprintf(str, "%d", Hp);
		uint8_t x = 0;
		if (strlen(str) == 1) {
			ILI9341_Draw_Text("0", 138, 162, WHITE, 4, BLACK);
			x = 27;
		}
		ILI9341_Draw_Text(str, 138 + x, 162, WHITE, 4, BLACK);
		lastHp = Hp;
	}
	if (countPlayTime <= 0) {
		ILI9341_Fill_Screen(WHITE);
		playState = 6;
	}

}
void state6() {
	ILI9341_Draw_Text("win", 100, 100, BLACK, 4, WHITE);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_7) {
		if (playState == 4) {
			if (count - debounceSw > 20000) {
				Hp--;
				debounceSw = count;
			}
		}
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
