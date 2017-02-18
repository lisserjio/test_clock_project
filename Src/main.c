/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
GPIO_TypeDef *hour_ports[12] = {
  GPIOB,
  GPIOA,
  GPIOF,
  GPIOA,
  GPIOB,
  GPIOB,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOF,
  GPIOB
};

uint16_t hour_pins[12] = {
  GPIO_PIN_3,
  GPIO_PIN_14,
  GPIO_PIN_6,
  GPIO_PIN_12,
  GPIO_PIN_14,
  GPIO_PIN_1,
  GPIO_PIN_7,
  GPIO_PIN_5,
  GPIO_PIN_3,
  GPIO_PIN_1,
  GPIO_PIN_1,
  GPIO_PIN_5
};

GPIO_TypeDef *minute_ports[12] = {
  GPIOA,
  GPIOF,
  GPIOA,
  GPIOA,
  GPIOB,
  GPIOB,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOA,
  GPIOF,
  GPIOB
};

uint16_t minute_pins[12] = {
  GPIO_PIN_15,
  GPIO_PIN_7,
  GPIO_PIN_13,
  GPIO_PIN_11,
  GPIO_PIN_13,
  GPIO_PIN_0,
  GPIO_PIN_6,
  GPIO_PIN_4,
  GPIO_PIN_2,
  GPIO_PIN_0,
  GPIO_PIN_0,
  GPIO_PIN_4
};

typedef struct
{
  uint8_t Hours;
  uint8_t Minutes;
}CircleTypeDef; 

CircleTypeDef Circle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define BUTTON_DEBOUNCE 150
uint32_t ButtonDebounce;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if ((HAL_GetTick() - ButtonDebounce) > BUTTON_DEBOUNCE){
	RTC_TimeTypeDef Time;
	RTC_DateTypeDef Date;
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	switch (GPIO_Pin){
	case GPIO_PIN_6:
	  Time.Hours = (Time.Hours == 11) ? 0 : Time.Hours + 1;
	  HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	  HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	  break;
	case GPIO_PIN_7:
	  Time.Hours = (Time.Hours == 0) ? 11 : Time.Hours - 1;
	  HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	  HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	  break;
	case GPIO_PIN_8:
	  Time.Minutes = (Time.Minutes == 59) ? 0 : Time.Minutes + 1;
	  HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	  HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	  break;
	case GPIO_PIN_9:
	  Time.Minutes = (Time.Minutes == 0) ? 59 : Time.Minutes - 1;
	  HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	  HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	  break;
	}
  }
  ButtonDebounce = HAL_GetTick();
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define PWM_LIMIT 5
uint8_t pwm_counter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim == &htim16){
	RTC_TimeTypeDef Time;
	RTC_DateTypeDef Date;
	
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	
	pwm_counter = (pwm_counter >= (PWM_LIMIT-1)) ? 0 : pwm_counter + 1;
	
	if (Circle.Minutes != Time.Minutes/5){
	  HAL_GPIO_WritePin(minute_ports[(Circle.Minutes)], minute_pins[(Circle.Minutes)], GPIO_PIN_SET);
	  Circle.Minutes = Time.Minutes/5;
	}
	
	if (pwm_counter >= Time.Minutes%5)
	  HAL_GPIO_WritePin(minute_ports[(Circle.Minutes)], minute_pins[(Circle.Minutes)], GPIO_PIN_RESET);
	else 
	  HAL_GPIO_WritePin(minute_ports[(Circle.Minutes)], minute_pins[(Circle.Minutes)], GPIO_PIN_SET);
	
	if (pwm_counter >= (5 - Time.Minutes%5))
	  HAL_GPIO_WritePin(minute_ports[(Circle.Minutes + 1)], minute_pins[(Circle.Minutes + 1)], GPIO_PIN_RESET);
	else 
	  HAL_GPIO_WritePin(minute_ports[(Circle.Minutes + 1)], minute_pins[(Circle.Minutes + 1)], GPIO_PIN_SET);
  }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	RTC_TimeTypeDef Time;
	RTC_DateTypeDef Date;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM16_Init();

  /* USER CODE BEGIN 2 */
  HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
  Circle.Hours = Time.Hours;
  Circle.Minutes = Time.Minutes/5;
  HAL_GPIO_WritePin(hour_ports[(Circle.Hours)], hour_pins[(Circle.Hours)],  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(minute_ports[(Circle.Minutes)], minute_pins[(Circle.Minutes)],  GPIO_PIN_RESET);

  HAL_TIM_Base_Start_IT(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
    if (Circle.Hours != Time.Hours){
      HAL_GPIO_WritePin(hour_ports[(Circle.Hours)], hour_pins[(Circle.Hours)],	GPIO_PIN_SET);
      Circle.Hours = Time.Hours;
      HAL_GPIO_WritePin(hour_ports[(Circle.Hours)], hour_pins[(Circle.Hours)], 	GPIO_PIN_RESET);
    }
    if (Circle.Minutes != Time.Minutes/5){
      HAL_GPIO_WritePin(minute_ports[(Circle.Minutes)], minute_pins[(Circle.Minutes)],GPIO_PIN_SET);
      Circle.Minutes = Time.Minutes/5;
      HAL_GPIO_WritePin(minute_ports[(Circle.Minutes)], minute_pins[(Circle.Minutes)], GPIO_PIN_RESET);
    }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
