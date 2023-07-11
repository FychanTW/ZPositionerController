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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f100xb.h"
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dac.h"
#include "core_cm3.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
bool Rotary2_L_Debounce = false; // when true, it's in waiting period
bool Rotary2_R_Debounce = false; // when true, it's in waiting period
const int Debounce_period = 10; //unit: ms
int totalCycles = 100;
int APB1_clk = 8000000;// unit: Hz
int Sawtooth_Freq = 1000;
int sample_per_period;
int length_of_waveform ;
int Waveform[100]; //length should be defined.
int duty_mode= 1;
int mode = 0;
bool PWM_duty_EXTI_Flag = false;
bool Toggle_unlock = false;
bool DAC_Out_EXTI_Flag = false;
uint32_t Speed = 0;
int rot_old_state = 0;
int rot_new_state = 0;
int rot_cnt = 0;
int Rotary_dir = 1;
int busy_flag = 0;
int32_t DAC_value = 0;
int32_t fine_DAC_value = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void generateSawtooth(uint32_t divider, int32_t Rotary_dir, uint32_t duty, int rot_cnt);
void SpeedLD_Display(void);
void UpdateInterruptSignal(void);
void Coarse_Move(uint32_t duty_counter, uint32_t total_counter, int32_t direction);
void Fine_Move(uint32_t increment, int32_t direction);
void BlinkLED(uint16_t Toggle_Pin);
void delay_ms(uint16_t au16_ms);
void delay_us(uint16_t au16_us);
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
  MX_DAC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
  HAL_SYSTICK_Config(SystemCoreClock/200);
  HAL_GPIO_WritePin(LD_Speed1_GPIO_Port, 0x1F00, GPIO_PIN_SET);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

  //SpeedLD_Display();
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  DAC_Out_EXTI_Flag = false;
	  PWM_duty_EXTI_Flag = false;


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim2.Init.Prescaler = 24;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD_Speed1_Pin|LD_Speed2_Pin|LD_Speed3_Pin|LD_Speed4_Pin
                          |LD_Speed5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Rotary_Right_Pin Rotary_Left_Pin Toggle_Up_Pin Toggle_Down_Pin */
  GPIO_InitStruct.Pin = Rotary_Right_Pin|Rotary_Left_Pin|Toggle_Up_Pin|Toggle_Down_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD_Speed1_Pin LD_Speed2_Pin LD_Speed3_Pin LD_Speed4_Pin
                           LD_Speed5_Pin */
  GPIO_InitStruct.Pin = LD_Speed1_Pin|LD_Speed2_Pin|LD_Speed3_Pin|LD_Speed4_Pin
                          |LD_Speed5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	uint16_t Toggle_Pin = 0x0100;
	uint16_t Not_Toggle_Pin = 0x1F00;
	if(GPIO_Pin == Toggle_Up_Pin && Toggle_unlock)
	{
		Toggle_unlock = false;

		PWM_duty_EXTI_Flag = false;
		if(duty_mode == 5)
		{
			duty_mode = 5;
		}
		else{
			 duty_mode++;
		}
		HAL_GPIO_WritePin(LD_Speed1_GPIO_Port, Not_Toggle_Pin, GPIO_PIN_RESET);

		uint16_t Pin_count = 0x0001;
		for(int mode = 1; mode<duty_mode; mode++)
		{
			Pin_count = Pin_count+(0x0001<<mode);
		}
		// Offset to GPIOA_Pin8
		Toggle_Pin = Pin_count <<8;


		HAL_GPIO_WritePin(LD_Speed1_GPIO_Port, Toggle_Pin, GPIO_PIN_SET);

	}

	else if(GPIO_Pin == Toggle_Down_Pin && Toggle_unlock)
	{
		Toggle_unlock = false;
		PWM_duty_EXTI_Flag = false;
		if(duty_mode == 1)
		{
			duty_mode = 1;

		}
		else{
			 duty_mode--;
		}

		uint16_t Pin_count = 0x0001;
		for(int mode = 1; mode<duty_mode; mode++)
		{
			Pin_count = Pin_count+(0x0001<<mode);
		}
		// Offset to GPIOA_Pin8
		Toggle_Pin = Pin_count <<8;

		HAL_GPIO_WritePin(LD_Speed1_GPIO_Port, Not_Toggle_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, Toggle_Pin, GPIO_PIN_SET);



	}

//	else if ((GPIO_Pin == Rotary_Right_Pin) && (!(DAC_Out_EXTI_Flag||PWM_duty_EXTI_Flag)))
	else if ((GPIO_Pin == Rotary_Right_Pin) && (!DAC_Out_EXTI_Flag))
	{
		if(HAL_GPIO_ReadPin(GPIOC, Rotary_Left_Pin)==1)
		{
			Rotary_dir = 1;
			DAC_Out_EXTI_Flag = true;
			generateSawtooth(1,Rotary_dir,duty_mode, 1);
		}
	}
	//else if ((GPIO_Pin == Rotary_Left_Pin) && (!(DAC_Out_EXTI_Flag||PWM_duty_EXTI_Flag)))
	else if ((GPIO_Pin == Rotary_Left_Pin) && !(DAC_Out_EXTI_Flag))
	{
		if(HAL_GPIO_ReadPin(GPIOC, Rotary_Right_Pin)==1)
		{
			Rotary_dir = -1;
			DAC_Out_EXTI_Flag = true;
			generateSawtooth(1,Rotary_dir,duty_mode, 1);
		}
	}
}


void SpeedLD_Display()
{
	//uint16_t PIN_Mask = 0x1100;
	uint16_t Toggle_Pin = 0x0000;
	uint16_t Not_Toggle_Pin = 0x1111;

	if(duty_mode == 1) Toggle_Pin = 0x0100;
	if(duty_mode == 2) Toggle_Pin = 0x0300;
	if(duty_mode == 3) Toggle_Pin = 0x0700;
	if(duty_mode == 4) Toggle_Pin = 0x0f00;
	if(duty_mode == 5) Toggle_Pin = 0x1f00;
//
//	if(duty_mode == 1) Toggle_Pin = 0x0100;
//	if(duty_mode == 2) Toggle_Pin = 0x0200;
//	if(duty_mode == 3) Toggle_Pin = 0x0400;
//	if(duty_mode == 4) Toggle_Pin = 0x0800;
//	if(duty_mode == 5) Toggle_Pin = 0x1000;

	Not_Toggle_Pin = (!Toggle_Pin)-0x00ff;

	HAL_GPIO_WritePin(LD_Speed1_GPIO_Port, Toggle_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD_Speed1_GPIO_Port, Not_Toggle_Pin, GPIO_PIN_RESET);

}

void  UpdateInterruptSignal()
{
	//PWM_duty_EXTI_Flag = true; // unlock every 100 ms;
	DAC_Out_EXTI_Flag = false;
	Toggle_unlock = true;
}

void generateSawtooth(uint32_t divider, int32_t direction, uint32_t duty_mode, int rot_cnt)
{

  DAC_value = (uint32_t)(2047);
  //uint32_t Output_EN = 1;
  uint32_t duty_counter = 1;
  uint32_t total_counter = 1;
  busy_flag = 1;
  switch (duty_mode)
  {
    //Fine mode 1
  case 1:
    Fine_Move(128, direction);
    break;

   //Fine mode 2 (Faster)
  case 2:
    Fine_Move(512, direction);
    break;

   //66% duty cycle
  case 3:
    duty_counter = 1;
    total_counter = 2;
    Coarse_Move(duty_counter, total_counter, direction);
    break;

   //100% duty cycle
  case 4:
    duty_counter = 2;
    total_counter = 3;
    Coarse_Move(duty_counter, total_counter, direction);
    break;

   //100% duty cycle
  case 5:
    duty_counter = 1;
    total_counter = 1;
    Coarse_Move(duty_counter, total_counter, direction);
    break;

  default:
    break;
  }

}

void Coarse_Move(uint32_t duty_counter, uint32_t total_counter, int32_t direction)
{
	  int32_t DAC_value = 0;
	  int32_t DAC_value_ini = 0;
	  uint32_t period =64;
	  uint32_t increment = (uint32_t)(4096/period);
	  uint32_t Output_EN = 1;



	  if(direction ==1)
	  {
		  DAC_value_ini = 0;
	  }
	  else if (direction == -1)
	  {
		  DAC_value_ini = 4095;
	  }

	  for (uint32_t cycle = 0; cycle <64; cycle++)
	  {
	    //calculate output_EN based on the duty cycle and the current cycle
	    if(cycle%total_counter == 0)
	    {
	      Output_EN = 1;
	    }
	    else if(cycle%total_counter < duty_counter)
	    {
	      Output_EN = 1;
	    }
	    else
	    {
	      Output_EN = 0;
	    }
	    DAC_value = DAC_value_ini;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_value*Output_EN);

	    // Generate a ramp-up cycle
	    for (uint32_t i = 0; i < (period-1); i++)
	    {
	      // Increment the DAC output value to generate the sawtooth waveform
	    	if (Output_EN)
	    	{
	    		DAC_value += increment*direction;
	    		if(DAC_value >=4095) { DAC_value =4095;}
	    		else if(DAC_value <0) {DAC_value =0;}
	    	}
	    	else
	    	{
	    		DAC_value = 0;
	    	}

	      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_value*Output_EN);
	      delay_us(10);
	    }
	  }
	  busy_flag = 0;
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

}
void Fine_Move(uint32_t increment, int32_t direction)
{
	int32_t tmp = fine_DAC_value;
    fine_DAC_value = fine_DAC_value + increment*direction;
    int32_t goal = fine_DAC_value;
    while(tmp!=goal)
    {
    	tmp = tmp + direction;
//    	if(tmp >=4095)
//    	{
//    		tmp =0;
//    		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tmp);
//    		fine_DAC_value=tmp;
//    		delay_us(10);
//
//    		break;
//    	}
//    	else if (tmp<0)
//    	{
//    		tmp = 4095;
//    		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tmp);
//    		fine_DAC_value=tmp;
//    		delay_us(10);
//
//    		break;
//    	}
//    	else
    	{
    		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tmp);
    		fine_DAC_value=tmp;
    		delay_us(1000);

    	}
    }
//	if(DAC_value >=4095) { DAC_value =0;}
//	else if(DAC_value <0) {DAC_value =4095;}
//
//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, fine_DAC_value);

}
void BlinkLED(uint16_t Toggle_Pin)
{
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, Toggle_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, Toggle_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, Toggle_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, Toggle_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, Toggle_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, Toggle_Pin, GPIO_PIN_SET);
}
void delay_us(uint16_t au16_us)
{
	htim2.Instance->CNT = 0;
    while (htim2.Instance->CNT < au16_us);
}
void delay_ms(uint16_t au16_ms)
{
    while(au16_ms > 0)
    {
    htim2.Instance->CNT = 0;
    au16_ms--;
    while (htim2.Instance->CNT < 1000);
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
