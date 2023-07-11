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
bool DAC_Out_EXTI_Flag = false;
uint32_t Speed = 0;
int rot_old_state = 0;
int rot_new_state = 0;
int rot_cnt = 0;
int Rotary_dir = 1;
int busy_flag = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void BlinkLED3(void);
void respirationLED(void);
void RotaryEncoderAction(int);
void generate100Sawtooth(uint32_t divider, int32_t Rotary_dir, uint32_t duty, int rot_cnt);
void SpeedLD_Display(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// calculate the waveform for the DAC
// the waveform is a sawtooth signal repeated from 100 times if the duty cycle is 100%
// the if the duty cycle is 50%, the last 50% of the waveform is 0-padding
// the waveform is stored in the array Waveform and can be used by the DMA
void generateWaveform(uint32_t frequency, uint32_t dutyCycle)
{
  // Calculate the number of ramp-up cycles and 0-padding cycles
  uint32_t numRampUpCycles = (uint32_t)((dutyCycle / 100.0) * totalCycles);
  uint32_t numPaddingCycles = totalCycles - numRampUpCycles;
  int * zeroPaddingPtn = &Waveform[numRampUpCycles*sample_per_period];
  // Generate the signal
  for (uint32_t i = 0; i < numRampUpCycles; i++)
  {
    // Generate a ramp-up cycle
    for (uint32_t j = 0; j < sample_per_period; j++)
    {
      // Increment the DAC output value to generate the sawtooth waveform
      //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, j * (4095 / rampUpDuration));
      Waveform[i*sample_per_period+j] = j * (4095 / sample_per_period);
    }

    // Delay to maintain the duty cycle
    //HAL_Delay(rampUpDuration);
  }

  // Generate the 0-padding cycles
  if (numPaddingCycles > 0){
    for (uint32_t i = 0; i < numPaddingCycles*sample_per_period; i++)
    {
      // Keep the DAC output value at 0 for the duration of each padding cycle
      //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
      *(zeroPaddingPtn+i) = 0;

      // Delay to maintain the duty cycle
      //HAL_Delay(rampUpDuration);
    }
  }
}
/*
void UpdateInterruptSignal(void)
{
	if(PWM_duty_EXTI_Flag)
	{
		if(duty_mode == 3)
		{
			duty_mode = 0;
			PWM_duty_EXTI_Flag = false;
		}
		else{
			 duty_mode++;
			 PWM_duty_EXTI_Flag = false;
		}
	}
	if(DAC_Out_EXTI_Flag)
	{
		if(busy_flag == 0)
		{
			busy_flag = 1;
			//generate100Sawtooth(1,Rotary_dir,duty_mode, 1);
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2047+Rotary_dir*2047);
			DAC_Out_EXTI_Flag=false;
			rot_cnt=0;

		}

	}
}
*/

uint8_t rot_get_state() {
	return (uint8_t)((HAL_GPIO_ReadPin(GPIOB, Rotary_Left_Pin) << 1) 
                | (HAL_GPIO_ReadPin(GPIOB, Rotary_Right_Pin)));
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
  MX_GPIO_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  //set the LD3 and LD4 to be off
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
  HAL_SYSTICK_Config(SystemCoreClock/1000);

  //SpeedLD_Display();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //generate100Sawtooth(1,1,duty_mode);
	  SpeedLD_Display();

	  DAC_Out_EXTI_Flag = false;
	  PWM_duty_EXTI_Flag = false;
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2047);
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

  /*Configure GPIO pin : Rotary_Right_Pin */
  GPIO_InitStruct.Pin = Rotary_Right_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Rotary_Right_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Rotary_Left_Pin Toggle_Up_Pin Toggle_Down_Pin */
  GPIO_InitStruct.Pin = Rotary_Left_Pin|Toggle_Up_Pin|Toggle_Down_Pin;
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
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
	if(GPIO_Pin == Toggle_Up_Pin)
	{
		//PWM_duty_EXTI_Flag = true;
		if(duty_mode == 5)
		{
			duty_mode = 5;
		}
		else{
			 duty_mode++;
		}
	}
	else if(GPIO_Pin == Toggle_Down_Pin)
	{
		//PWM_duty_EXTI_Flag = true;
		if(duty_mode == 1)
		{
			duty_mode = 1;
		}
		else{
			 duty_mode--;
		}
	}

	//if(GPIO_Pin == USER_B1_Pin)
	//{
		//PWM_duty_EXTI_Flag = true;
	//	DAC_Out_EXTI_Flag = true;
	//}
	else if ((GPIO_Pin == Rotary_Right_Pin) && (!(DAC_Out_EXTI_Flag||PWM_duty_EXTI_Flag)))
	{
		if(HAL_GPIO_ReadPin(GPIOC, Rotary_Left_Pin)==1)
		{
			Rotary_dir = 1;
			DAC_Out_EXTI_Flag = true;
			generate100Sawtooth(1,Rotary_dir,duty_mode, 1);
			//DAC_Out_EXTI_Flag = false;
		}
	}
	else if ((GPIO_Pin == Rotary_Left_Pin) && (!(DAC_Out_EXTI_Flag||PWM_duty_EXTI_Flag)))
	{
		if(HAL_GPIO_ReadPin(GPIOC, Rotary_Right_Pin)==1)
		{
			Rotary_dir = -1;
			DAC_Out_EXTI_Flag = true;
			generate100Sawtooth(1,Rotary_dir,duty_mode, 1);
			//DAC_Out_EXTI_Flag = false;
		}
	}
/*
	if (GPIO_Pin == Rotary_Right_Pin || GPIO_Pin == Rotary_Left_Pin) {

		rot_new_state = rot_get_state();

		//DBG("%d:%d", rot_old_state, rot_new_state);

		// Check transition
    //below are the forward direction
		if (rot_old_state == 3 && rot_new_state == 2) {        // 3 -> 2 transition
			//rot_cnt++;
	      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 511);
	      DAC_Out_EXTI_Flag = false;
		}
    else if (rot_old_state == 2 && rot_new_state == 0) { // 2 -> 0 transition
			//rot_cnt++;
	      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1023);
    	DAC_Out_EXTI_Flag = false;
		}
    else if (rot_old_state == 0 && rot_new_state == 1) { // 0 -> 1 transition
			//rot_cnt++;
	    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1535);
    	DAC_Out_EXTI_Flag = false;
		}
    else if (rot_old_state == 1 && rot_new_state == 3) { // 1 -> 3 transition
			//rot_cnt++;
      Rotary_dir = 1;
      DAC_Out_EXTI_Flag = true;
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2047);

		}
    // below are the reverse direction
    else if (rot_old_state == 3 && rot_new_state == 1) { // 3 -> 1 transition
			//rot_cnt--;
    	DAC_Out_EXTI_Flag = false;
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2047+512);
		}
    else if (rot_old_state == 1 && rot_new_state == 0) { // 1 -> 0 transition
			//rot_cnt--;


    	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2047+1024);
    	DAC_Out_EXTI_Flag = false;
		}
    else if (rot_old_state == 0 && rot_new_state == 2) { // 0 -> 2 transition
			//rot_cnt--;
    	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2047+1024+512);
    	DAC_Out_EXTI_Flag = false;
		}
    else if (rot_old_state == 2 && rot_new_state == 3) { // 2 -> 3 transition
			//rot_cnt--;
			Rotary_dir = -1;
    		DAC_Out_EXTI_Flag = true;
    		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
    		//UpdateInterruptSignal();
		}

		rot_old_state = rot_new_state;
	}
	*/
}

//void RotaryEncoderAction(int Rotary_Dir)
//{
// if(Rotary_Dir == -1)
// {
//  if(Speed != 0)
//  {
//   Speed = Speed - 8;
//  }
//  else
//  {
//   Speed = 0;
//  }
//  //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
// }
// else if(Rotary_Dir == 1)
// {
//  if(Speed != 128)
//  {
//   Speed = Speed + 8;
//  }
//  else
//  {
//   Speed = 128;
//  }
//  //HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
// }
// else
// {
//   //do nothing
// }
//}

//Toggle LED LD3 to have a breath effect
//The speed will be controlled by the variabl Speed
void BlinkLED3()
{
  int DelayIncrement = 1;
  //LED ON ramping up phase:
  for(int i = 1; i <= 9; i+=DelayIncrement)
  {
	HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_SET);
    HAL_Delay(i);
    HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_RESET);
    HAL_Delay(10-DelayIncrement);
  }
  //LED OFF ramping down phase:
  for(int i = 1; i <= 9; i+=DelayIncrement)
  {
	HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_SET);
    HAL_Delay(10-DelayIncrement);
    HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_RESET);;
    HAL_Delay(i);
  }
}

void SpeedLD_Display()
{
	uint16_t PIN_Mask = 0x1100;
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


void respirationLED(void)
{
  // Define the minimum and maximum brightness levels
  uint32_t minBrightness = 0;
  uint32_t maxBrightness = 500;

  // Define the breathing period in milliseconds
  uint32_t breathingPeriod = 3000;

  // Define the number of steps for the breathing effect
  uint32_t numSteps = 100;

  // Calculate the delay between each step
  uint32_t stepDelay = breathingPeriod / (2 * numSteps);

  // Calculate the brightness increment for each step
  uint32_t brightnessIncrement = (maxBrightness - minBrightness) / numSteps;

  // Perform the breathing effect
  for (uint32_t i = 0; i < numSteps; i++)
  {
    // Increase brightness
    for (uint32_t brightness = minBrightness; brightness <= maxBrightness; brightness += brightnessIncrement)
    {
      // Set LD3 pin high (LED on)
      HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_SET);
      HAL_Delay(brightness);

      // Set LD3 pin low (LED off)
      HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_RESET);
      HAL_Delay(stepDelay - brightness);
    }

    // Decrease brightness
//    for (uint32_t brightness = maxBrightness; brightness >= minBrightness; brightness -= brightnessIncrement)
//    {
//      // Set LD3 pin high (LED on)
//      HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_SET);
//      HAL_Delay(brightness);
//
//      // Set LD3 pin low (LED off)
//      HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_RESET);
//      HAL_Delay(stepDelay - brightness);
//    }
  }
}
//generate 20 sawtooth waveforms, with frequency of 200 hz
//initial value is 1.5V, then ramp up to 3V, then brutally drop down to 0 V, then ramp up back to 1.5V;
void generate100Sawtooth(uint32_t divider, int32_t direction, uint32_t duty_mode, int rot_cnt)
{
  //uint32_t period = HAL_RCC_GetHCLKFreq() / frequency;
  uint32_t period =  64;
  const uint32_t period_min = 64;
  const uint32_t period_max = 65536;

  uint32_t frequency = 64;

  int32_t increment = 0;
  uint32_t DAC_value = 0;
  

  increment = (int32_t)(4096/period);
  //increment = 16;
  DAC_value = (uint32_t)(2047);
  uint32_t Output_EN = 1;
  uint32_t duty_counter = 1;
  uint32_t total_counter = 1;
  busy_flag = 1;
  switch (duty_mode)
  {
    //33% duty cycle
  case 1:
    duty_counter = 1;
    total_counter = 3;
    break;

   //50% duty cycle 
  case 2:
    duty_counter = 1;
    total_counter = 2;
    break;

   //66% duty cycle 
  case 3:
    duty_counter = 2;
    total_counter = 3;
    break;
   
   //100% duty cycle 
  case 4:
    duty_counter = 2;
    total_counter = 3;
    break;

   //100% duty cycle
  case 5:
    duty_counter = 2;
    total_counter = 3;
    break;

   //100% duty cycle

  case 0:
    duty_counter = 1;
    total_counter = 1;
    break;

  //default duty cycle is 100%
  default:
    break;
  }

  
  for (uint32_t cycle = 0; cycle <9; cycle++)
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

	DAC_value = (uint32_t)(2047);
  


	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_value);

    // Generate a ramp-up cycle
    for (uint32_t i = 0; i < (0.5*period-1); i++)
    {
      // Increment the DAC output value to generate the sawtooth waveform
    	if (Output_EN)
    	{
    		DAC_value += increment*direction;
    	}
    	else
    	{
    		DAC_value = (uint32_t)(2047);
    	}

      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_value);
    }
    //When reach the peak, brutally drop down to 0V
	if (Output_EN)
	{
		DAC_value = 0;
	}
	else
	{
		DAC_value = (uint32_t)(2047);
	}
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_value);
    //HAL_Delay(1);
    //then ramp up back to 1.5V
    for (uint32_t i = 0; i < (0.5*period-1); i++)
    {
      // Increment the DAC output value to generate the sawtooth waveform
    	if (Output_EN)
    	{
    		DAC_value += increment*direction;
    	}
    	else
    	{
    		DAC_value = (uint32_t)(2047);
    	}
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_value);
    }

  }
  busy_flag = 0;
  
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
