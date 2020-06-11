
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "snow_tiger.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// Programdaki variable degerleri
uint8_t sesion=0;
uint8_t minute=2;
uint8_t second=11;
uint16_t breakk=20;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
ILI9341_Init();
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);// Buzzer in çalismasi için pwm timeri çalistirma kodu

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	uint32_t adcReturnValue = 0; // Adc degeri atama

	char Temp_Buffer_text[40];// Ekrana yazi yazdirmak için bir karakter arrayi olusturuyoruz.
	
	// Program Baslatildigindaki karsilama yazisi
	srand(234);
	ILI9341_Fill_Screen(BLUE);
	ILI9341_Draw_Text("WELCOME TO", 10, 10, BLACK, 4, RED);
	ILI9341_Draw_Text("POMODORO", 10, 80, WHITE,4, RED);
	ILI9341_Draw_Text("TECNIC", 10, 150, WHITE, 4, RED);

	HAL_Delay(5000);
  while (1)
  {
		
		
	// Program basladiktan sonra ayarlari degistirme hakkinda bilgiler	
	ILI9341_Fill_Screen(BLUE);
  ILI9341_Draw_Text("PRESS PA0", 10, 10, BLACK, 4, RED);
	ILI9341_Draw_Text("CHANGE", 10, 70, WHITE, 4, RED);
	ILI9341_Draw_Text("SETTINGS", 10, 120, WHITE, 4, RED);
	sprintf(Temp_Buffer_text, "SESION:%d", sesion);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 180, BLACK, 5, WHITE);		
	HAL_Delay(2000);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
			
		
		

		ILI9341_Fill_Screen(WHITE);
		

// PA0 pinine basinca seyans degerlerini degistirebiliyoruz
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))	{
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_0)){// PD0 pininne basinca seyans degeri artiyor
		if(sesion<255){
			sesion++;
		}
	}
if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1)){// PD1 pininne basinca seyans degeri azaliyor
	    if(sesion>0){
			sesion--;
		}
	}
HAL_Delay(1000);
	   // Seyans Degerini degistirmek için ekrandaki bilgi yazisi
		sprintf(Temp_Buffer_text, "SESION:%d", sesion);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 5, WHITE);	
	  ILI9341_Draw_Text("PRESS PD0", 10, 80, BLACK, 4, WHITE);
	  ILI9341_Draw_Text("TO INCREASE", 10, 120, BLACK, 4, WHITE);
	  ILI9341_Draw_Text("PRESS PD1", 10, 160, BLACK, 4, WHITE);
	  ILI9341_Draw_Text("TO DECREASE", 10, 200, BLACK, 4, WHITE);
	}

ILI9341_Fill_Screen(WHITE);
	
  ILI9341_Draw_Text("PRESS PD3", 10, 10, BLACK, 4, RED);
	ILI9341_Draw_Text("TO START", 10, 70, WHITE, 4, BLACK);
	ILI9341_Draw_Text("PROGRAM", 10, 120, WHITE, 4, BLACK);
	sprintf(Temp_Buffer_text, "SESION:%d", sesion);
	ILI9341_Draw_Text(Temp_Buffer_text, 10, 180, BLACK, 5, WHITE);	
	HAL_Delay(2000);

	
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)){	// PD3 pinine basinca programimiz çalisiyor
		
while(sesion>0){

while(minute>0){	
ILI9341_Fill_Screen(WHITE);
while(second>0){
	// Ekrandaki seyans dakika ve saniye degerleri
		sprintf(Temp_Buffer_text, "SESION:%d", sesion);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 5, WHITE);	
	
sprintf(Temp_Buffer_text, "Minute:%02d", minute);
  ILI9341_Draw_Text(Temp_Buffer_text, 10, 80, BLACK, 5, WHITE);
	
	sprintf(Temp_Buffer_text, "SECOND:%02d", second);
  ILI9341_Draw_Text(Temp_Buffer_text, 10, 150, BLACK, 5, WHITE);
	second--;
	HAL_Delay(1000);
}
second=11;
minute--;
}
minute=2;
sesion--;
}
sesion=0;
minute=0;
second=0;

HAL_Delay(1000);
ILI9341_Fill_Screen(WHITE);

sprintf(Temp_Buffer_text, "SESION:%d", sesion);
		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 5, WHITE);	
	
  sprintf(Temp_Buffer_text, "Minute:%02d", minute);
  ILI9341_Draw_Text(Temp_Buffer_text, 10, 80, BLACK, 5, WHITE);
	
	sprintf(Temp_Buffer_text, "SECOND:%02d", second);
  ILI9341_Draw_Text(Temp_Buffer_text, 10, 150, BLACK, 5, WHITE);
ILI9341_Fill_Screen(BLUE);
HAL_Delay(1000);

// Seyanslar bitince ekrana yazilan bilgiler
ILI9341_Draw_Text("FINISHED", 10, 10, BLACK, 5, WHITE);
ILI9341_Draw_Text("WEEL DONE", 10, 100, BLACK, 5, WHITE);
ILI9341_Draw_Text("GREAT", 10, 150, BLACK, 5, WHITE);
ILI9341_Draw_Text("WORK", 10, 200, BLACK, 5, WHITE);
HAL_Delay(2000);

ILI9341_Fill_Screen(RED);		
minute=2;
second=11;

//  Dinlenme bilgilerini içeren kod parçasi
while(breakk>0){
	sprintf(Temp_Buffer_text, "%02d", breakk);
  ILI9341_Draw_Text(Temp_Buffer_text, 100, 80, BLACK, 10, WHITE);
	breakk--;
	HAL_ADC_Start(&hadc1);
	// Adc degerinin degistirerek buzzerin çalisip çalismamasini kontrol etme
	if ( HAL_ADC_PollForConversion(&hadc1,1000) == HAL_OK){
			adcReturnValue = HAL_ADC_GetValue(&hadc1);
		 if(adcReturnValue<2048){
			 
		for(int i=0;i<1;i++){
	  	for(int j=0;j<1;j++){
	  	TIM1->CCR4 = 20000;
	    TIM1->ARR=30000;
	    HAL_Delay(400);
	  	TIM1->CCR4 = 20000;
	    TIM1->ARR=5000;
	  	HAL_Delay(400);

	  	}

	  	HAL_Delay(200);

	  }	

	  }	
	else
		 if(adcReturnValue<4096){
			 for(int i=0;i<1;i++){
	  	for(int j=0;j<1;j++){
	  	TIM1->CCR4 = 20000;
	    TIM1->ARR=0;
	   
	  	TIM1->CCR4 = 0;
	    TIM1->ARR=5000;
	  	

	  	}

	   HAL_Delay(1000);

	  }	
		 }
		HAL_ADC_Stop(&hadc1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
	
}	



breakk=20;// Break degerini tekrar 20 ye getiriyoruz diger seanslarda kullanmak için

}

	
		



		}

		
			
		
	
  }
  /* USER CODE END 3 */



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D2_Pin|D3_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin|LCD_RST_Pin|LCD_BCK_PWM_Pin 
                          |LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_CS_Pin 
                          |D0_Pin|D1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : D2_Pin D3_Pin D4_Pin D5_Pin 
                           D6_Pin D7_Pin LCD_RST_Pin LCD_BCK_PWM_Pin 
                           LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_CS_Pin 
                           D0_Pin D1_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin|LCD_RST_Pin|LCD_BCK_PWM_Pin 
                          |LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_CS_Pin 
                          |D0_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
