/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "test.h"
#include "OLED_1in51.h"
#include "heart_adc.h"
#include "JY62.h"
#include "string.h"
#include "aspro.h"
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
// 在main.c中，将BlackImage放在DTCM（0x20000000区域）
UBYTE BlackImage[1024];
UBYTE display_buf[30];
uint8_t Rx_buffer[64];
#define Rx_size 64
uint8_t Rx_len = 0;   // 接收到的数据长度
uint8_t flag = 0;       // 接收完成标志
JY62_Instance_t jy62;
uint8_t fall[30];
uint8_t Tx_Buffer[30];
uint8_t AS_buffer[64];
uint8_t AS_rx_buffer[10];
uint8_t AS_len = 0;
uint8_t light_time = 0;
uint16_t  len = 0;
uint16_t light_pin = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	uint32_t last_display_tick = 0;
	uint32_t last_jy_tick = 0;
	uint32_t last_aspro_tick = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
	System_Init(); 
  OLED_1IN51_Init();
  Paint_NewImage(BlackImage, OLED_1IN51_WIDTH, OLED_1IN51_HEIGHT, 0, BLACK);
  Paint_SelectImage(BlackImage); 
  uint32_t last_send_tick = 0;
  HAL_TIM_Base_Start_IT(&htim3);
	
	HAL_UART_Receive_DMA(&huart4, Rx_buffer, Rx_size);
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	
	JY62_Init(&jy62);
	HAL_UART_Receive_DMA(&huart5, jy62.rx_fifo, JY_BUF_SIZE);
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE); 
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			if(flag)
				{
					flag = 0;
					Paint_ClearWindows(0, 30, 63, 90, BLACK);
					Paint_DrawString_EN(0, 30,(char*)Rx_buffer, &Font16, WHITE, BLACK);
					OLED_1IN51_Display(BlackImage);
					if (HAL_GetTick() - last_aspro_tick >= 1000)
					{
						HAL_UART_Transmit_DMA(&huart7,AS_buffer,len);
						last_aspro_tick = HAL_GetTick();
					}
					//memset(Rx_buffer, 0, Rx_size);
				}
				if(HAL_GetTick() - last_jy_tick >= 100)
				{
					Paint_ClearWindows(0, 80, 64, 127, BLACK);
					sprintf((char*)fall,"Fall:%.1f%%",(jy62.fall_probability *100));
					 Paint_DrawString_EN(0, 80,(char*)fall, &Font16, WHITE, BLACK);
					//Paint_DrawNum(0, 90,(jy62.fall_probability *100), &Font16, 2, WHITE, BLACK);
					OLED_1IN51_Display(BlackImage);
					last_jy_tick = HAL_GetTick();
				}
			
				
		if (HAL_GetTick() - last_display_tick >= 500) 
    {
        //Paint_Clear(BLACK);
				Paint_ClearWindows(0, 0, 63, 60, BLACK);
        sprintf((char*)display_buf, "BPM: %d", BPM); 
        Paint_DrawString_EN(0, 0,(char*)display_buf, &Font16, WHITE, BLACK);
        
        if(QS) 
				{ // 如果检测到心跳瞬间，可以在屏幕上显示个符号
					QS = false; 
					Paint_DrawChar(50, 30, 'B', &Font8, WHITE, BLACK);
				}
				//Paint_ClearWindows(0, 0, 0, 60, BLACK);
        OLED_1IN51_Display(BlackImage);
				len = Process_All_Data((char*)Rx_buffer,BPM,(jy62.fall_probability *100),(char*)AS_buffer);
				
				HAL_UART_Transmit_DMA(&huart4,AS_buffer,len);
				if (AS_len > 0)
				{
        if (AS_rx_buffer[0] == '1')      
				{ 
					light_pin = GPIO_PIN_1;
					light_time = 0;
				}
				else if (AS_rx_buffer[0] == '2')
				{
					light_pin = GPIO_PIN_2; 
					light_time = 0; 
				}
        AS_len = 0; 
				}

				if (light_pin != 0)
				{
					HAL_GPIO_TogglePin(GPIOE, light_pin); // light_pin 是谁，就反转谁
					light_time++;
					if (light_time >= 10)
					{
            light_pin = 0; // 闪够10次（5秒），清零引脚，停止闪烁
					}
				}
        last_display_tick = HAL_GetTick();
    }
		if (HAL_GetTick() - last_aspro_tick >= 1000)
		{
			HAL_UART_Transmit_DMA(&huart7,AS_buffer,len);
			last_aspro_tick = HAL_GetTick();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
       HeartRate_Process(); // 核心采样和计算
			jy62.fall_probability = Fall_Probability(&jy62);
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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
