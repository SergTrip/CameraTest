/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 06/06/2015 13:19:16
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "crc.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

#include "camera_i2c.h"
#include "dcmi_ov9655.h"
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */
#define OV9655_DEVICE_WRITE_ADDRESS    0x60
#define OV9655_DEVICE_READ_ADDRESS     0x61

#define BNO055_DEVICE_ADDRESS 				0x28

#define  TIMEOUT  2

#define  CAMERA_RST_PIN			  GPIO_PIN_12	
#define  CAMERA_RST_PORT		  GPIOD

#define  CAMERA_PWR_EN_PIN		GPIO_PIN_6
#define  CAMERA_PWR_EN_PORT		GPIOD

uint32_t 						dcmiError = 0;
HAL_StatusTypeDef		dcmiStatus; 

uint16_t imageBuffer[1280]; 

uint16_t vsyncCounter 	= 0;

uint16_t lineCounter		= 0;
uint16_t fullFrameLines = 0;

uint16_t frameCounter 	= 0;

uint16_t errorCounter 	= 0;

#define DCMI_TIMEOUT_MAX  10000

HAL_StatusTypeDef i2cRes;
uint8_t 					IDData[] = {0, 0, 0, 0};

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* System interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
	
	for(int i = 0; i < 1280; i++ )
		imageBuffer[i] = i;
		
	/*Reset camera*/
  HAL_GPIO_WritePin(CAMERA_RST_PORT, CAMERA_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(CAMERA_RST_PORT, CAMERA_RST_PIN, GPIO_PIN_SET);
	
	/* camera PWR EN pin configuration */
  HAL_GPIO_WritePin(CAMERA_PWR_EN_PORT, CAMERA_PWR_EN_PIN, GPIO_PIN_RESET);
	
	// Пробуем читать ID устройства ( Это рабочая фукция !!! ) *********************************
	i2cRes = HAL_I2C_Mem_Read ( 					&hi2c1, 
															(uint16_t)BNO055_DEVICE_ADDRESS<<1,		// Адрес устройства
																				0x2, 												// Адрес регистра
																				I2C_MEMADD_SIZE_8BIT,				// Размерность данных
																				IDData, 										// Буфер для получения данных
																				4, 													// Сколько данных нужно получить
																				100);
	//********************************************************************************************
	
	//IDData[0] = DCMI_SingleRandomRead(OV9655_DEVICE_WRITE_ADDRESS, OV9655_MIDH);	
	//IDData[1] = DCMI_SingleRandomRead(OV9655_DEVICE_WRITE_ADDRESS, OV9655_MIDL);
	
		// Пробуем поменять режим устройства	
		i2cRes = HAL_I2C_Mem_Read	( 				&hi2c1, 
															(uint16_t)OV9655_DEVICE_WRITE_ADDRESS,	// Адрес устройства
																				OV9655_MIDH, 									// Адрес регистра
																				I2C_MEMADD_SIZE_8BIT, 				// Размерность данных
																				IDData, 											// Буфер для получения данных
																				2, 														// Сколько данных нужно отправить
																				100);

	// Пробуем читать ID устройства ( Это рабочая фукция !!! ) *********************************
//	i2cRes = HAL_I2C_Mem_Read ( 					&hi2c1, 
//															(uint16_t)BNO055_DEVICE_ADDRESS<<1,		// Адрес устройства
//																				0x2, 												// Адрес регистра
//																				I2C_MEMADD_SIZE_8BIT,				// Размерность данных
//																				IDData, 										// Буфер для получения данных
//																				4, 													// Сколько данных нужно получить
//																				100);
	//********************************************************************************************
	
	// Запустить камеру
	dcmiStatus = HAL_DCMI_Start_DMA ( &hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)((uint8_t*)imageBuffer), 1280 );
	
	if( HAL_OK != dcmiStatus )
	{		
		dcmiError = HAL_DCMI_GetError ( &hdcmi );
		while (1){}
	}

  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}

/* USER CODE BEGIN 4 */


/**
  * @brief  Error DCMI callback.
  * @param  hdcmi: pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval None
  */
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi)
{
	dcmiError = HAL_DCMI_GetError ( hdcmi );
	
	errorCounter++;
	// while(1){}
}

/**
  * @brief  Line Event callback.
  * @param  hdcmi: pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval None
  */
void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  // Закончилось считывание линии
		lineCounter++;
}

/**
  * @brief  VSYNC Event callback.
  * @param  hdcmi: pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval None
  */
void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	vsyncCounter++;
	//lineCounter = 0;
	
	__HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_VSYNC); 
}

/**
  * @brief  Frame Event callback.
  * @param  hdcmi: pointer to a DCMI_HandleTypeDef structure that contains
  *                the configuration information for DCMI.
  * @retval None
  */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	// Законилась загрузка рисунка
	frameCounter++;
	fullFrameLines = lineCounter;
	lineCounter = 0;	
		
	__HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_FRAME);
}

/* USER CODE END 4 */

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
