
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
float ax, ay, az, gx, gy, gz, temperature, roll, pitch;
uint8_t buffer[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void UART2_SendString(char* s)
{
 HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
}
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init(&hi2c1);

  MPU6050_SetInterruptMode(MPU6050_INTMODE_ACTIVEHIGH);
  MPU6050_SetInterruptDrive(MPU6050_INTDRV_PUSHPULL);
  MPU6050_SetInterruptLatch(MPU6050_INTLATCH_WAITCLEAR);
  MPU6050_SetInterruptLatchClear(MPU6050_INTCLEAR_STATUSREAD);

  MPU6050_SetIntEnableRegister(0); // Disable all interrupts

  // Enable Motion interrputs
  MPU6050_SetDHPFMode(MPU6050_DHPF_5);

  MPU6050_SetIntMotionEnabled(1);
  MPU6050_SetIntZeroMotionEnabled(1);
  MPU6050_SetIntFreeFallEnabled(1);

  MPU6050_SetFreeFallDetectionDuration(2);
  MPU6050_SetFreeFallDetectionThreshold(5);

  MPU6050_SetMotionDetectionDuration(5);
  MPU6050_SetMotionDetectionThreshold(2);

  MPU6050_SetZeroMotionDetectionDuration(2);
  MPU6050_SetZeroMotionDetectionThreshold(4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  MPU6050_GetAccelerometerScaled(&ax, &ay, &az);
	  MPU6050_GetGyroscopeScaled(&gx, &gy, &gz);
	  temperature = MPU6050_GetTemperatureCelsius();
	  memset(buffer, 0, 128);
	  sprintf((char*)buffer, "ACC: X: %.2f Y:%.2f Z:%.2f \n\rGYR: X: %.2f Y:%.2f Z:%.2f\n\rTEMP: %.2f\n\r", ax, ay, az, gx, gy, gz, temperature);
	  UART2_SendString((char*)buffer);

	  MPU6050_GetRollPitch(&roll, &pitch);
	  memset(buffer, 0, 128);
	  sprintf((char*)buffer, "RPY: Roll: %.2f Pitch: %.2f\n\r\n\r", roll, pitch);
	  UART2_SendString((char*)buffer);
	  HAL_Delay(100);
	  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == INT_Pin)
	{
		uint8_t interrupts = MPU6050_GetIntStatusRegister();
		MPU6050_GetMotionStatusRegister();
		sprintf((char*)buffer, "Int status triggered: %X\n\n\r", interrupts);
		UART2_SendString((char*)buffer);

		if(interrupts & (1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) // Bit 4 (0x10)
		{
			sprintf((char*)buffer, "FIFO Overflow detected\n\r");
			UART2_SendString((char*)buffer);
		}

		if(interrupts & (1<<MPU6050_INTERRUPT_MOT_BIT))	// Bit 6 (0x40)
		{
			sprintf((char*)buffer, "Motion detected\n\r");
			UART2_SendString((char*)buffer);
		}

		if(interrupts & (1<<MPU6050_INTERRUPT_ZMOT_BIT))	// Bit 5 (0x20)
		{
			sprintf((char*)buffer, "Zero Motion detected\n\r");
			UART2_SendString((char*)buffer);
		}

		if(interrupts & (1<<MPU6050_INTERRUPT_FF_BIT))	// Bit 7 (0x80)
		{
			sprintf((char*)buffer, "Freefall detected\n\r");
			UART2_SendString((char*)buffer);
		}

	}
}
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
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
