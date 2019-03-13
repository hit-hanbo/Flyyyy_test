#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "../HiSTM/MPU6050/MPU6050.h"
#include "../HiSTM/BSP.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "arm_math.h"
#include "../HiSTM/MPU6050/kalman.h"

extern arm_matrix_instance_f32  HiSTM_Kalman_Status_Matrix_Xk;
extern arm_matrix_instance_f32  HiSTM_Kalman_FactMeasurement_Matrix_Z;
extern float32_t HiSTM_Kalman_Status_Matrix_Xk_src[3];
extern float32_t Angle_X_Final; //X◊Ó÷’«„–±Ω«∂»
extern float32_t Angle_Y_Final; //Y◊Ó÷’«„–±Ω«∂»

void SystemClock_Config(void);

int main(void)
{
	volatile MPU6050_RawDataStructTypeDef temp_data;
	volatile uint8_t temperature;
	char     str[2048];
	uint32_t i;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_SPI3_Init();
    MX_TIM2_Init();
    MX_TIM10_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_USB_DEVICE_Init();
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    while( HiSTM_MPU6050_Init() != MPU6050_DEV_OK);
    STA_LD_OFF;



    while (1)
    {
    	STA_LD_ON;
    	HAL_Delay(1);
    	temp_data = HiSTM_MPU6050_Get_RawData();
    	temperature = temp_data.Temperature / 340 + 36;
    	STA_LD_OFF;
    	Angle_Calcu();

//    	sprintf(str, "{ 'Gyro_X' :%d, 'Gyro_y':%d, 'Gyro_Z':%d, 'Accel_X':%d, 'Accel_Y':%d, 'Accel_Z':%d, 'Temperature':%d }\r\n", \
//    			temp_data.Gyro_X, temp_data.Gyro_Y, temp_data.Gyro_Z, \
//				temp_data.Accel_X, temp_data.Accel_Y, temp_data.Accel_Z, temperature);
    	if(i % 10 == 0)
    	{
    		sprintf(str, "{ X: %f, Y: %f }\r\n", Angle_X_Final, Angle_Y_Final);
    		CDC_Transmit_FS(str, strlen(str));
    	}
    	i ++;
    }
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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
