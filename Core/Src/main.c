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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_REC_LEN  			100  	//定义最大接收字节数 200
#define RXBUFFERSIZE   			1 		//缓存大小
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t USART2_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
uint16_t USART2_RX_STA = 0; //接收状态标记
uint8_t aRxBuffer2[RXBUFFERSIZE];		  //HAL库使用的串口接收缓冲
BaseSerialData SBUS_CH;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int rec_cnt = 0;

int PWMcenter =1500;
int PWMlow = 1000;
int PWMhigh = 2000;

int PWM_CH1_rec = 1500;
int PWM_CH2_rec = 1500;
int PWM_CH3_rec = 1500;
int PWM_CH4_rec = 1500;

float x_angle = 0.0;
float y_angle = 0.0;
float z_angle = 0.0;
float init_z_angle = 0.0;

int init_z_flag = 0;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float speed_low = 1.0;
	//uint8_t send_data[5] = {0xA1, 0xF1, 0x1C, 0x2F, 0x33};

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
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer2, RXBUFFERSIZE); //该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
	//HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	
	if (HAL_GPIO_ReadPin(SPEED95_GPIO_Port, SPEED95_Pin) == GPIO_PIN_RESET)
	{
		speed_low = 0.95;
	}
	if (HAL_GPIO_ReadPin(SPEED90_GPIO_Port, SPEED90_Pin) == GPIO_PIN_RESET)
	{
		speed_low = 0.90;
	}
	if (HAL_GPIO_ReadPin(SPEED85_GPIO_Port, SPEED85_Pin) == GPIO_PIN_RESET)
	{
		speed_low = 0.85;
	}
	if (HAL_GPIO_ReadPin(SPEED80_GPIO_Port, SPEED80_Pin) == GPIO_PIN_RESET)
	{
		speed_low = 0.80;
	}
	
	if (HAL_GPIO_ReadPin(SPEED75_GPIO_Port, SPEED75_Pin) == GPIO_PIN_RESET)
	{
		speed_low = 0.75;
	}
	if (HAL_GPIO_ReadPin(SPEED70_GPIO_Port, SPEED70_Pin) == GPIO_PIN_RESET)
	{
		speed_low = 0.70;
	}
	if (HAL_GPIO_ReadPin(SPEED65_GPIO_Port, SPEED65_Pin) == GPIO_PIN_RESET)
	{
		speed_low = 0.65;
	}
	if (HAL_GPIO_ReadPin(SPEED60_GPIO_Port, SPEED60_Pin) == GPIO_PIN_RESET)
	{
		speed_low = 0.60;
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1500);
	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1500);
	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1500);
	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1500);
  while (1)
  {
		HAL_Delay(10);
		if (rec_cnt%100 == 0 && rec_cnt > 30)
		{
			HAL_GPIO_TogglePin(LED_SHOW_GPIO_Port,LED_SHOW_Pin);
		}
		if (init_z_flag != 0)
		{
			int z_tmp = 0;
			int x_tmp = 0;
			int y_tmp = 0;
			
			z_tmp = z_angle - init_z_angle;
			x_tmp = x_angle;
			y_tmp = y_angle;
			
			if (z_tmp < -180) z_tmp = 180 - (-180 - z_tmp);
			if (z_tmp > 180) z_tmp = -180 + (z_tmp - 180);
			
			if (abs(z_tmp) < 10) z_tmp = 0;

			if (abs(x_tmp) < 10) 
			{
				x_tmp = 0;
			}

			if (abs(y_tmp) < 10) y_tmp = 0;
			
			float dutyx = 0;
			float dutyy = 0;
			float servo = 0;
			
			dutyx = -x_tmp/90.f*speed_low;
			dutyy = -y_tmp/90.f*speed_low;
			servo = z_tmp/90.f*speed_low;
			
			float MOTOR_A = 0;
      float MOTOR_B = 0;
      float MOTOR_C = 0;
      float MOTOR_D = 0;
			
			MOTOR_A = +dutyx + dutyy - servo * (0.2 + 0.15);
      MOTOR_B = -dutyx + dutyy - servo * (0.2 + 0.15);
			MOTOR_C = +dutyx + dutyy + servo * (0.2 + 0.15);
      MOTOR_D = -dutyx + dutyy + servo * (0.2 + 0.15);
			
			//1 B
			PWM_CH1_rec = PWMcenter + 800 * MOTOR_B;
			
			//2 C
			PWM_CH2_rec = PWMcenter + 800 * MOTOR_C;
			
			//3 A
			PWM_CH3_rec = PWMcenter + 800 * MOTOR_A;
			
			//4 D
			PWM_CH4_rec = PWMcenter + 800 * MOTOR_D;
			
		}
		if (rec_cnt++ > 25)
		{
			PWM_CH4_rec = PWM_CH3_rec = PWM_CH2_rec = PWM_CH1_rec = PWMcenter;
		}
		//font left TIM1 CH1 CH2
		int set_pwm = PWM_CH1_rec - PWMcenter;
		if (set_pwm == 0)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, set_pwm);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, set_pwm);
		}
		else if (set_pwm > 0)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, set_pwm);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -set_pwm);
		}
		// font right TIM1 CH3 CH4
		set_pwm = PWM_CH2_rec - PWMcenter;
		if (set_pwm == 0)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, set_pwm);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, set_pwm);
		}
		else if (set_pwm > 0)
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, set_pwm);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -set_pwm);
		}
		// rear left TIM2 CH1 CH2
		set_pwm = PWM_CH3_rec - PWMcenter;
		if (set_pwm == 0)
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, set_pwm);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, set_pwm);
		}
		else if (set_pwm > 0)
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, set_pwm);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -set_pwm);
		}
		
		// rear right TIM2 CH3 CH4
		set_pwm = PWM_CH4_rec - PWMcenter;
		if (set_pwm == 0)
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, set_pwm);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, set_pwm);
		}
		else if (set_pwm > 0)
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, set_pwm);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -set_pwm);
		}
		
		
		//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_CH1_rec);
		//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_CH2_rec);
		//HAL_UART_Transmit(&huart1,send_data,5,10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	if (htim->Instance == htim2.Instance)
//	{
//		pwmcnt++;
//		if (pwmcnt < PWM_CH1_check)
//		{
//			HAL_GPIO_WritePin(PWM_CH1_GPIO_Port, PWM_CH1_Pin,GPIO_PIN_SET);
//		}else
//		{
//			HAL_GPIO_WritePin(PWM_CH1_GPIO_Port, PWM_CH1_Pin,GPIO_PIN_RESET);
//		}
//		
//		if (pwmcnt < PWM_CH2_check)
//		{
//			HAL_GPIO_WritePin(PWM_CH2_GPIO_Port, PWM_CH2_Pin,GPIO_PIN_SET);
//		}else
//		{
//			HAL_GPIO_WritePin(PWM_CH2_GPIO_Port, PWM_CH2_Pin,GPIO_PIN_RESET);
//		}
//		if (pwmcnt >= PWM1ms*10)
//		{
//			PWM_CH1_check = PWM_CH1_rec;
//			PWM_CH2_check = PWM_CH2_rec;
//			pwmcnt = 0;
//		}
//	}
}

int Usart1_Flag = 0;
uint8_t Usart1Rxbuf[38];
int check_sum = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int pwm_rec = 0;
	//float pwm_percent = 0;
	
	if (huart->Instance == USART2) //如果是串口2
	{	
		//HAL_UART_Transmit(&huart1,aRxBuffer2,1,10);
		uint8_t temp;
		static uint8_t U1_count,
									U1_last_data,
									U1_last_last_data;
		temp = aRxBuffer2[0];
		if(Usart1_Flag==0)
		{	
			if(U1_last_data==0x53&&U1_last_last_data==0x55)
			{ 
				Usart1_Flag=1,U1_count=0;	
				Usart1Rxbuf[U1_count++] = 0x55;
				Usart1Rxbuf[U1_count++] = 0x53;
				check_sum = 0x55+0x53;
			}
		}
		
		if(Usart1_Flag==1)
		{	
			
			Usart1Rxbuf[U1_count]=temp;
			     
			U1_count++; 
			if (U1_count < 11) check_sum += temp;			
			if(U1_count==11)
			{
				Usart1_Flag=0;
		
				//方向
				if (check_sum % 0x100 == temp)
				{
					HAL_GPIO_TogglePin(LED_SHOW_GPIO_Port,LED_SHOW_Pin);
					rec_cnt = 0;
					int k_angle = 180;
					
					float x_angle_tmp = (float)((Usart1Rxbuf[3] << 8)|Usart1Rxbuf[2])/32768.f*k_angle;
					float y_angle_tmp = (float)((Usart1Rxbuf[5] << 8)|Usart1Rxbuf[4])/32768.f*k_angle;
					float z_angle_tmp = (float)((Usart1Rxbuf[7] << 8)|Usart1Rxbuf[6])/32768.f*k_angle;
					
					(x_angle_tmp >= k_angle) ? (x_angle = x_angle_tmp - 2 * k_angle) : (x_angle = x_angle_tmp);
					(y_angle_tmp >= k_angle) ? (y_angle = y_angle_tmp - 2 * k_angle) : (y_angle = y_angle_tmp);
					(z_angle_tmp >= k_angle) ? (z_angle = z_angle_tmp - 2 * k_angle) : (z_angle = z_angle_tmp);
					
					
					if (init_z_flag == 0)
					{
						init_z_angle = z_angle;
						init_z_flag = 1;
					}
				}
					
				
			}
		}
		else
		{
		  Usart1_Flag =0;
		}

		U1_last_last_data=U1_last_data;
		U1_last_data=temp;
		
		HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer2, RXBUFFERSIZE); //该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
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
