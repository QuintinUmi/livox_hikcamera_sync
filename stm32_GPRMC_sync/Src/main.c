/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// 定义基准时间 2003-02-12 02:00:00
#define BASE_YEAR 2013
#define BASE_MONTH 9
#define BASE_DAY 20
#define BASE_HOUR 2
#define BASE_MINUTE 0
#define BASE_SECOND 0

// 定义位置
#define LATITUDE "2237.8840,N" // 22.6314°N
#define LONGITUDE "11009.2400,E" // 110.154°E

// 初始时间设置
uint32_t baseTime = 0; // 秒数

// GPS数据模拟
const char gprmc_data[100];



// USART句柄声明
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;

// 定时器句柄声明
extern TIM_HandleTypeDef htim2; // 100Hz -- /100: PPS ,  /10: Trigger

// 触发发送GPS数据的标志
volatile uint8_t send_gps_flag = 0;


void Error_Handler_Custom(void)
{

  while (1) {
    // 闪烁错误指示 LED 或其他错误提示
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5); 
    HAL_Delay(500);
  }
}

// 校验和计算函数
uint8_t nmea_checksum(const char* sentence) {
    uint8_t checksum = 0;
    if (*sentence == '$') {
        sentence++;  // 跳过起始的 '$'
    }
    while (*sentence && *sentence != '*') {
        checksum ^= *sentence++;
    }
    return checksum;
}

// GPRMC 句子生成函数
void GPRMC_Generator(uint32_t input_baseTime) {
    // 计算当前时间
    uint32_t currentTime = input_baseTime + HAL_GetTick() / 1000;
    uint8_t hour = (currentTime / 3600) % 24;
    uint8_t minute = (currentTime / 60) % 60;
    uint8_t second = currentTime % 60;

    memset(gprmc_data, 0, sizeof(gprmc_data));
    // 格式化 GPRMC 字符串，不包括校验和
    sprintf(gprmc_data, "$GPRMC,%02d%02d%02d,A,%s,%s,0.004,133.4,%02d%02d%02d,0.0,E,D",
            hour, minute, second, LATITUDE, LONGITUDE, BASE_DAY, BASE_MONTH, BASE_YEAR % 100);
    // snprintf(gprmc_data, "$GPRMC,%02d%02d%02d.00,A,5109.0262308,N,11401.8407342,W,0.004,133.4,130920,0.0,E,D",
		// 			hour, minute, second);

    // 计算校验和
    uint8_t checksum = nmea_checksum(gprmc_data);
    // 将校验和追加到字符串
    int length = strlen(gprmc_data);
    sprintf(gprmc_data + length, "*%02X\n", checksum);

}


void TIM2_IRQHandler(void) {
    static uint32_t count_10hz = 0;
    static uint32_t count_1hz = 0;
    
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
        
        count_10hz++;
        count_1hz++;
        
        if (count_10hz >= 10) {        // 100 Hz / 10 = 10 Hz

            count_10hz = 0;
            // 每秒10次
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4); // Toggle Trigger signal
        }
        if((int)(count_1hz / 10) % 2 == 1) {
          // 点亮PB2表示触发信号已发送
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);        
        } else {
          // 熄灭PB2，表示触发信号完成
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        }
        
        if (count_1hz >= 100) {        // 100 Hz / 100 = 1 Hz

            count_1hz = 0;
            // 每秒触发一次
            GPRMC_Generator(baseTime);
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); // Toggle PPS signal
            // baseTime ++;
            send_gps_flag = 1;
            
            // 点亮PC5表示正在发送GPS数据
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
        }

    }
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // 启动定时器
  HAL_TIM_Base_Start_IT(&htim2);
  
  
  while (1) {
      if (send_gps_flag) {

          // 发送GPS数据到 USART1
          int transmitStatus1 = HAL_UART_Transmit(&huart1, (uint8_t *)gprmc_data, strlen(gprmc_data), 100);
          if (transmitStatus1 != HAL_OK) {
            // 处理 USART1 错误
            Error_Handler_Custom(); // 自定义的错误处理函数
          }

          // 发送GPS数据到 USART3
          int transmitStatus3 = HAL_UART_Transmit(&huart3, (uint8_t *)gprmc_data, strlen(gprmc_data), 100);
          if (transmitStatus3 != HAL_OK) {
            // 处理 USART3 错误
            Error_Handler_Custom(); // 自定义的错误处理函数
          }

          send_gps_flag = 0;
          
          // 熄灭PC5，表示GPS数据发送完成
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
      }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  Error_Handler_Custom();
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
