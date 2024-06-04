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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "core_cm3.h"
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
#define BASE_YEAR 2003
#define BASE_MONTH 2
#define BASE_DAY 12
#define BASE_HOUR 2
#define BASE_MINUTE 0
#define BASE_SECOND 0

// 定义位置
#define LATITUDE "2237.8840,N" // 22.6314°N
#define LONGITUDE "11009.2400,E" // 110.154°E

// 初始时间设置
uint32_t baseTime = 0; // 秒数

// GPS数据模拟
char gprmc_data[100];



// USART句柄声明
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;

// 定时器句柄声明
extern TIM_HandleTypeDef htim2; // 100Hz -- /100: PPS ,  /10: Trigger
extern TIM_HandleTypeDef htim3;


// 触发发送trigger的标志
volatile uint8_t trigger_flag = 0;
// 触发发送GPS数据的标志
volatile uint8_t send_gps_flag = 0;



void Error_Handler_Custom(void)
{

  while (1) {
    // 闪烁错误指示 LED 
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5); 
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); 
    HAL_Delay(500);
  }
}


void DWT_Init(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 启用DWT和ITM单元
        DWT->CYCCNT = 0; // 清零计数器
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 启用CYCCNT
    }
}
void DWT_Delay(uint32_t us) {   // 微秒级延时
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000); // 转换微秒为tick数

    while (DWT->CYCCNT - startTick < delayTicks);
}


// 校验和计算函数
uint8_t nmea_checksum(const char* sentence) {
    uint8_t checksum = 0;
    if (*sentence == '$') {
        sentence++;  // Skip '$'
    }
    
    while (*sentence && *sentence != '*') {
        checksum ^= *sentence++;
    }
    return checksum;
}



// GPRMC Generator
void GPRMC_Generator(uint32_t input_baseTime) {
    
    uint32_t culmulative_hms = 3600 * BASE_HOUR + 60 * BASE_MINUTE + BASE_SECOND;
    uint32_t currentTime = culmulative_hms + HAL_GetTick() / 1000;
    uint8_t hour = (currentTime / 3600) % 24;
    uint8_t minute = (currentTime / 60) % 60;
    uint8_t second = currentTime % 60;

    memset(gprmc_data, 0, sizeof(gprmc_data));

    // sprintf(gprmc_data, "$GPRMC,020008.00,A,2237.8840,N,11009.2400,E,0.00,0.00,120203,,,A*");
    sprintf(gprmc_data, "$GPRMC,%02d%02d%02d.00,A,%s,%s,0.00,0.00,%02d%02d%02d,,,A*",
            hour, minute, second, LATITUDE, LONGITUDE, BASE_DAY, BASE_MONTH, (BASE_YEAR % 100));
    

    // check sum
    uint8_t checksum = nmea_checksum(gprmc_data);
    
    int length = strlen(gprmc_data);
    sprintf(gprmc_data + length, "%02X\r\n", checksum);

}




volatile uint8_t trigger_led_flag = 0;
int trigger_led_count = 0;
volatile uint8_t pps_flag = 0;
int pps_count = 0;
volatile uint8_t gprmc_led_flag = 0;
int gprmc_led_count = 0;

volatile uint8_t uart_transmit_flag = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
      
        if (trigger_led_flag) {

            trigger_led_count ++;
            
        }
        if (pps_flag) {

            pps_count ++;

            if (pps_count >= 2)
            {
                pps_flag = 0;
                pps_count = 0;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // PPS LOW
            }
            
        }
        if (gprmc_led_flag) {

            gprmc_led_count ++;
            
        }
    }

}

// uint32_t pps_offset = 397;
uint32_t pps_offset = 390;
uint32_t for_delay = 35;
void TIM2_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
        
        static uint32_t count_10hz = 0, count_1hz = 0;
        
        if (++count_10hz >= 500) {
            count_10hz = 0;
            trigger_flag = 1;
        }
        
        if (++count_1hz >= 5000) {
            count_1hz = 0;
            send_gps_flag = 1;

            if(pps_flag) {
              Error_Handler();
            }
            DWT_Delay(pps_offset);
            for(int i = 0; i < for_delay; i ++);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // PPS HIGH
            pps_flag = 1;  // Set flag to turn off PPS in ISR after 100 ms
        }

    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1 || huart->Instance == USART3)
    {
        gprmc_led_flag = 0;
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
  DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Error_Handler();  
  }
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

  // if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK) {
  //   Error_Handler();
  // }
  // HAL_NVIC_SetPriority(TIM5_IRQn, 0, 1);
  //   HAL_NVIC_EnableIRQ(TIM5_IRQn);
  
  while (1) {

      if (trigger_led_flag && trigger_led_count >= 5)
      {
          trigger_led_flag = 0;
          trigger_led_count = 0;
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // Trigger LED OFF
      }
      
      if (!gprmc_led_flag && gprmc_led_count >= 5)
      {
          gprmc_led_count = 0;
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // LED GPRMC Sending OFF
      }
    

      if (trigger_flag) {

          trigger_flag = 0;

          // 点亮PB2表示触发信号已发送
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Trigger LED
          trigger_led_flag = 1;

          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4); // Toggle Trigger signal
          
      }

      if (send_gps_flag) {


          send_gps_flag = 0;

          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // LED GPRMC Sending ON
          gprmc_led_flag = 1;

          GPRMC_Generator(baseTime);
          HAL_UART_Transmit(&huart1, (uint8_t *)gprmc_data, strlen(gprmc_data), 6);
          // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)gprmc_data, strlen(gprmc_data));
          // HAL_UART_Transmit(&huart3, (uint8_t *)gprmc_data, strlen(gprmc_data), 70);
          HAL_UART_Transmit_DMA(&huart3, (uint8_t *)gprmc_data, strlen(gprmc_data));
          
          
          
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
