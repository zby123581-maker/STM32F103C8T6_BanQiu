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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 坐标校正参数（由四角像素坐标计算得出）
// 四角：左上(175,50) 右上(574,46) 左下(188,459) 右下(566,463)
// 中心：(374, 254)
#define CAM_CENTER_X  374.0f    // 平台中心X像素坐标
#define CAM_CENTER_Y  254.0f    // 平台中心Y像素坐标
#define CAM_SCALE_X   194.25f   // X方向半径像素数：(374-181.5 + 570-374) / 2
#define CAM_SCALE_Y   206.5f    // Y方向半径像素数：(254-48 + 461-254) / 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


float ball_x = 0.0f;   // 归一化误差（中心=0，边界≈±1）
float ball_y = 0.0f;


float servo_x_angle = 90.0f;
float servo_y_angle = 90.0f;

// 位置环参数（外环，20ms周期）
float pos_Kp = 0.50f;
float pos_Kd = 6.00f;

float pos_prev_error_x = 0.0f;
float pos_prev_error_y = 0.0f;

// 速度环参数（内环，10ms周期）
float vel_Kp = 0.75f;
float vel_Kd = 4.00f;

float vel_prev_ball_x = 0.0f;
float vel_prev_ball_y = 0.0f;

// 位置环输出：目标速度
float target_vx = 0.0f;
float target_vy = 0.0f;

// 位置环计数器（每2次速度环跑一次位置环）
uint8_t pos_loop_cnt = 0;


float offset_x = 3.0f;
float offset_y = -3.0f;

uint32_t last_debug_time = 0;
volatile uint8_t control_flag = 0;
volatile uint8_t debug_tick = 0; 



#define K230_RX_BUF_SIZE  128   

uint8_t k230_rx_buf[K230_RX_BUF_SIZE]; 
volatile uint8_t k230_frame_ready = 0;   


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_debug_data(void)
{
    // 先用简单格式排查问题，确认正常后再换回详细格式
    char buf[64];
    int wucha_x = (int)(ball_x * 100);
    int wucha_y = (int)(ball_y * 100);
    int duoji_x = (int)(servo_x_angle * 10);
    int duoji_y = (int)(servo_y_angle * 10);
    int len = sprintf(buf, "wucha_x:%d wucha_y:%d duoji_x:%d duoji_y:%d\r\n",
                      wucha_x, wucha_y, duoji_x, duoji_y);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, HAL_MAX_DELAY);
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, k230_rx_buf, K230_RX_BUF_SIZE);

    HAL_TIM_Base_Start_IT(&htim4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


        // 解析K230坐标（有新数据就解析，不管PD周期）
        if (k230_frame_ready == 1)
        {
            k230_frame_ready = 0;

            if (k230_rx_buf[0] == 0xAA && k230_rx_buf[1] == 0x55)
            {
                uint8_t checksum = (0xAA + 0x55 + k230_rx_buf[2] + k230_rx_buf[3]
                                   + k230_rx_buf[4] + k230_rx_buf[5]) & 0xFF;

                if (checksum == k230_rx_buf[6])
                {
                    int16_t raw_x = (int16_t)((k230_rx_buf[2] << 8) | k230_rx_buf[3]);
                    int16_t raw_y = (int16_t)((k230_rx_buf[4] << 8) | k230_rx_buf[5]);
                   
                    ball_x = ((float)raw_x - CAM_CENTER_X) / CAM_SCALE_X;
                    ball_y = ((float)raw_y - CAM_CENTER_Y) / CAM_SCALE_Y;
                }
            }
        }

        // TIM4每10ms触发一次控制
        if (control_flag)
        {
            control_flag = 0;

            // ========== 速度环（内环，每10ms执行） ==========
            // 计算当前速度（ball_x已是归一化单位）
            float ball_vx = ball_x - vel_prev_ball_x;
            float ball_vy = ball_y - vel_prev_ball_y;

            // 速度死区
            if (ball_vx > -0.1f && ball_vx < 0.1f) ball_vx = 0;
            if (ball_vy > -0.1f && ball_vy < 0.1f) ball_vy = 0;

            // 速度误差 = 目标速度 - 实际速度
            float vel_error_x = target_vx - ball_vx;
            float vel_error_y = target_vy - ball_vy;

            // 速度环PD
            float vel_out_x = vel_Kp * vel_error_x + vel_Kd * ball_vx;
            float vel_out_y = vel_Kp * vel_error_y + vel_Kd * ball_vy;

            servo_x_angle = 90.0f - vel_out_x * 3.0f + offset_x;
            servo_y_angle = 90.0f + vel_out_y * 3.0f + offset_y;

            // 限幅
            if (servo_x_angle < 65) servo_x_angle = 65;
            if (servo_x_angle > 115) servo_x_angle = 115;
            if (servo_y_angle < 65) servo_y_angle = 65;
            if (servo_y_angle > 115) servo_y_angle = 115;

            // 输出PWM
            uint16_t servo_x_pwm = (uint16_t)(500 + servo_x_angle * 11.11f);
            uint16_t servo_y_pwm = (uint16_t)(500 + servo_y_angle * 11.11f);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_x_pwm);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo_y_pwm);

            vel_prev_ball_x = ball_x;
            vel_prev_ball_y = ball_y;

            // ========== 位置环（外环，每20ms执行一次） ==========
            pos_loop_cnt++;
            if (pos_loop_cnt >= 2)
            {
                pos_loop_cnt = 0;

                float error_x = ball_x;  // 已经是归一化误差
                float error_y = ball_y;

                // 误差死区
                if (error_x > -0.2f && error_x < 0.2f) error_x = 0;
                if (error_y > -0.2f && error_y < 0.2f) error_y = 0;

                // 位置环PD：输出目标速度
                float d_error_x = error_x - pos_prev_error_x;
                float d_error_y = error_y - pos_prev_error_y;
                target_vx = pos_Kp * error_x + pos_Kd * d_error_x;
                target_vy = pos_Kp * error_y + pos_Kd * d_error_y;

                pos_prev_error_x = error_x;
                pos_prev_error_y = error_y;
            }
        }


       
        if (debug_tick >= 20)
        {
            debug_tick = 0;
            send_debug_data();
        }


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


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        if (Size > 0)
        {
            k230_rx_buf[Size] = '\0';

            k230_frame_ready = 1;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, k230_rx_buf, K230_RX_BUF_SIZE);
    }
}

// TIM4每10ms中断一次，置位控制标志（速度环周期）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        control_flag = 1;
        debug_tick++;
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
