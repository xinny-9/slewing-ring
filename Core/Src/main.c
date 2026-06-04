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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "serial_servo.h"
#include "serial_servo_debug_cli.h"
#include "serial_servo_hal.h"
#include "Emm_V5.h"

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
/* 声明步进电机句柄 */
Emm_V5_Motor stepper;

/* 串口异步接收相关的缓存定义 (假设使用串口2连接电机) */
#define RX_BUFFER_SIZE  64
uint8_t g_stepper_rx_buf[RX_BUFFER_SIZE];
uint8_t g_stepper_rx_len = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

 /* 1. 初始化电机句柄：绑定串口2 (&huart2)，设置电机总线地址为1 */
    Emm_V5_Init(&stepper, &huart2, 1);
    
    /* 2. 确保电机驱动器上电稳定后，使能电机 */
    HAL_Delay(1000); 
    Emm_V5_En_Control(&stepper, true, false);
    HAL_Delay(200);

    /* 3. 阻塞式读取当前电机的状态，验证通信是否正常 */
    printf(">> motor's initial state...\r\n");
    if (Emm_V5_Read_Status_Blocking(&stepper))
    {
        printf(">> en_state: %d, arrive_state: %d, lock_state: %d\r\n", 
               stepper.en_state, stepper.arrive_state, stepper.lck_state);
    }
    else
    {
        printf(">> communication failed.\r\n");
    }



 /* 初始化总线物理层驱动并挂载回调，同时启动 huart1 的首次 HAL 中断接收监听 */
  Serial_Servo_HAL_Init();



  /* 初始化独立的 PC 串口命令行调试控制台，开启 huart3 的首次标准 1 字节中断接收监听 */
  Debug_CLI_Init();
  printf(">>串口调试初始化完成\r\n");

  uint32_t last_action_tick = 0;
  uint8_t toggle_pos_flag = 0;

  
  /*  开启 TIM3 的 10ms 定时中断服务，挂载遥测 */
  HAL_TIM_Base_Start_IT(&htim3);
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

     Debug_CLI_Process();

 /* 5. 位置控制演示：相对运动 */
    printf(">> 运动演示：顺时针旋转一圈 (在16细分下发送3200个脉冲，速度1000RPM，加速度5)\r\n");
    Emm_V5_Pos_Control(&stepper, EMM_CW, 500, 5, 6400, false, false);
    HAL_Delay(2000); // 等待运动完成
  printf(">> 运动演示：逆时针旋转一圈 (在16细分下发送3200个脉冲，速度1000RPM，加速度5)\r\n");
    Emm_V5_Pos_Control(&stepper, EMM_CCW, 500, 5, 6400, false, false);
    HAL_Delay(2000); // 等待运动完成


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

// =================================================================
// 1. TIM3 10ms interrupt - Control Loop and Telemetry Report
// =================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
      {



      }

}


/**
  * @brief  标准 HAL 库串口接收中断完成回调函数 (USER CODE 4 区域)
  * @note   在此处极度优雅、对称地分发处理来自不同串口设备的数据包，零侵入 stm32f1xx_it.c！
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 1. 处理来自串口 1 (总线舵机) 的回传接收字节数据 */
    Serial_Servo_RxCallback(huart);
    
    /* 2. 处理来自串口 2 (PC 调试控制台) 的英文字符命令字节数据 */
    Debug_CLI_RxCallback(huart);
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
