/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "i2c.h"
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lsm6dsv16x_read_data_polling(void);
void lsm6dsv16x_read_data_irq(void);
void lsm6dsv16x_self_test(void);
void lsm6dsv16x_compressed_fifo(void);
void lsm6dsv16x_fifo(void);
void lsm6dsv16x_fifo_irq(void);
void lsm6dsv16x_sensor_fusion(void);
void lsm6dsv16x_qvar_read_data_polling(void);
void lsm6dsv16x_free_fall(void);
void lsm6dsv16x_wakeup(void);
void lsm6dsv16x_sixd(void);
void lsm6dsv16x_single_double_tap(void);
void lsm6dsv16x_fsm_glance(void);
void lsm6dsv16x_fsm_fourd(void);
void lsm6dsv16x_mlc_gym(void);

void lsm6dsv16bx_read_data_polling(void);
void lsm6dsv16bx_activity(void);
void lsm6dsv16bx_fifo(void);
void lsm6dsv16bx_pedometer(void);
void lsm6dsv16bx_self_test(void);
void lsm6dsv16bx_tdm_conf(void);
void lsm6dsv16bx_wake_up(void);
void lsm6dsv16bx_sensor_fusion(void);

void lsm6dsv32x_read_data_polling(void);
void lsm6dsv32x_fifo(void);
void lsm6dsv32x_self_test(void);
void lsm6dsv32x_sensor_fusion(void);

void lsm6dsv16x_read_data_irq_handler(void);
void lsm6dsv16x_fifo_irq_handler(void);
void lsm6dsv16x_free_fall_handler(void);
void lsm6dsv16x_wakeup_handler(void);
void lsm6dsv16x_sixd_handler(void);
void lsm6dsv16x_single_double_tap_handler(void);
void lsm6dsv16x_fsm_glance_handler(void);
void lsm6dsv16x_fsm_fourd_handler(void);
void lsm6dsv16x_mlc_gym_handler(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //lsm6dsv16x_read_data_irq_handler();
  //lsm6dsv16x_fifo_irq_handler();
  //lsm6dsv16x_free_fall_handler();
  //lsm6dsv16x_wakeup_handler();
  //lsm6dsv16x_sixd_handler();
  //lsm6dsv16x_single_double_tap_handler();
  //lsm6dsv16x_fsm_glance_handler();
  //lsm6dsv16x_fsm_fourd_handler();
  lsm6dsv16x_mlc_gym_handler();
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //lsm6dsv16x_read_data_polling();
    //lsm6dsv16x_read_data_irq();
    //lsm6dsv16x_self_test();
    //lsm6dsv16x_compressed_fifo();
    //lsm6dsv16x_sensor_fusion();
    //lsm6dsv16x_fifo();
    //lsm6dsv16x_fifo_irq();
    //lsm6dsv16x_qvar_read_data_polling();
    //lsm6dsv16x_free_fall();
    //lsm6dsv16x_wakeup();
    //lsm6dsv16x_sixd();
    //lsm6dsv16x_single_double_tap();
    //lsm6dsv16x_fsm_glance();
    //lsm6dsv16x_fsm_fourd();
    lsm6dsv16x_mlc_gym();

    //lsm6dsv16bx_read_data_polling();
    //lsm6dsv16bx_activity();
    //lsm6dsv16bx_fifo();
    //lsm6dsv16bx_pedometer();
    //lsm6dsv16bx_self_test();
    //lsm6dsv16bx_tdm_conf();
    //lsm6dsv16bx_wake_up();
    //lsm6dsv16bx_sensor_fusion();

    //lsm6dsv32x_read_data_polling();
    //lsm6dsv32x_fifo();
    //lsm6dsv32x_self_test();
    lsm6dsv32x_sensor_fusion();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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

