/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file : main.c
  * @brief : Boscoe OBC Main Program Body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Matt DeShon.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>      // for altitude calculation

/* YOUR drivers */
#include "bmp390.h"
#include "mpu9250.h"
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
/* Global sensor objects */
bmp390_t     bmp;
mpu9250_t    mpu;

uint32_t last_telemetry = 0;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Retarget printf to the virtual COM port */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */
/**
  * @brief The application entry point.
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
MX_I2C1_Init();
MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n\r\n=== CubeSat OBC Lab 1 – YOUR OWN DRIVERS ===\r\n");
  printf("SystemCoreClock = %lu Hz\r\n", SystemCoreClock);

  /* ---------- Initialise YOUR BMP390 driver ---------- */
  if (bmp390_init(&bmp, &hi2c1) == 0)
      printf("YOUR BMP390 driver: INITIALIZED OK\r\n");
  else
      printf("YOUR BMP390 driver: FAILED!\r\n");

  /* ---------- Initialise YOUR MPU-9250 driver ---------- */
  if (mpu9250_init(&mpu, &hi2c1) == 0)
      printf("YOUR MPU-9250 driver: INITIALIZED OK (9-DoF)\r\n");
  else
      printf("YOUR MPU-9250 driver: FAILED!\r\n");

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // LED on at boot
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
  {
      if (HAL_GetTick() - last_telemetry >= 2000)  // Every 2 seconds
      {
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);   // Heartbeat

          /* ---- Read from YOUR BMP390 driver ---- */
          float temperature, pressure;
          if (bmp390_read_temp_pressure(&bmp, &temperature, &pressure) == 0)
          {
              /* Simple sea-level pressure to altitude (you can refine this later) */
              float altitude = 44330.0f * (1.0f - powf(pressure / 101325.0f, 0.190294957f));  // Fixed sea-level pressure in Pa (1013.25 hPa = 101325 Pa)

              printf("\r\n--- Telemetry (Your Drivers) ---\r\n");
              printf("BMP390 │ Temp: %.2f °C │ Press: %.2f hPa │ Alt: %.1f m\r\n",
                     temperature, pressure / 100.0f, altitude);
          }

          /* ---- Read from YOUR MPU-9250 driver ---- */
          mpu9250_read_all(&mpu);

          printf("MPU9250 Accel │ X: %.3f  Y: %.3f  Z: %.3f m/s²\r\n",
                 mpu.accel[0], mpu.accel[1], mpu.accel[2]);
          printf("MPU9250 Gyro  │ X: %.2f  Y: %.2f  Z: %.2f °/s\r\n",
                 mpu.gyro[0],  mpu.gyro[1],  mpu.gyro[2]);
          printf("MPU9250 Mag   │ X: %.0f  Y: %.0f  Z: %.0f µT\r\n",
                 mpu.mag[0],   mpu.mag[1],   mpu.mag[2]);

          last_telemetry = HAL_GetTick();
      }

      __WFI();   // Sleep until next tick – saves power
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
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
RCC_OscInitStruct.HSIState = RCC_HSI_ON;
RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
RCC_OscInitStruct.PLL.PLLM = 8;
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
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
Error_Handler();
  }
}
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
/**
  * @brief This function is executed in case of error occurrence.
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
  * @brief Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
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