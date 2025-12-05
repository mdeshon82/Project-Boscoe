/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @by 	 Deshon Aerospace Engineering
  * @brief   Boscoe OBC – BMP390 + MPU9250 + Watchdog + Fault Monitor
  * @note    FRAM logging is DISABLED (chip not installed) but placeholders kept.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "iwdg.h"

#include "bmp390.h"
#include "mpu9250.h"

/* USER CODE BEGIN PV */

/* Sensor objects */
bmp390_t bmp;
mpu9250_t mpu;

/* Telemetry timing */
static uint32_t last_telemetry = 0;
#define TELEMETRY_PERIOD_MS   2000U

/* Watchdog */


/* Fault counters */
typedef struct {
    uint8_t bmp_fail_count;
    uint8_t mpu_fail_count;
    uint32_t last_error_tick;
} system_faults_t;

static system_faults_t faults = {0};

/* CRC utilities (for future FRAM use) */
static uint32_t crc32_table[256];
static bool crc_ready = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void init_crc32_table(void);
static uint32_t crc32_compute(const uint8_t *data, size_t len);
// static void MX_IWDG_Init(void); NOT NEEDED FUTURE DELETE
static void FeedWatchdog(void);
static void verify_and_init_sensors(void);

/* Retarget printf to UART2 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* =========================================================================== */
/*                                  MAIN                                       */
/* =========================================================================== */

int main(void)
{
    /* HAL init */
    HAL_Init();
    SystemClock_Config();

    /* Init peripherals */
    MX_GPIO_Init();
    MX_I2C2_Init();     // IMPORTANT: using hi2c2
    MX_USART2_UART_Init();

    /* Init CRC32 table */
    init_crc32_table();

    /* Init Watchdog */
    MX_IWDG_Init();

    printf("\r\n==== BoscoSat OBC (Telemetry + Watchdog + Sensor Health) ====\r\n");
    printf("SystemCoreClock = %lu Hz\r\n", SystemCoreClock);

    /* Sensor initialization and verification */
    verify_and_init_sensors();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // LED on at boot

    /* Main loop */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (now - last_telemetry >= TELEMETRY_PERIOD_MS)
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // heartbeat LED

            float temperature = 0.0f, pressure = 0.0f;

            /* ---- BMP390 Read ---- */
            bool bmp_ok = (bmp390_read_temp_pressure(&bmp, &temperature, &pressure) == 0);

            if (!bmp_ok) {
                faults.bmp_fail_count++;
                faults.last_error_tick = now;
                printf("BMP390 READ FAILED (%u)\r\n", faults.bmp_fail_count);
            }

            /* ---- Compute altitude regardless ---- */
            float altitude = 44330.f * (1.0f - powf(pressure / 101325.f, 0.190294957f));

            /* ---- MPU9250 Read ---- (void return) */
            uint8_t last_accel[3];
            memcpy(last_accel, mpu.accel, sizeof(last_accel));
            mpu9250_read_all(&mpu);

            /* determine if we got new data or it failed */
            bool mpu_ok = true;
            if (mpu.accel[0] == 0 && mpu.accel[1] == 0 && mpu.accel[2] == 0 &&
                last_accel[0] == 0 && last_accel[1] == 0 && last_accel[2] == 0)
            {
                /* If both previous and current are zeros → likely sensor error */
                faults.mpu_fail_count++;
                faults.last_error_tick = now;
                mpu_ok = false;
                printf("MPU9250 READ FAILED (%u)\r\n", faults.mpu_fail_count);
            }

            /* ---- Print Telemetry ---- */
            printf("\r\n--- Telemetry ---\r\n");

            if (bmp_ok)
                printf("BMP390 Temp: %.2f C  Pressure: %.2f hPa  Alt: %.1f m\r\n",
                       temperature, pressure / 100.0f, altitude);
            else
                printf("BMP390: ERROR\r\n");

            if (mpu_ok)
            {
                printf("Accel: %.3f  %.3f  %.3f m/s²\r\n",
                       mpu.accel[0], mpu.accel[1], mpu.accel[2]);
                printf("Gyro : %.2f  %.2f  %.2f °/s\r\n",
                       mpu.gyro[0], mpu.gyro[1], mpu.gyro[2]);
                printf("Mag  : %.2f  %.2f  %.2f uT\r\n",
                       mpu.mag[0], mpu.mag[1], mpu.mag[2]);
            }
            else
                printf("MPU9250: ERROR\r\n");

            last_telemetry = now;
        }

        /* Feed watchdog */
        FeedWatchdog();

        __WFI();  // low-power wait
    }
}

/* =========================================================================== */
/*                              SUPPORT FUNCTIONS                               */
/* =========================================================================== */

static void verify_and_init_sensors(void)
{
    /* BMP390 Init */
    if (bmp390_init(&bmp, &hi2c2) == 0)
        printf("BMP390: init OK\r\n");
    else {
        printf("BMP390: init FAILED\r\n");
        faults.bmp_fail_count++;
    }

    /* MPU9250 Init */
    uint8_t mpu_init_status = mpu9250_init(&mpu, &hi2c2);
    if (mpu_init_status == 0)
    {
        printf("MPU9250: init OK\r\n");
    }
    else
    {
        printf("MPU9250: init FAILED (%u)\r\n", mpu_init_status);
        faults.mpu_fail_count++;
    }
}

/* Watchdog: ~4 second timeout */
// static void MX_IWDG_Init(void)
//    hiwdg.Instance = IWDG;
//  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
//  hiwdg.Init.Reload = 4095;   // max timeout
//    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
//        Error_Handler();
//}

static void FeedWatchdog(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

/* CRC32 table for future FRAM usage */
static void init_crc32_table(void)
{
    if (crc_ready) return;

    uint32_t poly = 0xEDB88320UL;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc >> 1) ^ ((crc & 1) ? poly : 0);
        crc32_table[i] = crc;
    }
    crc_ready = true;
}

static uint32_t crc32_compute(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < len; i++)
        crc = (crc >> 8) ^ crc32_table[(crc ^ data[i]) & 0xFF];
    return crc ^ 0xFFFFFFFFUL;
}

/* Default handlers */

void SystemClock_Config(void)
{
    /* This is YOUR CubeMX code — keep it unchanged.
       I am not modifying your clock tree. */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 100;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(200);
    }
}
