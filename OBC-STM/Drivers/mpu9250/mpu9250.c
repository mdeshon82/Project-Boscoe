#include "mpu9250.h"
#include <string.h>

static uint8_t read_reg(mpu9250_t *dev, uint8_t i2c_addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static uint8_t write_reg(mpu9250_t *dev, uint8_t i2c_addr, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

uint8_t mpu9250_init(mpu9250_t *dev, I2C_HandleTypeDef *hi2c)
{
    dev->hi2c = hi2c;

    uint8_t who;
    if (read_reg(dev, MPU9250_ADDR, 0x75, &who, 1) != HAL_OK || who != 0x71) {
        return 1;   // No MPU9250
    }

    if (write_reg(dev, MPU9250_ADDR, 0x6B, 0x00) != HAL_OK) return 1;    // Wake up
    HAL_Delay(10);

    if (write_reg(dev, MPU9250_ADDR, 0x1B, 0x10) != HAL_OK) return 1;    // Gyro ±1000 °/s (FS_SEL=2)
    if (write_reg(dev, MPU9250_ADDR, 0x1C, 0x10) != HAL_OK) return 1;    // Accel ±8g (FS_SEL=2)
    if (write_reg(dev, MPU9250_ADDR, 0x37, 0x02) != HAL_OK) return 1;    // I2C bypass → direct AK8963 access

    if (read_reg(dev, AK8963_ADDR, 0x00, &who, 1) != HAL_OK || who != 0x48) {
        return 2;   // No magnetometer
    }

    if (write_reg(dev, AK8963_ADDR, 0x0A, 0x16) != HAL_OK) return 2;     // 16-bit, continuous mode 2 (100 Hz)
    HAL_Delay(10);

    return 0;
}

void mpu9250_read_all(mpu9250_t *dev)
{
    uint8_t data[22];

    // Accel + Gyro
    if (read_reg(dev, MPU9250_ADDR, 0x3B, data, 14) != HAL_OK) {
        // Handle error, e.g., set values to 0
        memset(dev->accel, 0, sizeof(dev->accel));
        memset(dev->gyro, 0, sizeof(dev->gyro));
        memset(dev->mag, 0, sizeof(dev->mag));
        return;
    }

    dev->accel[0] = (int16_t)(data[0]  << 8 | data[1])  / 4096.0f;   // ±8g
    dev->accel[1] = (int16_t)(data[2]  << 8 | data[3])  / 4096.0f;
    dev->accel[2] = (int16_t)(data[4]  << 8 | data[5])  / 4096.0f;

    dev->gyro[0]  = (int16_t)(data[8]  << 8 | data[9])  / 32.8f;     // ±1000 °/s
    dev->gyro[1]  = (int16_t)(data[10] << 8 | data[11]) / 32.8f;
    dev->gyro[2]  = (int16_t)(data[12] << 8 | data[13]) / 32.8f;

    // Magnetometer (wait for DRDY with timeout)
    uint8_t st1;
    uint32_t timeout = 100;  // ~100ms max wait
    do {
        if (read_reg(dev, AK8963_ADDR, 0x02, &st1, 1) != HAL_OK) {
            memset(dev->mag, 0, sizeof(dev->mag));
            return;
        }
        if (timeout-- == 0) {
            memset(dev->mag, 0, sizeof(dev->mag));
            return;  // Timeout
        }
        HAL_Delay(1);
    } while (!(st1 & 0x01));

    if (read_reg(dev, AK8963_ADDR, 0x03, data, 7) != HAL_OK) {
        memset(dev->mag, 0, sizeof(dev->mag));
        return;
    }

    // Check for magnetic overflow (ST2 bit 3)
    if (data[6] & 0x08) {
        // Invalid data, set to 0 or retry if desired
        memset(dev->mag, 0, sizeof(dev->mag));
        return;
    }

    int16_t mx = (int16_t)(data[1] << 8 | data[0]);
    int16_t my = (int16_t)(data[3] << 8 | data[2]);
    int16_t mz = (int16_t)(data[5] << 8 | data[4]);

    dev->mag[0] = mx * 0.15f;   // µT
    dev->mag[1] = my * 0.15f;
    dev->mag[2] = mz * 0.15f;
}