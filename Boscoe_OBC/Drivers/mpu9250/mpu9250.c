#include "mpu9250.h"

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
    
    // Check MPU-9250
    read_reg(dev, MPU9250_ADDR, 0x75, &who, 1);
    if (who != MPU9250_WHO_AM_I_VAL) return 1;

    // Wake up
    write_reg(dev, MPU9250_ADDR, 0x6B, 0x00);
    HAL_Delay(10);

    // ±8g full scale, ±1000 °/s
    write_reg(dev, MPU9250_ADDR, 0x1B, 0x08);   // Gyro config
    write_reg(dev, MPU9250_ADDR, 0x1C, 0x08);   // Accel config

    // Bypass mode to access magnetometer
    write_reg(dev, MPU9250_ADDR, 0x37, 0x02);

    // Check AK8963
    read_reg(dev, AK8963_ADDR, 0x00, &who, 1);
    if (who != AK8963_WHO_AM_I_VAL) return 2;

    // 16-bit, continuous mode 2 (100 Hz)
    write_reg(dev, AK8963_ADDR, 0x0A, 0x16);
    HAL_Delay(10);

    return 0;
}

void mpu9250_read_all(mpu9250_t *dev)
{
    uint8_t data[22];

    // Read accel + gyro
    read_reg(dev, MPU9250_ADDR, 0x3B, data, 14);

    int16_t ax = (int16_t)(data[0]  << 8 | data[1]);
    int16_t ay = (int16_t)(data[2]  << 8 | data[3]);
    int16_t az = (int16_t)(data[4]  << 8 | data[5]);
    int16_t gx = (int16_t)(data[8]  << 8 | data[9]);
    int16_t gy = (int16_t)(data[10] << 8 | data[11]);
    int16_t gz = (int16_t)(data[12] << 8 | data[13]);

    dev->accel[0] = ax / 4096.0f;   // ±8g
    dev->accel[1] = ay / 4096.0f;
    dev->accel[2] = az / 4096.0f;
    dev->gyro[0]  = gx / 32.8f;     // ±1000 °/s
    dev->gyro[1]  = gy / 32.8f;
    dev->gyro[2]  = gz / 32.8f;

    // Read magnetometer
    uint8_t st1;
    do {
        read_reg(dev, AK8963_ADDR, 0x02, &st1, 1);
    } while (!(st1 & 0x01));

    read_reg(dev, AK8963_ADDR, 0x03, data, 7);

    int16_t mx = (int16_t)(data[1] << 8 | data[0]);
    int16_t my = (int16_t)(data[3] << 8 | data[2]);
    int16_t mz = (int16_t)(data[5] << 8 | data[4]);

    dev->mag[0] = mx * 0.15f;   // µT (factory sensitivity 0.15 µT/LSB)
    dev->mag[1] = my * 0.15f;
    dev->mag[2] = mz * 0.15f;
}