#ifndef MPU9250_H
#define MPU9250_H

#include "stm32f4xx_hal.h"

#define MPU9250_ADDR         0x68 << 1
#define MPU9250_WHO_AM_I_VAL 0x71
#define AK8963_ADDR          0x0C << 1
#define AK8963_WHO_AM_I_VAL  0x48

typedef struct {
    I2C_HandleTypeDef *hi2c;
    float accel[3], gyro[3], mag[3];
} mpu9250_t;

uint8_t mpu9250_init(mpu9250_t *dev, I2C_HandleTypeDef *hi2c);
void mpu9250_read_all(mpu9250_t *dev);

#endif