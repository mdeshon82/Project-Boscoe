#ifndef BMP390_H
#define BMP390_H

#include "stm32f4xx_hal.h"

#define BMP390_I2C_ADDR_LOW   0x76 << 1
#define BMP390_I2C_ADDR_HIGH  0x77 << 1   // yours is probably 0x77

// Register addresses
#define BMP390_REG_CHIP_ID     0x00
#define BMP390_REG_STATUS      0x03
#define BMP390_REG_DATA        0x04   // Pressure starts here
#define BMP390_REG_CMD         0x7E
#define BMP390_REG_OSR         0x1C
#define BMP390_REG_ODR         0x1D
#define BMP390_REG_PWR_CTRL    0x1B
#define BMP390_REG_CALIB      0x31   // Calibration data starts
#define BMP390_REG_CONFIG 0x1F 

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
    
    // Calibration coefficients (will be read once at init)
    double par_t1, par_t2, par_t3;
    double par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
    double t_lin;
} bmp390_t;

// Public functions
uint8_t bmp390_init(bmp390_t *dev, I2C_HandleTypeDef *hi2c);
uint8_t bmp390_read_temp_pressure(bmp390_t *dev, float *temperature, float *pressure);

#endif