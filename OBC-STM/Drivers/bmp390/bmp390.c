#include "bmp390.h"

static uint8_t bmp390_read_reg(bmp390_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static uint8_t bmp390_write_reg(bmp390_t *dev, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static void bmp390_read_calibration(bmp390_t *dev)
{
    uint8_t calib[21];
    bmp390_read_reg(dev, BMP390_REG_CALIB, calib, 21);

    uint16_t t1 = calib[1] << 8 | calib[0];
    uint16_t t2 = calib[3] << 8 | calib[2];
    int8_t  t3 = calib[4];

    int16_t p1 = (int16_t)(calib[6] << 8 | calib[5]);
    int16_t p2 = (int16_t)(calib[8] << 8 | calib[7]);
    int8_t  p3 = calib[9];
    int8_t  p4 = calib[10];
    uint16_t p5 = calib[12] << 8 | calib[11];
    uint16_t p6 = calib[14] << 8 | calib[13];
    int8_t  p7 = calib[15];
    int8_t  p8 = calib[16];
    int16_t p9 = (int16_t)(calib[18] << 8 | calib[17]);
    int8_t  p10 = calib[19];
    int8_t  p11 = calib[20];

    // Fixed-point → float conversion (no pow(), no double)
    dev->par_t1 = t1  / 256.0f;
    dev->par_t2 = t2  / 1073741824.0f;     // 2^30
    dev->par_t3 = t3  / 281474976710656.0f; // 2^48

    dev->par_p1 = (p1 - 16384) / 1048576.0f;      // 2^20
    dev->par_p2 = (p2 - 16384) / 536870912.0f;    // 2^29
    dev->par_p3 = p3 / 281474976710656.0f;
    dev->par_p4 = p4 / 281474976710656.0f;
    dev->par_p5 = p5 * 2.0f;                      // 2^-1
    dev->par_p6 = p6 / 64.0f;
    dev->par_p7 = p7 / 256.0f;
    dev->par_p8 = p8 / 32768.0f;
    dev->par_p9 = p9 / 281474976710656.0f;
    dev->par_p10 = p10 / 281474976710656.0f;
    dev->par_p11 = p11 / 3.6893488147419103e19f;  // 2^65
}

uint8_t bmp390_init(bmp390_t *dev, I2C_HandleTypeDef *hi2c)
{
    dev->hi2c = hi2c;
    dev->dev_addr = BMP390_I2C_ADDR_HIGH;   // Change to BMP390_I2C_ADDR_LOW if SDO grounded

    uint8_t id;
    if (bmp390_read_reg(dev, BMP390_REG_CHIP_ID, &id, 1) != HAL_OK || id != 0x60) {
        return 1;
    }

    if (bmp390_write_reg(dev, BMP390_REG_CMD, 0xB6) != HAL_OK) {  // Soft reset
        return 1;
    }
    HAL_Delay(2);

    // 8x pressure oversampling, 2x temp, IIR filter coeff 3, 25 Hz ODR
    if (bmp390_write_reg(dev, BMP390_REG_OSR,     0x1C) != HAL_OK) return 1;   // OSR_P=8x, OSR_T=2x
    if (bmp390_write_reg(dev, BMP390_REG_ODR,     0x04) != HAL_OK) return 1;   // 25 Hz
    if (bmp390_write_reg(dev, BMP390_REG_CONFIG,  0x03) != HAL_OK) return 1;   // IIR filter coeff = 3
    if (bmp390_write_reg(dev, BMP390_REG_PWR_CTRL,0x33) != HAL_OK) return 1;   // Enable pressure + temp, normal mode

    bmp390_read_calibration(dev);
    HAL_Delay(50);
    return 0;
}

uint8_t bmp390_read_temp_pressure(bmp390_t *dev, float *temperature, float *pressure)
{
    uint8_t raw[6];
    if (bmp390_read_reg(dev, BMP390_REG_DATA, raw, 6) != HAL_OK) return 1;

    uint32_t raw_press = (raw[2] << 16) | (raw[1] << 8) | raw[0];
    uint32_t raw_temp  = (raw[5] << 16) | (raw[4] << 8) | raw[3];

    float t = (float)raw_temp;
    float p = (float)raw_press;

    // Temperature compensation
    float dt = t - dev->par_t1;
    float dt2 = dt * dt;
    *temperature = dt * dev->par_t2 + dt2 * dev->par_t3;
    float t_lin = *temperature;  // Compensated temp

    // Pressure compensation (grouped to minimize overflow)
    float pd1 = dev->par_p6 * t_lin;
    float pd2 = dev->par_p7 * (t_lin * t_lin);
    float pd3 = dev->par_p8 * (t_lin * t_lin * t_lin);
    float po1 = dev->par_p5 + pd1 + pd2 + pd3;

    pd1 = dev->par_p2 * t_lin;
    pd2 = dev->par_p3 * (t_lin * t_lin);
    pd3 = dev->par_p4 * (t_lin * t_lin * t_lin);
    float po2 = p * (dev->par_p1 + pd1 + pd2 + pd3);

    pd1 = p * p;
    pd2 = dev->par_p9 + (dev->par_p10 * t_lin);
    pd3 = pd1 * pd2;
    float pd4 = pd3 + ((p * pd1) * dev->par_p11);

    *pressure = po1 + po2 + pd4;
    dev->t_lin = t_lin;

    return 0;
}
