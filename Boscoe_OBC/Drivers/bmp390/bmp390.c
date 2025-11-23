#include "bmp390.h"
#include <math.h>

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
    uint8_t calib[22];
    bmp390_read_reg(dev, BMP390_REG_CALIB, calib, 22);

    uint16_t t1 = calib[1] << 8 | calib[0];
    uint16_t t2 = calib[3] << 8 | calib[2];
    int8_t  t3 = calib[4];

    int16_t p1 = (int16_t)((calib[7] << 8) | calib[6]) - (1 << 14);
    int16_t p2 = (int16_t)((calib[9] << 8) | calib[8]) - (1 << 14);
    int8_t  p3 = calib[10];
    int8_t  p4 = calib[11];
    uint16_t p5 = calib[13] << 8 | calib[12];
    uint16_t p6 = calib[15] << 8 | calib[14];
    int8_t  p7 = calib[16];
    int8_t  p8 = calib[17];
    int16_t p9 = (int16_t)((calib[19] << 8) | calib[18]);
    int8_t  p10 = calib[20];
    int8_t  p11 = calib[21];

    dev->par_t1 = t1 / pow(2, -8.0);
    dev->par_t2 = t2 / pow(2, 30.0);
    dev->par_t3 = t3 / pow(2, 48.0);

    dev->par_p1 = (p1 - pow(2, 14)) / pow(2, 20.0);
    dev->par_p2 = (p2 - pow(2, 14)) / pow(2, 29.0);
    dev->par_p3 = p3 / pow(2, 48.0);
    dev->par_p4 = p4 / pow(2, 48.0);
    dev->par_p5 = p5 / pow(2, -1.0);
    dev->par_p6 = p6 / pow(2, 6.0);
    dev->par_p7 = p7 / pow(2, 8.0);
    dev->par_p8 = p8 / pow(2, 15.0);
    dev->par_p9 = p9 / pow(2, 48.0);
    dev->par_p10 = p10 / pow(2, 48.0);
    dev->par_p11 = p11 / pow(2, 65.0);
}

uint8_t bmp390_init(bmp390_t *dev, I2C_HandleTypeDef *hi2c)
{
    dev->hi2c = hi2c;
    dev->dev_addr = BMP390_I2C_ADDR_HIGH;   // change to LOW if your SDO is grounded

    uint8_t id;
    if (bmp390_read_reg(dev, BMP390_REG_CHIP_ID, &id, 1) != HAL_OK || id != 0x60) {
        return 1;   // failed
    }

    // Soft reset
    bmp390_write_reg(dev, BMP390_REG_CMD, 0xB6);
    HAL_Delay(2);

    // Set oversampling: 8x pressure, 2x temperature, normal mode
    bmp390_write_reg(dev, BMP390_REG_OSR,     0x0C);   // OSR_P = 8x, OSR_T = 2x
    bmp390_write_reg(dev, BMP390_REG_ODR,     0x04);   // 25 Hz output rate
    bmp390_write_reg(dev, BMP390_REG_PWR_CTRL, 0x33);   // Enable pressure + temp, normal mode

    bmp390_read_calibration(dev);
    HAL_Delay(40);   // wait for first measurement
    return 0;
}

uint8_t bmp390_read_temp_pressure(bmp390_t *dev, float *temperature, float *pressure)
{
    uint8_t raw[6];
    if (bmp390_read_reg(dev, BMP390_REG_DATA, raw, 6) != HAL_OK) return 1;

    uint32_t raw_press = (raw[2] << 16) | (raw[1] << 8) | raw[0];
    uint32_t raw_temp  = (raw[5] << 16) | (raw[4] << 8) | raw[3];

    double t = (double)raw_temp;
    double p = (double)raw_press;

    double pd = dev->par_p1 * t + dev->par_p2 * dev->t_lin + dev->par_p3 * t * t +
                dev->par_p4 * dev->t_lin * dev->t_lin + dev->par_p5 * t * dev->t_lin +
                dev->par_p6 * t * t * t + dev->par_p7 * dev->t_lin * dev->t_lin * dev->t_lin +
                dev->par_p8 * p + dev->par_p9 + dev->par_p10 * t * p + dev->par_p11 * dev->t_lin * p;

    dev->t_lin = dev->par_t2 * t + dev->par_t3 * t * t + dev->par_t1;

    *temperature = (float)(dev->par_t1 + dev->par_t2 * dev->t_lin + dev->par_t3 * dev->t_lin * dev->t_lin);
    *pressure    = (float)pd;

    return 0;
}