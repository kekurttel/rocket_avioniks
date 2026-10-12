/* BMP280 access + Bosch integer compensation. See bmp280.h. */
#include "bmp280.h"

static int32_t bmp280_compensate_temp(bmp280_t *d, int32_t adc_T)
{
  int32_t var1 = (((adc_T >> 3) - ((int32_t)d->dig_T1 << 1)) * (int32_t)d->dig_T2) >> 11;
  int32_t var2 = (((((adc_T >> 4) - (int32_t)d->dig_T1) * ((adc_T >> 4) - (int32_t)d->dig_T1)) >> 12)
                  * (int32_t)d->dig_T3) >> 14;
  d->t_fine = var1 + var2;
  return (d->t_fine * 5 + 128) >> 8; /* 0.01 degC */
}

/* Bosch 64-bit compensation. Datasheet returns Q24.8; we fold the /256
 * here (with rounding) so callers get plain Pa. */
static uint32_t bmp280_compensate_press(bmp280_t *d, int32_t adc_P)
{
  int64_t var1 = ((int64_t)d->t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)d->dig_P6;
  var2 = var2 + ((var1 * (int64_t)d->dig_P5) << 17);
  var2 = var2 + (((int64_t)d->dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)d->dig_P3) >> 8) + ((var1 * (int64_t)d->dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1) * (int64_t)d->dig_P1) >> 33;
  if (var1 == 0)
  {
    return 0; /* Avoid division by zero; caller treats short read as invalid. */
  }
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)d->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)d->dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)d->dig_P7) << 4);
  return (uint32_t)((p + 128) >> 8); /* Q24.8 -> Pa */
}

HAL_StatusTypeDef bmp280_init(bmp280_t *d, I2C_HandleTypeDef *bus, uint8_t addr7)
{
  d->bus = bus;
  d->addr8 = (uint8_t)(addr7 << 1);
  d->present = false;
  d->t_fine = 0;
  d->id = 0;

  HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(bus, d->addr8, 1, BMP280_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

  uint8_t id = 0;
  st = HAL_I2C_Mem_Read(bus, d->addr8, BMP280_REG_ID,
                        I2C_MEMADD_SIZE_8BIT, &id, 1, BMP280_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }
  d->id = id;
  if (id != BMP280_CHIP_ID)
  {
    return HAL_ERROR;
  }

  uint8_t cal[24];
  st = HAL_I2C_Mem_Read(bus, d->addr8, BMP280_REG_CALIB,
                        I2C_MEMADD_SIZE_8BIT, cal, sizeof(cal), BMP280_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

  d->dig_T1 = (uint16_t)(cal[1] << 8 | cal[0]);
  d->dig_T2 = (int16_t)(cal[3] << 8 | cal[2]);
  d->dig_T3 = (int16_t)(cal[5] << 8 | cal[4]);
  d->dig_P1 = (uint16_t)(cal[7] << 8 | cal[6]);
  d->dig_P2 = (int16_t)(cal[9] << 8 | cal[8]);
  d->dig_P3 = (int16_t)(cal[11] << 8 | cal[10]);
  d->dig_P4 = (int16_t)(cal[13] << 8 | cal[12]);
  d->dig_P5 = (int16_t)(cal[15] << 8 | cal[14]);
  d->dig_P6 = (int16_t)(cal[17] << 8 | cal[16]);
  d->dig_P7 = (int16_t)(cal[19] << 8 | cal[18]);
  d->dig_P8 = (int16_t)(cal[21] << 8 | cal[20]);
  d->dig_P9 = (int16_t)(cal[23] << 8 | cal[22]);

  /* 1. Configure IIR filter & standby in SLEEP mode (device defaults to sleep at POR).
   * Writes to 0xF5 in normal mode may be ignored according to Bosch datasheet section 4.3.5.
   * filter = 001 (coeff 2), t_sb = 000 (0.5 ms), spi3w_en = 0 -> 0x04 */
  uint8_t cfg = BMP280_CONFIG_DEMO;
  st = HAL_I2C_Mem_Write(bus, d->addr8, BMP280_REG_CONFIG,
                         I2C_MEMADD_SIZE_8BIT, &cfg, 1, BMP280_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

  /* 2. Switch to NORMAL mode with x1 temperature, x4 pressure oversampling -> 0x2F */
  uint8_t ctrl = BMP280_CTRL_MEAS_DEMO;
  st = HAL_I2C_Mem_Write(bus, d->addr8, BMP280_REG_CTRL,
                         I2C_MEMADD_SIZE_8BIT, &ctrl, 1, BMP280_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

  /* Diagnostic read-back of registers to verify configuration succeeded */
  (void)HAL_I2C_Mem_Read(bus, d->addr8, BMP280_REG_CONFIG,
                         I2C_MEMADD_SIZE_8BIT, &d->config_reg, 1, BMP280_TIMEOUT_MS);
  (void)HAL_I2C_Mem_Read(bus, d->addr8, BMP280_REG_CTRL,
                         I2C_MEMADD_SIZE_8BIT, &d->ctrl_reg, 1, BMP280_TIMEOUT_MS);

  d->present = true;
  return HAL_OK;
}

HAL_StatusTypeDef bmp280_read(bmp280_t *d, int32_t *temp_c01, uint32_t *press_pa)
{
  if (!d->present)
  {
    return HAL_ERROR;
  }

  uint8_t raw[6];
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(d->bus, d->addr8, BMP280_REG_DATA,
                                          I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw),
                                          BMP280_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

  int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);
  int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);

  *temp_c01 = bmp280_compensate_temp(d, adc_T);
  *press_pa = bmp280_compensate_press(d, adc_P);
  return HAL_OK;
}
