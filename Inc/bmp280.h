/* BMP280 access + Bosch integer compensation.
 *
 * Self-contained I2C driver for the bench-top avionics demo.
 * Register map and compensation formulas follow the Bosch BMP280
 * datasheet (integer versions, temperature first then pressure).
 * All I2C timeouts are finite; no HAL_MAX_DELAY, no printf.
 */
#ifndef BMP280_H
#define BMP280_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define BMP280_CHIP_ID   (0x58)
#define BMP280_REG_ID    (0xD0)
#define BMP280_REG_CALIB (0x88)
#define BMP280_REG_CTRL  (0xF4)
#define BMP280_REG_CONFIG (0xF5)
#define BMP280_REG_DATA  (0xF7)

/* Demo ctrl_meas: osrs_t x1 [7:5]=001, osrs_p x4 [4:2]=011, normal [1:0]=11.
 * 0b001_011_11 = 0x2F. x4 pressure oversampling lowers noise vs x1
 * (datasheet) while staying fast enough for the 5 Hz loop; no x16. */
#define BMP280_CTRL_MEAS_DEMO (0x2F)

/* Config register: t_sb 0.5 ms [7:5]=000, filter coeff 2 [4:2]=001, spi3w_en=0.
 * 0b000_001_00 = 0x04. Written in sleep mode prior to normal mode to engage
 * the internal IIR filter with reduced latency for responsive bench testing. */
#define BMP280_CONFIG_DEMO    (0x04)

#define BMP280_TIMEOUT_MS (100U)

typedef struct
{
  I2C_HandleTypeDef *bus;
  uint8_t addr8;      /* 7-bit address shifted left (HAL format). */
  uint8_t id;         /* Last chip-ID read (0xD0). */
  uint8_t config_reg; /* Read-back diagnostic of CONFIG (0xF5) register. */
  uint8_t ctrl_reg;   /* Read-back diagnostic of CTRL_MEAS (0xF4) register. */
  bool present;       /* True once init fully succeeded. */
  int32_t t_fine;     /* Bosch intermediate, per instance (two sensors!). */

  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
} bmp280_t;

/* Probe device, verify chip ID, read calibration, start normal mode.
 * Sets d->present=true only when every step succeeded. */
HAL_StatusTypeDef bmp280_init(bmp280_t *d, I2C_HandleTypeDef *bus, uint8_t addr7);

/* Single pressure+temperature sample.
 * temp_c01: temperature in 0.01 degC. press_pa: pressure in Pa. */
HAL_StatusTypeDef bmp280_read(bmp280_t *d, int32_t *temp_c01, uint32_t *press_pa);

#endif /* BMP280_H */
