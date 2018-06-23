#ifndef _VL53L0X_PLATFORM_H
#define _VL53L0X_PLATFORM_H

#include "esp_system.h"
#include "vl53l0x_def.h"

typedef struct {
  vl53l0x_dev_data_t data;

  uint8_t  i2c_addr, i2c_port;
  uint32_t i2c_timeout;
} vl53l0x_dev_t;

typedef vl53l0x_dev_t* vl53l0x_handle_t;

vl53l0x_err_t vl53l0x_write_n(vl53l0x_handle_t dev, uint8_t reg, uint8_t* data, uint32_t length);
vl53l0x_err_t vl53l0x_read_n(vl53l0x_handle_t dev, uint8_t reg, uint8_t* data, uint32_t length);
vl53l0x_err_t vl53l0x_write_8(vl53l0x_handle_t dev, uint8_t reg, uint8_t data);
vl53l0x_err_t vl53l0x_write_16(vl53l0x_handle_t dev, uint8_t reg, uint16_t data);
vl53l0x_err_t vl53l0x_write_32(vl53l0x_handle_t dev, uint8_t reg, uint32_t data);
vl53l0x_err_t vl53l0x_read_8(vl53l0x_handle_t dev, uint8_t reg, uint8_t* data);
vl53l0x_err_t vl53l0x_read_16(vl53l0x_handle_t dev, uint8_t reg, uint16_t* data);
vl53l0x_err_t vl53l0x_read_32(vl53l0x_handle_t dev, uint8_t reg, uint32_t* data);
vl53l0x_err_t vl53l0x_update_8(vl53l0x_handle_t dev, uint8_t reg, uint8_t and_data,
                               uint8_t or_data);

#endif
