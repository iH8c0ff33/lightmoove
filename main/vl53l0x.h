#ifndef _VL53L0X_H
#define _VL53L0X_H

#include "esp_system.h"
#include "i2c_proto.h"

#define DEFAULT_I2C_ADDR 0x29

#ifdef __cplusplus
extern "C" {
#endif

typedef struct vl53l0x *vl53l0x_handle_t;

typedef enum {
  SYSTEM_SEQUENCE_CONFIG                 = 0x01,
  MSRC_CONFIG_CONTROL                    = 0x60,
  FINAL_RANGE_CONFIG_MIN_COUNT_RTN_LIMIT = 0x44,
} vl53l0x_reg_addr_t;

typedef struct {
  const char addr;
  const char port;
  const int  timeout;
} vl53l0x_config_t;

vl53l0x_handle_t vl53l0x_create(vl53l0x_config_t *config);

esp_err_t vl53l0x_init(vl53l0x_handle_t vl53l0x);

#ifdef __cplusplus
}
#endif

#endif
