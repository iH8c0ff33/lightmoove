#ifndef _I2C_PROTO_H
#define _I2C_PROTO_H

#include "driver/i2c.h"
#include "esp_system.h"

esp_err_t i2c_write_reg8(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg, uint8_t data);
esp_err_t i2c_write_reg16(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg, uint16_t data);
esp_err_t i2c_read_reg8(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg, uint8_t *data);
esp_err_t i2c_read_reg16(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg, uint16_t *data);
esp_err_t i2c_update_reg8(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg,
                          uint8_t update);
esp_err_t i2c_remove_reg8(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg,
                          uint8_t remove);

#endif
