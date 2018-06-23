#include "i2c_util.h"

esp_err_t i2c_write_reg(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg, uint8_t *data,
                        uint8_t length) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);

  i2c_master_write(cmd, data, length, true);
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(port, cmd, timeout);
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t i2c_write_reg8(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);  // send the start bit
  // initiate a write to addr and expect ACK
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);  // send the register address

  i2c_master_write_byte(cmd, data, true);                    // send the actual data
  i2c_master_stop(cmd);                                      // send the stop bit
  esp_err_t err = i2c_master_cmd_begin(port, cmd, timeout);  // perform the command
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t i2c_write_reg16(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg,
                          uint16_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);

  i2c_master_write_byte(cmd, (data & 0xff00) >> 8, true);
  i2c_master_write_byte(cmd, (data & 0xff), true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(port, cmd, timeout);
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t i2c_write_reg32(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg,
                          uint32_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);

  i2c_master_write_byte(cmd, (data & 0xff000000) >> 24, true);
  i2c_master_write_byte(cmd, (data & 0xff0000) >> 16, true);
  i2c_master_write_byte(cmd, (data & 0xff00) >> 8, true);
  i2c_master_write_byte(cmd, (data & 0xff), true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(port, cmd, timeout);
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t i2c_read_reg(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg, uint8_t *data,
                       uint8_t length) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, data + (length - 2), I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(port, cmd, timeout);
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t i2c_read_reg8(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg, uint8_t *data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);  // send the register address to read from
  // read the chosen register
  i2c_master_start(cmd);  // we need to send another start bit because we're issuing a new command
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(port, cmd, timeout);
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t i2c_read_reg16(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg,
                         uint16_t *data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
  uint8_t msb, lsb;
  i2c_master_read_byte(cmd, &msb, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &lsb, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(port, cmd, timeout);
  i2c_cmd_link_delete(cmd);

  *data = (msb << 8) | lsb;
  return err;
}

esp_err_t i2c_update_reg8(uint8_t port, uint32_t timeout, uint8_t addr, uint8_t reg,
                          uint8_t and_data, uint8_t or_data) {
  uint8_t   data;
  esp_err_t err = i2c_read_reg8(port, timeout, addr, reg, &data);
  if (err != 0) return err;

  return i2c_write_reg8(port, timeout, addr, reg, (data & and_data) | or_data);
}
