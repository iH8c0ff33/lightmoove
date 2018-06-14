/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/i2c.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l0x.h"
#include <stdio.h>

#define SCL_IO 25
#define SDA_IO 26
#define I2C_PORT I2C_NUM_0
#define I2C_FREQ 100000
#define I2C_RX_BUF 0
#define I2C_TX_BUF 0
#define I2C_CMD_TIMEOUT 10 / portTICK_PERIOD_MS
#define I2C_ADDR 0x29

enum reg_addr {
  SYSTEM_SEQUENCE_CONFIG = 0x01,
  MSRC_CONFIG_CONTROL = 0x60,
  FINAL_RANGE_CONFIG_MIN_COUNT_RTN_LIMIT = 0x44,
};

void i2c_init() {
  int i2c_port = I2C_PORT;
  i2c_config_t cfg;
  cfg.mode = I2C_MODE_MASTER;
  cfg.sda_io_num = SDA_IO;
  cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.scl_io_num = SCL_IO;
  cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.master.clk_speed = I2C_FREQ;
  i2c_param_config(i2c_port, &cfg);
  i2c_driver_install(i2c_port, cfg.mode, I2C_RX_BUF, I2C_TX_BUF, 0);
}

esp_err_t i2c_write_reg8(uint8_t port, uint8_t addr, uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd); // send the start bit
  // initiate a write to addr and expect ACK
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true); // send the register address

  i2c_master_write_byte(cmd, data, true);                           // send the actual data
  i2c_master_stop(cmd);                                             // send the stop bit
  esp_err_t err = i2c_master_cmd_begin(port, cmd, I2C_CMD_TIMEOUT); // perform the command
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t i2c_write_reg16(uint8_t port, uint8_t addr, uint8_t reg, uint16_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);

  i2c_master_write_byte(cmd, (data & 0xff00) >> 8, true);
  i2c_master_write_byte(cmd, (data & 0xff), true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(port, cmd, I2C_CMD_TIMEOUT);
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t i2c_read_reg8(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true); // send the register address to read from
  // read the chosen register
  i2c_master_start(cmd); // we need to send another start bit because we're issuing a new command
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(port, cmd, I2C_CMD_TIMEOUT);
  i2c_cmd_link_delete(cmd);

  return err;
}

esp_err_t i2c_read_reg16(uint8_t port, uint8_t addr, uint8_t reg, uint16_t *data) {
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
  esp_err_t err = i2c_master_cmd_begin(port, cmd, I2C_CMD_TIMEOUT);
  i2c_cmd_link_delete(cmd);

  *data = (msb << 8) | lsb;
  return err;
}

esp_err_t i2c_update_reg8(uint8_t port, uint8_t addr, uint8_t reg, uint8_t update) {
  uint8_t data;
  esp_err_t err = i2c_read_reg8(port, addr, reg, &data);
  if (err != 0)
    return err;

  return i2c_write_reg8(port, addr, reg, data | update);
}

esp_err_t set_signal_rate_limit(float limit) {
  if (limit < 0 || limit > 511.99)
    return ESP_ERR_INVALID_ARG;

  return i2c_write_reg16(I2C_PORT, I2C_ADDR, FINAL_RANGE_CONFIG_MIN_COUNT_RTN_LIMIT,
                         limit * (1 << 7));
}

esp_err_t start_proc() {
  esp_err_t err = 0;
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x80, 0x01);
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0xff, 0x01);
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x00, 0x00);
  return err;
}

esp_err_t stop_proc() {
  esp_err_t err = 0;
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x00, 0x01);
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0xff, 0x00);
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x80, 0x00);
  return err;
}

void app_main() {
  printf("Hello world!\n");

  i2c_init();

  esp_err_t err = 0;
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x88, 0x00); // set i2c standard mode
  err |= start_proc();
  unsigned char stop_var;
  err |= i2c_read_reg8(I2C_PORT, I2C_ADDR, 0x91, &stop_var);
  err |= stop_proc();

  uint8_t cfg;
  err |= i2c_read_reg8(I2C_PORT, I2C_ADDR, MSRC_CONFIG_CONTROL, &cfg);
  printf("cfg: 0x%.2x\n", cfg);
  err |= i2c_update_reg8(I2C_PORT, I2C_ADDR, MSRC_CONFIG_CONTROL, 0x12);
  err |= i2c_read_reg8(I2C_PORT, I2C_ADDR, MSRC_CONFIG_CONTROL, &cfg);
  printf("cfg: 0x%.2x\n", cfg);

  err |= set_signal_rate_limit(0.25);
  uint16_t rate;
  err |= i2c_read_reg16(I2C_PORT, I2C_ADDR, FINAL_RANGE_CONFIG_MIN_COUNT_RTN_LIMIT, &rate);
  printf("rate: 0x%.4x -> %.2f\n", rate, (float)rate / (1 << 7));

  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, SYSTEM_SEQUENCE_CONFIG, 0xff);

  err |= start_proc();
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0xff, 0x06);
  err |= i2c_update_reg8(I2C_PORT, I2C_ADDR, 0x83, 0x04);
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0xff, 0x07);
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x81, 0x01);

  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x80, 0x01);

  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x94, 0x6b);
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x83, 0x00);

  uint8_t data = 0;
  while (data == 0)
    i2c_read_reg8(I2C_PORT, I2C_ADDR, 0x83, &data);
  printf("out of loop 0x83 is 0x%.2x\n", data);

  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x83, 0x01);

  uint8_t tmp;
  err |= i2c_read_reg8(I2C_PORT, I2C_ADDR, 0x92, &tmp);
  uint8_t count = tmp & 0x7f;
  bool is_aperture = (tmp & 0x80) >> 7;
  printf("count is %u and type is%s aperture\n", count, is_aperture ? "" : " not");

  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x81, 0x00);
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0xff, 0x06);
  err |= i2c_read_reg8(I2C_PORT, I2C_ADDR, 0x83, &tmp);
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0x83, tmp & ~0x04);
  err |= i2c_write_reg8(I2C_PORT, I2C_ADDR, 0xff, 0x01);
  err |= stop_proc();

  if (err == 0)
    printf("no err, stop var is %.2x\n", stop_var);
  else
    printf("error occurred: %.2x\n", err);
}
