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
  i2c_master_write_byte(cmd, reg, true);                            // send the register address
  i2c_master_write_byte(cmd, data, true);                           // send the actual data
  i2c_master_stop(cmd);                                             // send the stop bit
  esp_err_t err = i2c_master_cmd_begin(port, cmd, I2C_CMD_TIMEOUT); // perform the command
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

void app_main() {
  printf("Hello world!\n");

  i2c_init();

  uint8_t data = -1, datab = -1;
  esp_err_t err = i2c_read_reg8(I2C_PORT, 0x29, 0xc0, &data);
  err |= i2c_read_reg8(I2C_PORT, 0x29, 0xc1, &datab);
  if (err == 0)
    printf("no err, data is 0x%.2x and 0x%.2x\n", data, datab);
  else
    printf("got err %u", err);
}
