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
#include "i2c_proto.h"
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

void i2c_init() {
  int i2c_port = I2C_PORT;

  i2c_config_t cfg;
  cfg.mode             = I2C_MODE_MASTER;
  cfg.sda_io_num       = SDA_IO;
  cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
  cfg.scl_io_num       = SCL_IO;
  cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
  cfg.master.clk_speed = I2C_FREQ;
  i2c_param_config(i2c_port, &cfg);
  i2c_driver_install(i2c_port, cfg.mode, I2C_RX_BUF, I2C_TX_BUF, 0);
}

void app_main() {
  printf("Hello world!\n");

  i2c_init();

  vl53l0x_config_t config = {
      .addr    = I2C_ADDR,
      .port    = I2C_PORT,
      .timeout = I2C_CMD_TIMEOUT,
  };
  vl53l0x_handle_t vl53l0x = vl53l0x_create(&config);
  esp_err_t        err     = vl53l0x_init(vl53l0x);

  if (err == 0)
    printf("no err\n");
  else
    printf("error occurred: %.2x\n", err);
}
