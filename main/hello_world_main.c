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

void app_main() {
  printf("Hello world!\n");

  i2c_init();

  esp_err_t err;
  // printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
  // printf("00:         ");
  for (int i = 3; i < 0x78; i++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_PORT, cmd, 10 / portTICK_PERIOD_MS);
    if (i % 16 == 0) {
      // printf("\n%.2x:", i);
    }
    if (err == 0) {
      printf("found dev @ 0x%.2x\n", i);
      // printf(" %.2x", i);
    } else {
      printf("err @ 0x%.2x # %i\n", i, err);
      // printf(" --");
    }

    i2c_cmd_link_delete(cmd);
  }
  printf("\n");
}
