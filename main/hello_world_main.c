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
#define GPIO_INPUT_0 27
#define GPIO_INPUT_1 0
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_INPUT_0) | (1ULL << GPIO_INPUT_1))
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle     gpio_event_queue = NULL;
static vl53l0x_handle_t vl53l0x          = NULL;
static bool             check            = false;
static bool             init_done        = false;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_event_queue, &gpio_num, NULL);
}

static void gpio_task(void *arg) {
  uint32_t io_num;
  for (;;) {
    if (xQueueReceive(gpio_event_queue, &io_num, portMAX_DELAY)) {
      if (io_num == 0) {
        ESP_ERROR_CHECK(vl53l0x_start_continuous_range(vl53l0x, 100));
      } else if (init_done) {
        uint16_t  measurement;
        esp_err_t err = vl53l0x_read_range_mm(vl53l0x, &measurement);
        if (err != ESP_OK) {
          printf("measure failed: %s\n", esp_err_to_name(err));
        } else {
          printf("measured %umm\n", measurement);
          if (measurement < 250 && !check)
            printf("GOT %u\n", measurement);
          check = measurement < 250;
        }
      }
    }
  }
}

void i2c_init(void) {
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
  esp_log_level_set("vl53l0x", ESP_LOG_DEBUG);

  printf("Hello world!\n");
  gpio_config_t io_conf;
  io_conf.intr_type    = GPIO_PIN_INTR_NEGEDGE;
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  io_conf.mode         = GPIO_MODE_INPUT;
  gpio_config(&io_conf);

  gpio_set_pull_mode(GPIO_INPUT_1, GPIO_PULLUP_ONLY);

  gpio_event_queue = xQueueCreate(10, sizeof(uint32_t));
  xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(GPIO_INPUT_0, gpio_isr_handler, (void *)GPIO_INPUT_0);
  gpio_isr_handler_add(GPIO_INPUT_1, gpio_isr_handler, (void *)GPIO_INPUT_1);

  i2c_init();

  vl53l0x_config_t config = {
      .addr          = I2C_ADDR,
      .port          = I2C_PORT,
      .timeout       = I2C_CMD_TIMEOUT,
      .io_timeout_us = 20 * 1000,
  };
  vl53l0x       = vl53l0x_create(&config);
  esp_err_t err = vl53l0x_init(vl53l0x);

  printf("init result was %s\n", esp_err_to_name(err));
  // vl53l0x_set_signal_rate_limit(vl53l0x, 0.1);
  // vl53l0x_set_vcsel_pulse_period(vl53l0x, VCSEL_PERIOD_PRE_RANGE, 18);
  // vl53l0x_set_vcsel_pulse_period(vl53l0x, VCSEL_PERIOD_FINAL_RANGE, 14);
  err = vl53l0x_set_meas_timing_budget(vl53l0x, 20000);
  // vTaskDelay(3000 / portTICK_PERIOD_MS);
  printf("set meas timing budget: %s\n", esp_err_to_name(err));
  init_done = true;

  // while (true) {
  // ESP_ERROR_CHECK(vl53l0x_perform_range_single(vl53l0x));
  // ESP_ERROR_CHECK(vl53l0x_start_continuous_range(vl53l0x, 10));
  // vTaskDelay(500 / portTICK_PERIOD_MS);
  // }
}
