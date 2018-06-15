#include "vl53l0x.h"

struct vl53l0x {
  char     addr;
  char     port;
  int      timeout;
  uint32_t meas_timing_budget;
};

vl53l0x_handle_t vl53l0x_create(vl53l0x_config_t *config) {
  vl53l0x_handle_t handle = malloc(sizeof(struct vl53l0x));

  handle->addr    = config->addr;
  handle->port    = config->port;
  handle->timeout = config->timeout;

  return handle;
}

uint32_t _timeout_mclks_to_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ns = CALC_MACRO_PERIOD(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

uint32_t _timeout_us_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ns = CALC_MACRO_PERIOD(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

esp_err_t _vl53l0x_write(vl53l0x_handle_t vl53l0x, uint8_t reg, uint8_t *data, uint8_t length) {
  return i2c_write_reg(vl53l0x->port, vl53l0x->timeout, vl53l0x->addr, reg, data, length);
}

esp_err_t _vl53l0x_write8(vl53l0x_handle_t vl53l0x, uint8_t reg, uint8_t data) {
  return i2c_write_reg8(vl53l0x->port, vl53l0x->timeout, vl53l0x->addr, reg, data);
}

esp_err_t _vl53l0x_write16(vl53l0x_handle_t vl53l0x, uint8_t reg, uint16_t data) {
  return i2c_write_reg16(vl53l0x->port, vl53l0x->timeout, vl53l0x->addr, reg, data);
}

esp_err_t _vl53l0x_read(vl53l0x_handle_t vl53l0x, uint8_t reg, uint8_t *data, uint8_t length) {
  return i2c_read_reg(vl53l0x->port, vl53l0x->timeout, vl53l0x->addr, reg, data, length);
}

esp_err_t _vl53l0x_read8(vl53l0x_handle_t vl53l0x, uint8_t reg, uint8_t *data) {
  return i2c_read_reg8(vl53l0x->port, vl53l0x->timeout, vl53l0x->addr, reg, data);
}

esp_err_t _vl53l0x_read16(vl53l0x_handle_t vl53l0x, uint8_t reg, uint16_t *data) {
  return i2c_read_reg16(vl53l0x->port, vl53l0x->timeout, vl53l0x->addr, reg, data);
}

esp_err_t _vl53l0x_update8(vl53l0x_handle_t vl53l0x, uint8_t reg, uint8_t update) {
  return i2c_update_reg8(vl53l0x->port, vl53l0x->timeout, vl53l0x->addr, reg, update);
}

esp_err_t _vl53l0x_remove8(vl53l0x_handle_t vl53l0x, uint8_t reg, uint8_t remove) {
  return i2c_remove_reg8(vl53l0x->port, vl53l0x->timeout, vl53l0x->addr, reg, remove);
}

esp_err_t _vl53l0x_start_proc(vl53l0x_handle_t vl53l0x) {
  esp_err_t err = _vl53l0x_write8(vl53l0x, 0x80, 0x01);
  err |= _vl53l0x_write8(vl53l0x, 0xff, 0x01);
  err |= _vl53l0x_write8(vl53l0x, 0x00, 0x00);
  return err;
}

esp_err_t _vl53l0x_stop_proc(vl53l0x_handle_t vl53l0x) {
  esp_err_t err = _vl53l0x_write8(vl53l0x, 0x00, 0x01);
  err |= _vl53l0x_write8(vl53l0x, 0xff, 0x00);
  err |= _vl53l0x_write8(vl53l0x, 0x80, 0x00);
  return err;
}

esp_err_t _vl53l0x_set_signal_rate_limit(vl53l0x_handle_t vl53l0x, float limit) {
  if (limit < 0 || limit > 511.99)
    return ESP_ERR_INVALID_ARG;

  return _vl53l0x_write16(vl53l0x, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit * (1 << 7));
}

esp_err_t _vl53l0x_get_signal_rate_limit(vl53l0x_handle_t vl53l0x, float *limit) {
  uint16_t  data;
  esp_err_t err = _vl53l0x_read16(vl53l0x, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &data);

  *limit = data / (1 << 7);
  return err;
}

esp_err_t _vl53l0x_get_spad_info(vl53l0x_handle_t vl53l0x, uint8_t *count, bool *aperture) {
  esp_err_t err = ESP_OK;

  ERR_CHECK(_vl53l0x_start_proc(vl53l0x));

  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0xff, 0x06));
  ERR_CHECK(_vl53l0x_update8(vl53l0x, 0x83, 0x04));
  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0xff, 0x07));
  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0x81, 0x01));

  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0x80, 0x01));

  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0x94, 0x6b));
  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0x83, 0x00));

  uint8_t data = 0;
  while (data == 0) // TODO: WARN: That loop could be fatal
    _vl53l0x_read8(vl53l0x, 0x83, &data);
  printf("out of loop 0x83 is 0x%.2x\n", data);
  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0x83, 0x01));

  ERR_CHECK(_vl53l0x_read8(vl53l0x, 0x92, &data));
  *count    = data & 0x7f;
  *aperture = (data & 0x80) >> 7;
  printf("count is %u and type is%s aperture\n", *count, *aperture ? "" : " not");

  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0x81, 0x00));
  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0xff, 0x06));
  ERR_CHECK(_vl53l0x_remove8(vl53l0x, 0x83, 0x04));
  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0xff, 0x01));

  ERR_CHECK(_vl53l0x_stop_proc(vl53l0x));

  return err;
}

esp_err_t _vl53l0x_set_spad_ref(vl53l0x_handle_t vl53l0x, uint8_t count, bool aperture) {
  uint8_t spad_ref_map[6];
  ERR_CHECK(_vl53l0x_read(vl53l0x, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_ref_map, 6));

  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0xff, 0x01));
  ERR_CHECK(_vl53l0x_write8(vl53l0x, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00));
  ERR_CHECK(_vl53l0x_write8(vl53l0x, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2c));
  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0xff, 0x00));
  ERR_CHECK(_vl53l0x_write8(vl53l0x, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xb4));

  uint8_t spad_first   = aperture ? 12 : 0; // 12 is the first aperture SPAD
  uint8_t spad_enabled = 0;

  for (uint8_t i = 0; i < 48; i++) {
    if (i < spad_first || spad_enabled == count) {
      // This bit `i` is either before the first to be enabled or after the
      // last one (according to spad_count), then it should be zeroed.
      spad_ref_map[i / 8] &= ~(1 << (i % 8));
    } else if ((spad_ref_map[i / 8] >> (i % 8)) & 0x1) {
      spad_enabled++;
    }
  }

  ERR_CHECK(_vl53l0x_write(vl53l0x, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_ref_map, 6));

  return ESP_OK;
}

esp_err_t _vl53l0x_load_tuning(vl53l0x_handle_t vl53l0x, const uint8_t *tuning) {
  uint8_t i = 0;
  while ((tuning[i] != 0xff || tuning[i + 1] != 0xff) && i < 180) {
    printf("tune 0x%.2x <- 0x%.2x\n", tuning[i], tuning[i + 1]);
    ERR_CHECK(_vl53l0x_write8(vl53l0x, tuning[i], tuning[i + 1]));
    i += 2;
  }

  return ESP_OK;
}

esp_err_t _vl53l0x_set_gpio_config(vl53l0x_handle_t vl53l0x) {
  ERR_CHECK(_vl53l0x_write8(vl53l0x, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04));
  ERR_CHECK(_vl53l0x_remove8(vl53l0x, GPIO_HV_MUX_ACTIVE_HIGH, 0x10));
  ERR_CHECK(_vl53l0x_write8(vl53l0x, SYSTEM_INTERRUPT_CLEAR, 0x01));

  return ESP_OK;
}

esp_err_t _vl53l0x_get_sequence_steps(vl53l0x_handle_t vl53l0x, sequence_steps_t *steps) {
  uint8_t sequence_config;
  ERR_CHECK(_vl53l0x_read8(vl53l0x, SYSTEM_SEQUENCE_CONFIG, &sequence_config));

  steps->msrc        = (sequence_config >> 2) & 0x1;
  steps->dss         = (sequence_config >> 3) & 0x1;
  steps->tcc         = (sequence_config >> 4) & 0x1;
  steps->pre_range   = (sequence_config >> 6) & 0x1;
  steps->final_range = (sequence_config >> 7) & 0x1;

  return ESP_OK;
}

esp_err_t _vl53l0x_get_vcsel_pulse_period(vl53l0x_handle_t vl53l0x, vcsel_period_t type,
                                          uint8_t *period) {
  ERR_CHECK(_vl53l0x_read8(vl53l0x,
                           type == VCSEL_PERIOD_PRE_RANGE ? PRE_RANGE_CONFIG_VCSEL_PERIOD
                                                          : FINAL_RANGE_CONFIG_VCSEL_PERIOD,
                           period));
  *period = DECODE_VCSEL_PERIOD(*period);

  return ESP_OK;
}

esp_err_t _vl53l0x_get_sequence_steps_timeouts(vl53l0x_handle_t vl53l0x, sequence_steps_t *steps,
                                               sequence_steps_timeouts_t *timeouts) {
  uint8_t data;
  ERR_CHECK(_vl53l0x_get_vcsel_pulse_period(vl53l0x, VCSEL_PERIOD_PRE_RANGE, &data));
  timeouts->pre_range_vcsel_period_pclks = data;

  ERR_CHECK(_vl53l0x_read8(vl53l0x, MSRC_CONFIG_TIMEOUT_MACROP, &data));
  timeouts->msrc_dss_tcc_mckls = data + 1;
  timeouts->msrc_dss_tcc_us =
      _timeout_mclks_to_us(timeouts->msrc_dss_tcc_mckls, timeouts->pre_range_vcsel_period_pclks);

  uint16_t data16;
  ERR_CHECK(_vl53l0x_read16(vl53l0x, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &data16));
  timeouts->pre_range_mclks = DECODE_TIMEOUT(data16);
  timeouts->pre_range_us =
      _timeout_mclks_to_us(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

  ERR_CHECK(_vl53l0x_get_vcsel_pulse_period(vl53l0x, VCSEL_PERIOD_FINAL_RANGE, &data));
  timeouts->final_range_vcsel_period_pckls = data;

  ERR_CHECK(_vl53l0x_read16(vl53l0x, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &data16));
  timeouts->final_range_mckls = DECODE_TIMEOUT(data16);

  if (steps->pre_range)
    timeouts->final_range_mckls -= timeouts->pre_range_mclks;

  timeouts->final_range_us =
      _timeout_mclks_to_us(timeouts->final_range_mckls, timeouts->final_range_vcsel_period_pckls);

  return ESP_OK;
}

esp_err_t _vl53l0x_get_meas_timing_budget(vl53l0x_handle_t vl53l0x, uint32_t *output) {
  uint16_t const overhead_start       = 1910;
  uint16_t const overhead_end         = 960;
  uint16_t const overhead_msrc        = 660;
  uint16_t const overhead_tcc         = 590;
  uint16_t const overhead_dss         = 690;
  uint16_t const overhead_pre_range   = 660;
  uint16_t const overhead_final_range = 550;

  uint32_t budget_us = overhead_start + overhead_end;

  sequence_steps_t steps;
  ERR_CHECK(_vl53l0x_get_sequence_steps(vl53l0x, &steps));
  sequence_steps_timeouts_t timeouts;
  ERR_CHECK(_vl53l0x_get_sequence_steps_timeouts(vl53l0x, &steps, &timeouts));

  if (steps.tcc)
    budget_us += timeouts.msrc_dss_tcc_us + overhead_tcc;

  if (steps.dss)
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + overhead_dss);

  if (steps.msrc)
    budget_us += timeouts.msrc_dss_tcc_us + overhead_msrc;

  if (steps.pre_range)
    budget_us += timeouts.pre_range_us + overhead_pre_range;

  if (steps.final_range)
    budget_us += timeouts.final_range_us + overhead_final_range;

  *output = budget_us;

  return ESP_OK;
}

// esp_err_t _vl53l0x_set_sequence_steps

esp_err_t _vl53l0x_data_init(vl53l0x_handle_t vl53l0x) {
#ifdef I2C_2V8
  ERR_CHECK(_vl53l0x_update8(vl53l0x, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0x01));
#endif

  ERR_CHECK(_vl53l0x_write8(vl53l0x, 0x88, 0x00));
  ERR_CHECK(_vl53l0x_start_proc(vl53l0x));
  uint8_t stop_var;
  ERR_CHECK(_vl53l0x_read8(vl53l0x, 0x91, &stop_var));
  ERR_CHECK(_vl53l0x_stop_proc(vl53l0x));

  ERR_CHECK(_vl53l0x_update8(vl53l0x, MSRC_CONFIG_CONTROL, 0x12));

  ERR_CHECK(_vl53l0x_set_signal_rate_limit(vl53l0x, 0.25));

  ERR_CHECK(_vl53l0x_write8(vl53l0x, SYSTEM_SEQUENCE_CONFIG, 0xff));

  return ESP_OK;
}

esp_err_t _vl53l0x_static_init(vl53l0x_handle_t vl53l0x) {
  uint8_t spad_count;
  bool    spad_aperture;
  ERR_CHECK(_vl53l0x_get_spad_info(vl53l0x, &spad_count, &spad_aperture));
  ERR_CHECK(_vl53l0x_set_spad_ref(vl53l0x, spad_count, spad_aperture));

  ERR_CHECK(_vl53l0x_load_tuning(vl53l0x, DEFAULT_TUNING));

  ERR_CHECK(_vl53l0x_set_gpio_config(vl53l0x));

  uint32_t budget;
  ERR_CHECK(_vl53l0x_get_meas_timing_budget(vl53l0x, &budget));
  vl53l0x->meas_timing_budget = budget;

  return ESP_OK;
}

esp_err_t vl53l0x_init(vl53l0x_handle_t vl53l0x) {
  ERR_CHECK(_vl53l0x_data_init(vl53l0x));
  ERR_CHECK(_vl53l0x_static_init(vl53l0x));

  return ESP_OK;
}


const uint8_t DEFAULT_TUNING[] = {
    0xFF, 0x01, //
    0x00, 0x00, //

    0xFF, 0x00, //
    0x09, 0x00, //
    0x10, 0x00, //
    0x11, 0x00, //

    0x24, 0x01, //
    0x25, 0xff, //
    0x75, 0x00, //

    0xFF, 0x01, //
    0x4e, 0x2c, //
    0x48, 0x00, //
    0x30, 0x20, //

    0xFF, 0x00, //
    0x30, 0x09, // /* mja changed from 0x64. */
    0x54, 0x00, //
    0x31, 0x04, //
    0x32, 0x03, //
    0x40, 0x83, //
    0x46, 0x25, //
    0x60, 0x00, //
    0x27, 0x00, //
    0x50, 0x06, //
    0x51, 0x00, //
    0x52, 0x96, //
    0x56, 0x08, //
    0x57, 0x30, //
    0x61, 0x00, //
    0x62, 0x00, //
    0x64, 0x00, //
    0x65, 0x00, //
    0x66, 0xa0, //

    0xFF, 0x01, //
    0x22, 0x32, //
    0x47, 0x14, //
    0x49, 0xff, //
    0x4a, 0x00, //

    0xFF, 0x00, //
    0x7a, 0x0a, //
    0x7b, 0x00, //
    0x78, 0x21, //

    0xFF, 0x01, //
    0x23, 0x34, //
    0x42, 0x00, //
    0x44, 0xff, //
    0x45, 0x26, //
    0x46, 0x05, //
    0x40, 0x40, //
    0x0E, 0x06, //
    0x20, 0x1a, //
    0x43, 0x40, //

    0xFF, 0x00, //
    0x34, 0x03, //
    0x35, 0x44, //

    0xFF, 0x01, //
    0x31, 0x04, //
    0x4b, 0x09, //
    0x4c, 0x05, //
    0x4d, 0x04, //

    0xFF, 0x00, //
    0x44, 0x00, //
    0x45, 0x20, //
    0x47, 0x08, //
    0x48, 0x28, //
    0x67, 0x00, //
    0x70, 0x04, //
    0x71, 0x01, //
    0x72, 0xfe, //
    0x76, 0x00, //
    0x77, 0x00, //

    0xFF, 0x01, //
    0x0d, 0x01, //

    0xFF, 0x00, //
    0x80, 0x01, //
    0x01, 0xF8, //

    0xFF, 0x01, //
    0x8e, 0x01, //
    0x00, 0x01, //
    0xFF, 0x00, //
    0x80, 0x00, //

    0xff, 0xff // END
};
