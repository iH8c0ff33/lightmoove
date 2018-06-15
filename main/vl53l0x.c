#include "vl53l0x.h"

struct vl53l0x {
  char addr;
  char port;
  int  timeout;
};

vl53l0x_handle_t vl53l0x_create(vl53l0x_config_t *config) {
  vl53l0x_handle_t handle = malloc(sizeof(struct vl53l0x));

  handle->addr    = config->addr;
  handle->port    = config->port;
  handle->timeout = config->timeout;

  return handle;
}

esp_err_t _vl53l0x_write8(vl53l0x_handle_t vl53l0x, uint8_t reg, uint8_t data) {
  return i2c_write_reg8(vl53l0x->port, vl53l0x->timeout, vl53l0x->addr, reg, data);
}

esp_err_t _vl53l0x_write16(vl53l0x_handle_t vl53l0x, uint8_t reg, uint16_t data) {
  return i2c_write_reg16(vl53l0x->port, vl53l0x->timeout, vl53l0x->addr, reg, data);
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

  return _vl53l0x_write16(vl53l0x, FINAL_RANGE_CONFIG_MIN_COUNT_RTN_LIMIT, limit * (1 << 7));
}

esp_err_t _vl53l0x_get_signal_rate_limit(vl53l0x_handle_t vl53l0x, float *limit) {
  uint16_t  data;
  esp_err_t err = _vl53l0x_read16(vl53l0x, FINAL_RANGE_CONFIG_MIN_COUNT_RTN_LIMIT, &data);

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

#define ERR_CHECK(x)                                                                               \
  do {                                                                                             \
    esp_err_t retval = (x);                                                                        \
    if (retval != ESP_OK)                                                                          \
      return retval;                                                                               \
  } while (0)

esp_err_t vl53l0x_init(vl53l0x_handle_t vl53l0x) {
  esp_err_t err = ESP_OK;

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

  uint8_t spad_count;
  bool    spad_aperture;
  ERR_CHECK(_vl53l0x_get_spad_info(vl53l0x, &spad_count, &spad_aperture));

  return err;
}
