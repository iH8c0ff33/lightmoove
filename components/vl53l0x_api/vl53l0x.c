#include "vl53l0x.h"
#include "vl53l0x_calibration.h"
#include "vl53l0x_core.h"

// PAL General functions

vl53l0x_err_t vl53l0x_get_product_rev(vl53l0x_handle_t dev, uint8_t* major, uint8_t* minor) {
  uint8_t rev_id;

  ERR_CHECK(vl53l0x_read_8(dev, IDENTIFICATION_REVISION_ID, &rev_id));
  *major = 1;
  *minor = (uint8_t)((rev_id & 0xf0u) >> 4u);

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_device_info(vl53l0x_handle_t dev, vl53l0x_dev_info_t* info) {
  return VL53L0X_ERR_NOT_IMPLEMENTED;
}

vl53l0x_err_t vl53l0x_get_device_error_status(vl53l0x_handle_t dev, vl53l0x_dev_err_t* error) {
  uint8_t status;

  ERR_CHECK(vl53l0x_read_8(dev, RESULT_RANGE_STATUS, &status));

  *error = (vl53l0x_dev_err_t)((status & 0x78u) >> 3u);

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_state(vl53l0x_handle_t dev, vl53l0x_state_t* state) {
  *state = dev->data.pal_state;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_pwr_mode(vl53l0x_handle_t dev, vl53l0x_pwr_mode_t mode) {
  switch (mode) {
    case VL53L0X_PWRMODE_STDBY_1:
      ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x00));
      dev->data.pal_state = VL53L0X_STATE_STDBY;
      dev->data.pwr_mode  = VL53L0X_PWRMODE_STDBY_1;
      break;

    case VL53L0X_PWRMODE_IDLE_1:
      ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x00));
      ERR_CHECK(vl53l0x_static_init(dev));
      dev->data.pwr_mode = VL53L0X_PWRMODE_IDLE_1;
      break;

    default:
      return VL53L0X_ERR_NOT_SUPPORTED;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_pwr_mode(vl53l0x_handle_t dev, vl53l0x_pwr_mode_t* mode) {
  uint8_t byte;
  ERR_CHECK(vl53l0x_read_8(dev, 0x80, &byte));
  dev->data.pwr_mode = byte == 1 ? VL53L0X_PWRMODE_IDLE_1 : VL53L0X_PWRMODE_STDBY_1;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_offset_calibration_data_um(vl53l0x_handle_t dev, int32_t data) {
  ERR_CHECK(_vl53l0x_set_offset_calibration_data_um(dev, data));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_offset_calibration_data_um(vl53l0x_handle_t dev, int32_t* data) {
  ERR_CHECK(_vl53l0x_get_offset_calibration_data_um(dev, data));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_lin_correct_gain(vl53l0x_handle_t dev, int16_t gain) {
  if (gain < 0 || gain > 1000) return VL53L0X_ERR_INVALID_PARAMS;

  dev->data.lin_correct_gain = gain;
  if (gain != 1000) ERR_CHECK(vl53l0x_write_16(dev, CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, 0));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_lin_correct_gain(vl53l0x_handle_t dev, uint16_t* gain) {
  *gain = dev->data.lin_correct_gain;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_group_param_hold(vl53l0x_handle_t dev, uint8_t hold) {
  return VL53L0X_ERR_NOT_IMPLEMENTED;
}

vl53l0x_err_t vl53l0x_get_upper_limit_mm(vl53l0x_handle_t dev, uint16_t* limit) {
  return VL53L0X_ERR_NOT_IMPLEMENTED;
}

vl53l0x_err_t vl53l0x_get_total_signal_rate(vl53l0x_handle_t dev, fp1616_t* rate) {
  ERR_CHECK(_vl53l0x_get_total_signal_rate(dev, &dev->data.last_range_meas, rate));

  return VL53L0X_OK;
}

// PAL Init functions

vl53l0x_err_t vl53l0x_set_dev_addr(vl53l0x_handle_t dev, uint8_t address) {
  ERR_CHECK(vl53l0x_write_8(dev, I2C_SLAVE_DEVICE_ADDRESS, address / 2));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_data_init(vl53l0x_handle_t dev) {
  vl53l0x_dev_params_t current_params;

/* by default the I2C is running at 1V8 if you want to change it you
 * need to include this define at compilation level. */
#ifdef USE_I2C_2V8
  ERR_CHECK(vl53l0x_update_8(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0xfe, 0x01));
#endif

  // set i2c standard mode
  ERR_CHECK(vl53l0x_write_8(dev, 0x88, 0x00));
  // read who am i
  uint8_t byte;
  ERR_CHECK(vl53l0x_read_8(dev, 0xc0, &byte));

  dev->data.dev_spec_params.read_from_dev_done = false;

#ifdef USE_IQC_STATION
  ERR_CHECK(_vl53l0x_apply_offset_adjustment(dev));
#endif

  // linearity corrective gain default value is 1000
  dev->data.lin_correct_gain = 1000;
  // dmax default parameter
  dev->data.dmax_cal_range_mm             = 400;
  dev->data.dmax_cal_signal_rate_rtn_mcps = (fp1616_t)0x00016b85;

  // set default static parameters
  dev->data.dev_spec_params.osc_freq_mhz = 618660;
  // set default xtalk compensation rate to 0
  dev->data.current_params.xtalk_compensation_rate_mcps = 0;

  // get default parameters
  ERR_CHECK(vl53l0x_get_dev_params(dev, &current_params));
  // initialize pal values
  current_params.dev_mode  = VL53L0X_DEVMODE_SINGLE_RANGING;
  current_params.hist_mode = VL53L0X_HISTMODE_DISABLED;
  dev->data.current_params = current_params;

  // sigma estimator variable
  dev->data.sigma_est_ref_array       = 100;
  dev->data.sigma_est_eff_pulse_width = 900;
  dev->data.sigma_est_eff_amd_width   = 500;
  dev->data.target_ref_rate           = 0x0a00;  // 20 MCPS in 9:7 format

  dev->data.use_internal_tuning_settings = 1;

  ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x01));
  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
  ERR_CHECK(vl53l0x_write_8(dev, 0x00, 0x00));
  ERR_CHECK(vl53l0x_read_8(dev, 0x91, &dev->data.stop_var));
  ERR_CHECK(vl53l0x_write_8(dev, 0x00, 0x01));
  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
  ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x00));

  // enable all checks
  for (uint8_t i = 0; i < VL53L0X_CHECKS_NUMBER; i++) {
    ERR_CHECK(vl53l0x_set_limit_check(dev, i, true));
  }

  // disable following checks
  ERR_CHECK(vl53l0x_set_limit_check(dev, VL53L0X_CHECK_SIGNAL_REF_CLIP, false));
  ERR_CHECK(vl53l0x_set_limit_check(dev, VL53L0X_CHECK_RANGE_IGNORE_THRESHOLD, false));
  ERR_CHECK(vl53l0x_set_limit_check(dev, VL53L0X_CHECK_SIGNAL_RATE_MSRC, false));
  ERR_CHECK(vl53l0x_set_limit_check(dev, VL53L0X_CHECK_SIGNAL_RATE_PRE_RANGE, false));

  // limit default values
  ERR_CHECK(
      vl53l0x_set_limit_check_value(dev, VL53L0X_CHECK_SIGMA_FINAL_RANGE, (fp1616_t)(18 * 65536)));
  ERR_CHECK(vl53l0x_set_limit_check_value(dev, VL53L0X_CHECK_SIGNAL_RATE_FINAL_RANGE,
                                          (fp1616_t)(25 * 65536 / 100)));
  ERR_CHECK(
      vl53l0x_set_limit_check_value(dev, VL53L0X_CHECK_SIGNAL_REF_CLIP, (fp1616_t)(35 * 65536)));
  ERR_CHECK(vl53l0x_set_limit_check_value(dev, VL53L0X_CHECK_RANGE_IGNORE_THRESHOLD,
                                          (fp1616_t)(0 * 65536)));

  dev->data.seq_conf = 0xff;
  ERR_CHECK(vl53l0x_write_8(dev, SYSTEM_SEQUENCE_CONFIG, 0xff));
  // set pal state to tell we are waiting for static_init
  dev->data.pal_state = VL53L0X_STATE_WAIT_STATICINIT;

  dev->data.dev_spec_params.ref_spad_initialised = false;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_tuning_settings(vl53l0x_handle_t dev, uint8_t* settings,
                                          bool use_internal) {
  dev->data.use_internal_tuning_settings = use_internal;
  if (!use_internal) dev->data.tuning_settings = settings;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_tuning_settings(vl53l0x_handle_t dev, uint8_t** settings,
                                          bool* use_internal) {
  *settings     = dev->data.tuning_settings;
  *use_internal = dev->data.use_internal_tuning_settings;
}
