#include "vl53l0x.h"
#include "vl53l0x_calibration.h"
#include "vl53l0x_core.h"
#include "vl53l0x_device.h"
#include "vl53l0x_interrupt_tuning.h"
#include "vl53l0x_tuning.h"

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

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_static_init(vl53l0x_handle_t dev) {
  ERR_CHECK(vl53l0x_get_info_from_dev(dev, 1));

  // set the ref spad from NVM
  uint32_t count    = dev->data.dev_spec_params.ref_spad_count;
  bool     aperture = dev->data.dev_spec_params.ref_spad_aperture;

  // check NVM values
  if ((aperture && count > 32) || (!aperture && count > 12))
    ERR_CHECK(vl53l0x_perform_ref_spad_management(dev, &count, &aperture));
  else
    ERR_CHECK(_vl53l0x_set_ref_spads(dev, count, aperture));

  // load the tuning settings
  ERR_CHECK(vl53l0x_load_tuning_settings(dev, dev->data.use_internal_tuning_settings
                                                  ? VL53L0X_DEFAULT_TUNING
                                                  : dev->data.tuning_settings));

  // set interrupt config to new sample ready
  ERR_CHECK(vl53l0x_set_gpio_config(dev, 0, 0, VL53L0X_GPIOFUNC_NEW_MEAS_READY,
                                    VL53L0X_INTERRUPT_POLARITY_LOW));

  // read oscillator frequency
  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
  uint16_t word;
  ERR_CHECK(vl53l0x_read_16(dev, 0x84, &word));
  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
  dev->data.dev_spec_params.osc_freq_mhz = VL53L0X_FP412_TO_FP1616(word);

  // after static init, some parameters may have changed: update them
  ERR_CHECK(vl53l0x_get_dev_params(dev, &dev->data.current_params));
  ERR_CHECK(vl53l0x_get_range_fraction_enable(dev, &dev->data.range_fractional_enable));

  // read and save system sequence config
  ERR_CHECK(vl53l0x_read_8(dev, SYSTEM_SEQUENCE_CONFIG, &dev->data.seq_conf));

  // disable, by default, MSRC and TCC
  ERR_CHECK(vl53l0x_set_seq_step(dev, VL53L0X_SEQSTEP_MSRC, false));
  ERR_CHECK(vl53l0x_set_seq_step(dev, VL53L0X_SEQSTEP_TCC, false));

  // set PAL state to standby
  dev->data.pal_state = VL53L0X_STATE_IDLE;

  // store pre/final-range periods
  ERR_CHECK(
      vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                     &dev->data.dev_spec_params.pre_range_vcsel_pulse_period));
  ERR_CHECK(
      vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                                     &dev->data.dev_spec_params.final_range_vcsel_pulse_period));

  // store pre/final-range timeouts
  ERR_CHECK(vl53l0x_get_seq_step_timeout(dev, VL53L0X_SEQSTEP_PRE_RANGE,
                                         &dev->data.dev_spec_params.pre_range_timeout_us));
  ERR_CHECK(vl53l0x_get_seq_step_timeout(dev, VL53L0X_SEQSTEP_FINAL_RANGE,
                                         &dev->data.dev_spec_params.final_range_timeout_us));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_wait_dev_booted(vl53l0x_handle_t dev) { return VL53L0X_ERR_NOT_IMPLEMENTED; }

vl53l0x_err_t vl53l0x_reset(vl53l0x_handle_t dev) {
  // set reset bit
  ERR_CHECK(vl53l0x_write_8(dev, SOFT_RESET_GO2_SOFT_RESET_N, 0x00));

  // wait for some time
  uint8_t byte;
  do {
    ERR_CHECK(vl53l0x_read_8(dev, IDENTIFICATION_MODEL_ID, &byte));
  } while (byte != 0x00);

  // release reset bit
  ERR_CHECK(vl53l0x_write_8(dev, SOFT_RESET_GO2_SOFT_RESET_N, 0x01));

  // wait until boot is complete
  do {
    ERR_CHECK(vl53l0x_read_8(dev, IDENTIFICATION_MODEL_ID, &byte));
  } while (byte == 0x00);

  // set pal state to powerdown
  dev->data.pal_state = VL53L0X_STATE_PWRDOWN;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_dev_params(vl53l0x_handle_t dev, const vl53l0x_dev_params_t* params) {
  ERR_CHECK(vl53l0x_set_dev_mode(dev, dev->data.current_params.dev_mode));
  ERR_CHECK(vl53l0x_set_inter_meas_period_ms(dev, dev->data.current_params.inter_meas_period_ms));
  ERR_CHECK(
      vl53l0x_set_xtalk_comp_rate_mcps(dev, dev->data.current_params.xtalk_compensation_rate_mcps));
  ERR_CHECK(vl53l0x_set_offset_calibration_data_um(dev, dev->data.current_params.range_offset_um));

  for (uint8_t i = 0; i < VL53L0X_CHECKS_NUMBER; i++) {
    ERR_CHECK(vl53l0x_set_limit_check(dev, i, dev->data.current_params.limit_checks[i]));
    ERR_CHECK(
        vl53l0x_set_limit_check_value(dev, i, dev->data.current_params.limit_checks_value[i]));
  }

  ERR_CHECK(vl53l0x_set_wrap_around_check(dev, dev->data.current_params.wrap_around_check_enable));
  ERR_CHECK(vl53l0x_set_meas_timing_budget_us(dev, dev->data.current_params.meas_timing_budget_us));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_dev_params(vl53l0x_handle_t dev, vl53l0x_dev_params_t* params) {
  ERR_CHECK(vl53l0x_get_dev_mode(dev, &dev->data.current_params.dev_mode));
  ERR_CHECK(vl53l0x_get_inter_meas_period_ms(dev, &dev->data.current_params.inter_meas_period_ms));
  dev->data.current_params.xtalk_compensation_enabled = false;
  ERR_CHECK(vl53l0x_get_xtalk_comp_rate_mcps(
      dev, &dev->data.current_params.xtalk_compensation_rate_mcps));
  ERR_CHECK(vl53l0x_get_offset_calibration_data_um(dev, &dev->data.current_params.range_offset_um));

  for (uint8_t i = 0; i < VL53L0X_CHECKS_NUMBER; i++) {
    ERR_CHECK(vl53l0x_get_limit_check(dev, i, &dev->data.current_params.limit_checks[i]));
    ERR_CHECK(
        vl53l0x_get_limit_check_value(dev, i, &dev->data.current_params.limit_checks_value[i]));
  }

  ERR_CHECK(vl53l0x_get_wrap_around_check(dev, &dev->data.current_params.wrap_around_check_enable));
  ERR_CHECK(
      vl53l0x_get_meas_timing_budget_us(dev, &dev->data.current_params.meas_timing_budget_us));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_dev_mode(vl53l0x_handle_t dev, vl53l0x_dev_mode_t mode) {
  switch (mode) {
    case VL53L0X_DEVMODE_SINGLE_RANGING:
    case VL53L0X_DEVMODE_CONTINUOUS_RANGING:
    case VL53L0X_DEVMODE_CONTINUOUS_TIMED_RANGING:
    case VL53L0X_DEVMODE_GPIO_DRIVE:
    case VL53L0X_DEVMODE_GPIO_OSC:
      dev->data.current_params.dev_mode = mode;
    default:
      return VL53L0X_ERR_NOT_SUPPORTED;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_dev_mode(vl53l0x_handle_t dev, vl53l0x_dev_mode_t* mode) {
  *mode = dev->data.current_params.dev_mode;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_range_fraction_enable(vl53l0x_handle_t dev, bool enable) {
  ERR_CHECK(vl53l0x_write_8(dev, SYSTEM_RANGE_CONFIG, enable));
  dev->data.range_fractional_enable = enable;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_range_fraction_enable(vl53l0x_handle_t dev, bool* enable) {
  uint8_t byte;
  ERR_CHECK(vl53l0x_read_8(dev, SYSTEM_RANGE_CONFIG, &byte));
  *enable = byte & 0x01;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_meas_timing_budget_us(vl53l0x_handle_t dev, uint32_t budget) {
  ERR_CHECK(_vl53l0x_set_meas_timing_budget_us(dev, budget));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_meas_timing_budget_us(vl53l0x_handle_t dev, uint32_t* budget) {
  ERR_CHECK(_vl53l0x_get_meas_timing_budget_us(dev, budget));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_vcsel_pulse_period(vl53l0x_handle_t dev, vl53l0x_vcsel_period_type_t type,
                                             uint8_t period) {
  ERR_CHECK(_vl53l0x_set_vcsel_pulse_period(dev, type, period));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_vcsel_pulse_period(vl53l0x_handle_t dev, vl53l0x_vcsel_period_type_t type,
                                             uint8_t* period) {
  ERR_CHECK(_vl53l0x_get_vcsel_pulse_period(dev, type, period));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_seq_step(vl53l0x_handle_t dev, vl53l0x_seq_step_t step, bool enabled) {
  uint8_t seq_conf;
  ERR_CHECK(vl53l0x_read_8(dev, SYSTEM_SEQUENCE_CONFIG, &seq_conf));
  uint8_t new_seq_conf = seq_conf;

  if (enabled) {
    // enable specified step
    switch (step) {
      case VL53L0X_SEQSTEP_TCC:
        new_seq_conf |= 0x10;
        break;
      case VL53L0X_SEQSTEP_DSS:
        new_seq_conf |= 0x28;
        break;
      case VL53L0X_SEQSTEP_MSRC:
        new_seq_conf |= 0x04;
        break;
      case VL53L0X_SEQSTEP_PRE_RANGE:
        new_seq_conf |= 0x40;
        break;
      case VL53L0X_SEQSTEP_FINAL_RANGE:
        new_seq_conf |= 0x80;
        break;
      default:
        return VL53L0X_ERR_INVALID_PARAMS;
    }
  } else {
    // disable specified step
    switch (step) {
      case VL53L0X_SEQSTEP_TCC:
        new_seq_conf &= !0x10;
        break;
      case VL53L0X_SEQSTEP_DSS:
        new_seq_conf &= !0x28;
        break;
      case VL53L0X_SEQSTEP_MSRC:
        new_seq_conf &= !0x04;
        break;
      case VL53L0X_SEQSTEP_PRE_RANGE:
        new_seq_conf &= !0x40;
        break;
      case VL53L0X_SEQSTEP_FINAL_RANGE:
        new_seq_conf &= !0x80;
        break;
      default:
        return VL53L0X_ERR_INVALID_PARAMS;
    }
  }

  if (new_seq_conf != seq_conf) {
    // apply new sequence
    ERR_CHECK(vl53l0x_write_8(dev, SYSTEM_SEQUENCE_CONFIG, new_seq_conf));
    dev->data.seq_conf = new_seq_conf;

    // recalculate timing budget
    uint32_t budget = dev->data.current_params.meas_timing_budget_us;
    ERR_CHECK(vl53l0x_set_meas_timing_budget_us(dev, budget));
  }

  return VL53L0X_OK;
}

vl53l0x_err_t is_step_enabled(uint8_t seq_conf, vl53l0x_seq_step_t step, bool* enabled) {
  switch (step) {
    case VL53L0X_SEQSTEP_TCC:
      *enabled = (seq_conf & 0x10) >> 4;
      break;
    case VL53L0X_SEQSTEP_DSS:
      *enabled = (seq_conf & 0x08) >> 3;
      break;
    case VL53L0X_SEQSTEP_MSRC:
      *enabled = (seq_conf & 0x04) >> 2;
      break;
    case VL53L0X_SEQSTEP_PRE_RANGE:
      *enabled = (seq_conf & 0x40) >> 6;
      break;
    case VL53L0X_SEQSTEP_FINAL_RANGE:
      *enabled = (seq_conf & 0x80) >> 7;
      break;
    default:
      return VL53L0X_ERR_INVALID_PARAMS;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_seq_step(vl53l0x_handle_t dev, vl53l0x_seq_step_t step, bool* enabled) {
  uint8_t seq_conf;
  ERR_CHECK(vl53l0x_read_8(dev, SYSTEM_SEQUENCE_CONFIG, &seq_conf));

  ERR_CHECK(is_step_enabled(seq_conf, step, enabled));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_seq_steps(vl53l0x_handle_t dev, vl53l0x_seq_steps_t* steps) {
  uint8_t seq_conf;
  ERR_CHECK(vl53l0x_read_8(dev, SYSTEM_SEQUENCE_CONFIG, &seq_conf));

  ERR_CHECK(is_step_enabled(seq_conf, VL53L0X_SEQSTEP_TCC, &steps->tcc));
  ERR_CHECK(is_step_enabled(seq_conf, VL53L0X_SEQSTEP_DSS, &steps->dss));
  ERR_CHECK(is_step_enabled(seq_conf, VL53L0X_SEQSTEP_MSRC, &steps->msrc));
  ERR_CHECK(is_step_enabled(seq_conf, VL53L0X_SEQSTEP_PRE_RANGE, &steps->pre_range));
  ERR_CHECK(is_step_enabled(seq_conf, VL53L0X_SEQSTEP_FINAL_RANGE, &steps->final_range));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_seq_steps_number(vl53l0x_handle_t dev, uint8_t* number) {
  *number = VL53L0X_SEQ_STEP_CHECKS_NUMBER;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_seq_step_timeout(vl53l0x_handle_t dev, vl53l0x_seq_step_t step,
                                           fp1616_t timeout_ms) {
  uint32_t timeout_us = ((timeout_ms * 1000) + 0x8000) >> 16;

  // read curret value (in order to revert in case of an error)
  fp1616_t old_timeout_us;
  ERR_CHECK(_vl53l0x_get_seq_step_timeout(dev, step, &old_timeout_us));
  // now set the new value
  ERR_CHECK(_vl53l0x_set_seq_step_timeout(dev, step, timeout_us));

  vl53l0x_err_t err =
      vl53l0x_set_meas_timing_budget_us(dev, dev->data.current_params.meas_timing_budget_us);
  if (err != VL53L0X_OK) {  // revert if failed
    ERR_CHECK(_vl53l0x_set_seq_step_timeout(dev, step, old_timeout_us));
    ERR_CHECK(
        vl53l0x_set_meas_timing_budget_us(dev, dev->data.current_params.meas_timing_budget_us));
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_seq_step_timeout(vl53l0x_handle_t dev, vl53l0x_seq_step_t step,
                                           fp1616_t* timeout_ms) {
  uint32_t timeout_us;
  ERR_CHECK(_vl53l0x_get_seq_step_timeout(dev, step, &timeout_us));
  uint32_t whole_part    = 0;
  uint32_t fraction_part = 0;

  whole_part    = timeout_us / 1000;
  fraction_part = timeout_us - (whole_part * 1000);
  *timeout_ms   = (whole_part << 16) + (((fraction_part * 0xffff) + 500) / 1000);

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_seq_step_info(vl53l0x_handle_t dev, vl53l0x_seq_step_t step, char* info) {
  return VL53L0X_ERR_NOT_IMPLEMENTED;
}

vl53l0x_err_t vl53l0x_set_inter_meas_period_ms(vl53l0x_handle_t dev, uint32_t period) {
  uint16_t osc_cal;
  ERR_CHECK(vl53l0x_read_16(dev, OSC_CALIBRATE_VAL, &osc_cal));

  uint32_t period_ms = osc_cal != 0 ? period * osc_cal : period;

  ERR_CHECK(vl53l0x_write_32(dev, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms));
  dev->data.current_params.inter_meas_period_ms = period;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_inter_meas_period_ms(vl53l0x_handle_t dev, uint32_t* period) {
  uint16_t osc_cal;
  ERR_CHECK(vl53l0x_read_16(dev, OSC_CALIBRATE_VAL, &osc_cal));

  uint32_t period_ms;
  ERR_CHECK(vl53l0x_read_32(dev, SYSTEM_INTERMEASUREMENT_PERIOD, &period_ms));
  if (osc_cal != 0) period_ms /= osc_cal;

  dev->data.current_params.inter_meas_period_ms = period_ms;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_xtalk_comp(vl53l0x_handle_t dev, bool enable) {
  fp1616_t temp_fix = !enable || dev->data.lin_correct_gain != 1000
                          ? 0
                          : dev->data.current_params.xtalk_compensation_rate_mcps;

  ERR_CHECK(vl53l0x_write_16(dev, CROSSTALK_COMPENSATION_PEAK_RATE_MCPS,
                             VL53L0X_FP1616_TO_FP313(temp_fix)));

  dev->data.current_params.xtalk_compensation_enabled = enable;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_xtalk_comp(vl53l0x_handle_t dev, bool* enable) {
  *enable = dev->data.current_params.xtalk_compensation_enabled;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_xtalk_comp_rate_mcps(vl53l0x_handle_t dev, fp1616_t rate) {
  if (!dev->data.current_params.xtalk_compensation_enabled) {
    dev->data.current_params.xtalk_compensation_rate_mcps = rate;
  } else {
    uint16_t word = dev->data.lin_correct_gain == 1000 ? VL53L0X_FP1616_TO_FP313(rate) : 0;
    ERR_CHECK(vl53l0x_write_16(dev, CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, word));

    dev->data.current_params.xtalk_compensation_rate_mcps = rate;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_xtalk_comp_rate_mcps(vl53l0x_handle_t dev, fp1616_t* rate) {
  uint16_t word;
  ERR_CHECK(vl53l0x_read_16(dev, CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, &word));
  if (word == 0) {
    *rate = dev->data.current_params.xtalk_compensation_rate_mcps;
    dev->data.current_params.xtalk_compensation_enabled = false;
  } else {
    fp1616_t temp_fix                                     = VL53L0X_FP313_TO_FP1616(word);
    *rate                                                 = temp_fix;
    dev->data.current_params.xtalk_compensation_rate_mcps = temp_fix;
    dev->data.current_params.xtalk_compensation_enabled   = true;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_ref_calibration(vl53l0x_handle_t dev, uint8_t vhv_settings,
                                          uint8_t phase_calibration) {
  ERR_CHECK(_vl53l0x_set_ref_calibration(dev, vhv_settings, phase_calibration));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_ref_calibration(vl53l0x_handle_t dev, uint8_t* vhv_settings,
                                          uint8_t* phase_calibration) {
  ERR_CHECK(_vl53l0x_get_ref_calibration(dev, vhv_settings, phase_calibration));

  return VL53L0X_OK;
}

// check limit functions

vl53l0x_err_t vl53l0x_get_limit_checks_number(vl53l0x_handle_t dev, uint16_t* number) {
  *number = VL53L0X_CHECKS_NUMBER;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_limit_check_info(vl53l0x_handle_t dev, vl53l0x_check_t check,
                                           char* info) {
  return VL53L0X_ERR_NOT_IMPLEMENTED;
}

vl53l0x_err_t vl53l0x_get_limit_check_status(vl53l0x_handle_t dev, vl53l0x_check_t check,
                                             bool* failed) {
  if (check >= VL53L0X_CHECKS_NUMBER) return VL53L0X_ERR_INVALID_PARAMS;

  *failed = dev->data.current_params.limit_checks_status[check];

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_limit_check(vl53l0x_handle_t dev, vl53l0x_check_t check, bool enabled) {
  if (check >= VL53L0X_CHECKS_NUMBER) return VL53L0X_ERR_INVALID_PARAMS;

  fp1616_t value = enabled ? dev->data.current_params.limit_checks_value[check] : 0;

  uint8_t byte;
  switch (check) {
    case VL53L0X_CHECK_SIGNAL_RATE_FINAL_RANGE:
      ERR_CHECK(vl53l0x_write_16(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                                 enabled ? VL53L0X_FP1616_TO_FP97(value) : 0));
      break;
    case VL53L0X_CHECK_SIGMA_FINAL_RANGE:
    case VL53L0X_CHECK_SIGNAL_REF_CLIP:
    case VL53L0X_CHECK_RANGE_IGNORE_THRESHOLD:
      // internal computation
      break;
    case VL53L0X_CHECK_SIGNAL_RATE_MSRC:
      byte = (uint8_t)(!enabled << 1);
      ERR_CHECK(vl53l0x_update_8(dev, MSRC_CONFIG_CONTROL, 0xfe, byte));
      break;
    case VL53L0X_CHECK_SIGNAL_RATE_PRE_RANGE:
      byte = (uint8_t)(!enabled << 4);
      ERR_CHECK(vl53l0x_update_8(dev, MSRC_CONFIG_CONTROL, 0xef, byte));
      break;
    default:
      return VL53L0X_ERR_INVALID_PARAMS;
  }

  dev->data.current_params.limit_checks[check] = enabled;
  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_limit_check(vl53l0x_handle_t dev, vl53l0x_check_t check, bool* enabled) {
  if (check >= VL53L0X_CHECKS_NUMBER) return VL53L0X_ERR_INVALID_PARAMS;
  *enabled = dev->data.current_params.limit_checks[check];

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_limit_check_value(vl53l0x_handle_t dev, vl53l0x_check_t check,
                                            fp1616_t value) {
  if (check >= VL53L0X_CHECKS_NUMBER) return VL53L0X_ERR_INVALID_PARAMS;

  if (dev->data.current_params.limit_checks[check]) {
    switch (check) {
      case VL53L0X_CHECK_SIGMA_FINAL_RANGE:
      case VL53L0X_CHECK_SIGNAL_REF_CLIP:
      case VL53L0X_CHECK_RANGE_IGNORE_THRESHOLD:
        // internal computation
        break;
      case VL53L0X_CHECK_SIGNAL_RATE_FINAL_RANGE:
        ERR_CHECK(vl53l0x_write_16(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                                   VL53L0X_FP1616_TO_FP97(value)));
        break;
      case VL53L0X_CHECK_SIGNAL_RATE_MSRC:
      case VL53L0X_CHECK_SIGNAL_RATE_PRE_RANGE:
        ERR_CHECK(vl53l0x_write_16(dev, PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, value));
        break;
      default:
        return VL53L0X_ERR_INVALID_PARAMS;
    }
  }
  dev->data.current_params.limit_checks_value[check] = value;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_limit_check_value(vl53l0x_handle_t dev, vl53l0x_check_t check,
                                            fp1616_t* value) {
  if (check >= VL53L0X_CHECKS_NUMBER) return VL53L0X_ERR_INVALID_PARAMS;

  fp1616_t device_value = 0;

  switch (check) {
    case VL53L0X_CHECK_SIGMA_FINAL_RANGE:
    case VL53L0X_CHECK_SIGNAL_REF_CLIP:
    case VL53L0X_CHECK_RANGE_IGNORE_THRESHOLD:
      // internal computation
      break;
    case VL53L0X_CHECK_SIGNAL_RATE_FINAL_RANGE:
      ERR_CHECK(vl53l0x_read_32(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &device_value));
      break;
    case VL53L0X_CHECK_SIGNAL_RATE_MSRC:
    case VL53L0X_CHECK_SIGNAL_RATE_PRE_RANGE:
      ERR_CHECK(vl53l0x_read_32(dev, PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, &device_value));
      break;
    default:
      return VL53L0X_ERR_INVALID_PARAMS;
  }

  *value = device_value != 0 ? device_value : dev->data.current_params.limit_checks_value[check];

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_limit_check_current(vl53l0x_handle_t dev, vl53l0x_check_t check,
                                              fp1616_t* current) {
  if (check >= VL53L0X_CHECKS_NUMBER) return VL53L0X_ERR_INVALID_PARAMS;

  switch (check) {
    // NOTE: run a ranging measure to get latest values
    case VL53L0X_CHECK_SIGMA_FINAL_RANGE:
      *current = dev->data.sigma_est;
      break;
    case VL53L0X_CHECK_SIGNAL_REF_CLIP:
      *current = dev->data.last_signal_ref_mcps;
      break;
    case VL53L0X_CHECK_SIGNAL_RATE_FINAL_RANGE:
    case VL53L0X_CHECK_RANGE_IGNORE_THRESHOLD:
    case VL53L0X_CHECK_SIGNAL_RATE_MSRC:
    case VL53L0X_CHECK_SIGNAL_RATE_PRE_RANGE:
      *current = dev->data.last_range_meas.signal_rate_rtn_mcps;
      break;
    default:
      return VL53L0X_ERR_INVALID_PARAMS;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_wrap_around_check(vl53l0x_handle_t dev, bool enabled) {
  uint8_t byte;
  ERR_CHECK(vl53l0x_read_8(dev, SYSTEM_SEQUENCE_CONFIG, &byte));
  byte = enabled ? (byte | 0x80) : (byte & 0xf7);
  ERR_CHECK(vl53l0x_write_8(dev, SYSTEM_SEQUENCE_CONFIG, byte));

  dev->data.current_params.wrap_around_check_enable = enabled;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_wrap_around_check(vl53l0x_handle_t dev, bool* enabled) {
  uint8_t byte;
  ERR_CHECK(vl53l0x_read_8(dev, SYSTEM_SEQUENCE_CONFIG, &byte));
  *enabled = byte & 0x80;

  dev->data.current_params.wrap_around_check_enable = *enabled;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_set_dmax_cal_params(vl53l0x_handle_t dev, uint16_t range_mm,
                                          fp1616_t signal_rate_rtn_mcps) {
  // if one of the parameters is zero read value from NVM
  if (range_mm == 0 || signal_rate_rtn_mcps == 0) {
    // run get_info_from_dev with option 4 to get signal rate at 400mm
    vl53l0x_get_info_from_dev(dev, 4);

    dev->data.dmax_cal_range_mm = 400;
    dev->data.dmax_cal_signal_rate_rtn_mcps =
        dev->data.dev_spec_params.signal_rate_meas_fixed_400mm;
  } else {
    // set user parameters
    dev->data.dmax_cal_range_mm             = range_mm;
    dev->data.dmax_cal_signal_rate_rtn_mcps = signal_rate_rtn_mcps;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_dmax_cal_params(vl53l0x_handle_t dev, uint16_t* range_mm,
                                          fp1616_t* signal_rate_rtn_mcps) {
  *range_mm             = dev->data.dmax_cal_range_mm;
  *signal_rate_rtn_mcps = dev->data.dmax_cal_signal_rate_rtn_mcps;

  return VL53L0X_OK;
}

// PAL measurement functions

vl53l0x_err_t vl53l0x_perform_single_meas(vl53l0x_handle_t dev) {
  vl53l0x_dev_mode_t mode;
  ERR_CHECK(vl53l0x_get_dev_mode(dev, &mode));

  if (mode == VL53L0X_DEVMODE_SINGLE_RANGING) ERR_CHECK(vl53l0x_start_meas(dev));
  ERR_CHECK(vl53l0x_meas_poll_for_completion(dev));

  if (mode == VL53L0X_DEVMODE_SINGLE_RANGING) dev->data.pal_state = VL53L0X_STATE_IDLE;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_perform_ref_calibration(vl53l0x_handle_t dev, uint8_t* vhv_settings,
                                              uint8_t* phase_cal) {
  ERR_CHECK(_vl53l0x_perform_ref_calibration(dev, vhv_settings, phase_cal, true));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_perform_xtalk_calibration(vl53l0x_handle_t dev, fp1616_t xtalk_cal_distance,
                                                fp1616_t* xtalk_compensation_rate_mcps) {
  ERR_CHECK(
      _vl53l0x_perform_xtalk_calibration(dev, xtalk_cal_distance, xtalk_compensation_rate_mcps));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_perform_offset_calibration(vl53l0x_handle_t dev, fp1616_t cal_dist_mm,
                                                 int32_t* offset_um) {
  ERR_CHECK(_vl53l0x_perform_offset_calibration(dev, cal_dist_mm, offset_um));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_check_load_interrupt_settings(vl53l0x_handle_t dev, uint8_t start_flag) {
  uint8_t cfg = dev->data.dev_spec_params.pin_0_gpio_func;

  if (cfg == VL53L0X_GPIOFUNC_THRESHOLD_CROSS_HIGH || cfg == VL53L0X_GPIOFUNC_THRESHOLD_CROSS_LOW ||
      cfg == VL53L0X_GPIOFUNC_THRESHOLD_CROSS_OUT) {
    fp1616_t thresh_low, thresh_high;
    ERR_CHECK(vl53l0x_get_interrupt_threshold(dev, VL53L0X_DEVMODE_CONTINUOUS_RANGING, &thresh_low,
                                              &thresh_high));

    if (thresh_low > 255 * 65535 || thresh_high > 255 * 65536) {
      if (start_flag) {
        ERR_CHECK(vl53l0x_load_tuning_settings(dev, VL53L0X_INTERRUPT_THRESH_TUNING));
      } else {
        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x04));
        ERR_CHECK(vl53l0x_write_8(dev, 0x70, 0x00));
        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
      }
    }
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_start_meas(vl53l0x_handle_t dev) {
  vl53l0x_dev_mode_t mode;
  ERR_CHECK(vl53l0x_get_dev_mode(dev, &mode));

  ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x01));
  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
  ERR_CHECK(vl53l0x_write_8(dev, 0x00, 0x00));
  ERR_CHECK(vl53l0x_write_8(dev, 0x91, dev->data.stop_var));
  ERR_CHECK(vl53l0x_write_8(dev, 0x00, 0x01));
  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
  ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x00));

  switch (mode) {
    case VL53L0X_DEVMODE_SINGLE_RANGING:
      ERR_CHECK(vl53l0x_write_8(dev, SYSRANGE_START, 0x01));
      uint8_t byte = VL53L0X_SYSRANGE_MODE_START_STOP, loops = 0;

      do {  // loop until start bit has been cleared
        ERR_CHECK(vl53l0x_read_8(dev, SYSRANGE_START, &byte));
        loops++;
      } while ((byte & VL53L0X_SYSRANGE_MODE_START_STOP) == VL53L0X_SYSRANGE_MODE_START_STOP &&
               loops < VL53L0X_MAX_LOOPS);

      if (loops >= VL53L0X_MAX_LOOPS) return VL53L0X_ERR_TIMEOUT;
      break;
    case VL53L0X_DEVMODE_CONTINUOUS_RANGING:
      // back-to-back
      // check wether we need to apply interrupt settings
      ERR_CHECK(vl53l0x_check_load_interrupt_settings(dev, true));

      ERR_CHECK(vl53l0x_write_8(dev, SYSRANGE_START, VL53L0X_SYSRANGE_MODE_BACKTOBACK));

      dev->data.pal_state = VL53L0X_STATE_RUNNING;
      break;
    case VL53L0X_DEVMODE_CONTINUOUS_TIMED_RANGING:
      // continuous mode
      // check wether we need to apply interrupt settings
      ERR_CHECK(vl53l0x_check_load_interrupt_settings(dev, true));

      ERR_CHECK(vl53l0x_write_8(dev, SYSRANGE_START, VL53L0X_SYSRANGE_MODE_TIMED));

      dev->data.pal_state = VL53L0X_STATE_RUNNING;
      break;
    default:
      // mode not supported
      return VL53L0X_ERR_MODE_NOT_SUPPORTED;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_stop_meas(vl53l0x_handle_t dev) {
  ERR_CHECK(vl53l0x_write_8(dev, SYSRANGE_START, VL53L0X_SYSRANGE_MODE_SINGLESHOT));

  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
  ERR_CHECK(vl53l0x_write_8(dev, 0x00, 0x00));
  ERR_CHECK(vl53l0x_write_8(dev, 0x91, 0x00));
  ERR_CHECK(vl53l0x_write_8(dev, 0x00, 0x01));
  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));

  dev->data.pal_state = VL53L0X_STATE_IDLE;

  // check wether we need to apply interrupt settings
  ERR_CHECK(vl53l0x_check_load_interrupt_settings(dev, false));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_meas_data_ready(vl53l0x_handle_t dev, bool* ready) {
  if (dev->data.dev_spec_params.pin_0_gpio_func == VL53L0X_GPIOFUNC_NEW_MEAS_READY) {
    uint32_t mask;
    ERR_CHECK(vl53l0x_get_interrupt_mask_status(dev, &mask));

    *ready = (uint8_t)mask == VL53L0X_GPIOFUNC_NEW_MEAS_READY;
  } else {
    uint8_t status;
    ERR_CHECK(vl53l0x_read_8(dev, RESULT_RANGE_STATUS, &status));

    *ready = status & 0x01;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_ranging_meas_data(vl53l0x_handle_t             dev,
                                            vl53l0x_ranging_meas_data_t* data) {
  uint8_t buffer[12];
  ERR_CHECK(vl53l0x_read_n(dev, 0x14, buffer, 12));

  data->zone_id   = 0;  // only one zone
  data->timestamp = 0;  // WARN: not implemented

  // measurement
  uint16_t word = VL53L0X_MAKEUINT16(buffer[11], buffer[10]);

  data->meas_time_us = 0;

  fp1616_t signal_rate       = VL53L0X_FP97_TO_FP1616(VL53L0X_MAKEUINT16(buffer[7], buffer[6]));
  data->signal_rate_rtn_mcps = signal_rate;

  fp1616_t ambient_rate       = VL53L0X_FP97_TO_FP1616(VL53L0X_MAKEUINT16(buffer[9], buffer[8]));
  data->ambient_rate_rtn_mcps = ambient_rate;

  data->effective_spad_rtn_count = VL53L0X_MAKEUINT16(buffer[3], buffer[2]);

  uint8_t status = buffer[0];

  if (dev->data.lin_correct_gain != 1000) {
    word = (uint16_t)((dev->data.lin_correct_gain * word + 500) / 1000);

    // implement xtalk
    uint16_t xtalk_range_mm;
    if (dev->data.current_params.xtalk_compensation_enabled) {
      if ((signal_rate - ((dev->data.current_params.xtalk_compensation_rate_mcps *
                           data->effective_spad_rtn_count) >>
                          8)) <= 0) {
        xtalk_range_mm = dev->data.range_fractional_enable ? 8888 : (8888 << 2);
      } else {
        xtalk_range_mm = (word * signal_rate) /
                         (signal_rate - ((dev->data.current_params.xtalk_compensation_rate_mcps *
                                          data->effective_spad_rtn_count) >>
                                         8));
      }

      word = xtalk_range_mm;
    }
  }

  data->range_mm = dev->data.range_fractional_enable ? (uint16_t)(word >> 2) : word;
  data->range_fractional_part =
      dev->data.range_fractional_enable ? (uint8_t)((word & 0x03) << 6) : 0;

  uint8_t pal_range_status;
  ERR_CHECK(vl53l0x_get_pal_range_status(dev, status, signal_rate, data->effective_spad_rtn_count,
                                         data, &pal_range_status));

  data->range_status = pal_range_status;

  // copy last read data into dev buffer
  dev->data.last_range_meas = *data;

  return VL53L0X_OK;
}
