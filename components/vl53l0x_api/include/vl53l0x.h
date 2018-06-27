#ifndef _VL53L0X_H
#define _VL53L0X_H

#include "esp_system.h"
#include "vl53l0x_platform.h"

vl53l0x_err_t vl53l0x_get_product_rev(vl53l0x_handle_t dev, uint8_t* major, uint8_t* minor);
vl53l0x_err_t vl53l0x_get_device_info(vl53l0x_handle_t dev, vl53l0x_dev_info_t* info);
vl53l0x_err_t vl53l0x_get_device_error_status(vl53l0x_handle_t dev, vl53l0x_dev_err_t* error);

// vl53l0x_err_t vl53l0x_get_range_status_string(uint8_t status, char* string);
// vl53l0x_err_t vl53l0x_get_dev_err_string(vl53l0x_dev_err_t error, char* string);
// vl53l0x_err_t vl53l0x_get_err_string(vl53l0x_err_t error, char* string);
// vl53l0x_err_t vl53l0x_get_state_string(vl53l0x_state_t state, char* string);

vl53l0x_err_t vl53l0x_get_state(vl53l0x_handle_t dev, vl53l0x_state_t* state);

vl53l0x_err_t vl53l0x_set_pwr_mode(vl53l0x_handle_t dev, vl53l0x_pwr_mode_t mode);
vl53l0x_err_t vl53l0x_get_pwr_mode(vl53l0x_handle_t dev, vl53l0x_pwr_mode_t* mode);

vl53l0x_err_t vl53l0x_set_offset_calibration_data_um(vl53l0x_handle_t dev, int32_t data);
vl53l0x_err_t vl53l0x_get_offset_calibration_data_um(vl53l0x_handle_t dev, int32_t* data);
vl53l0x_err_t vl53l0x_set_lin_correct_gain(vl53l0x_handle_t dev, int16_t gain);
vl53l0x_err_t vl53l0x_get_lin_correct_gain(vl53l0x_handle_t dev, uint16_t* gain);

vl53l0x_err_t vl53l0x_get_total_signal_rate(vl53l0x_handle_t dev, fp1616_t* rate);

vl53l0x_err_t vl53l0x_set_dev_addr(vl53l0x_handle_t dev, uint8_t address);

vl53l0x_err_t vl53l0x_data_init(vl53l0x_handle_t dev);

vl53l0x_err_t vl53l0x_set_tuning_settings(vl53l0x_handle_t dev, uint8_t* settings,
                                          bool use_internal);
vl53l0x_err_t vl53l0x_get_tuning_settings(vl53l0x_handle_t dev, uint8_t** settings,
                                          bool* use_internal);

vl53l0x_err_t vl53l0x_static_init(vl53l0x_handle_t dev);

vl53l0x_err_t vl53l0x_wait_dev_booted(vl53l0x_handle_t dev);

vl53l0x_err_t vl53l0x_reset(vl53l0x_handle_t dev);

vl53l0x_err_t vl53l0x_set_dev_params(vl53l0x_handle_t dev, const vl53l0x_dev_params_t* params);
vl53l0x_err_t vl53l0x_get_dev_params(vl53l0x_handle_t dev, vl53l0x_dev_params_t* params);

vl53l0x_err_t vl53l0x_set_dev_mode(vl53l0x_handle_t dev, vl53l0x_dev_mode_t mode);
vl53l0x_err_t vl53l0x_get_dev_mode(vl53l0x_handle_t dev, vl53l0x_dev_mode_t* mode);

vl53l0x_err_t vl53l0x_set_range_fraction_enable(vl53l0x_handle_t dev, bool enable);
vl53l0x_err_t vl53l0x_get_range_fraction_enable(vl53l0x_handle_t dev, bool* enable);

vl53l0x_err_t vl53l0x_set_meas_timing_budget_us(vl53l0x_handle_t dev, uint32_t budget);
vl53l0x_err_t vl53l0x_get_meas_timing_budget_us(vl53l0x_handle_t dev, uint32_t* budget);

vl53l0x_err_t vl53l0x_set_vcsel_pulse_period(vl53l0x_handle_t dev, vl53l0x_vcsel_period_type_t type,
                                             uint8_t period);
vl53l0x_err_t vl53l0x_get_vcsel_pulse_period(vl53l0x_handle_t dev, vl53l0x_vcsel_period_type_t type,
                                             uint8_t* period);

vl53l0x_err_t vl53l0x_set_seq_step(vl53l0x_handle_t dev, vl53l0x_seq_step_t step, bool enabled);
vl53l0x_err_t vl53l0x_get_seq_step(vl53l0x_handle_t dev, vl53l0x_seq_step_t step, bool* enabled);
vl53l0x_err_t vl53l0x_get_seq_steps(vl53l0x_handle_t dev, vl53l0x_seq_steps_t* steps);
vl53l0x_err_t vl53l0x_get_seq_steps_number(vl53l0x_handle_t dev, uint8_t* number);
vl53l0x_err_t vl53l0x_set_seq_step_timeout(vl53l0x_handle_t dev, vl53l0x_seq_step_t step,
                                           fp1616_t timeout_ms);
vl53l0x_err_t vl53l0x_get_seq_step_timeout(vl53l0x_handle_t dev, vl53l0x_seq_step_t step,
                                           fp1616_t* timeout_ms);
vl53l0x_err_t vl53l0x_get_seq_step_info(vl53l0x_handle_t dev, vl53l0x_seq_step_t step, char* info);

vl53l0x_err_t vl53l0x_set_inter_meas_period_ms(vl53l0x_handle_t dev, uint32_t period);
vl53l0x_err_t vl53l0x_get_inter_meas_period_ms(vl53l0x_handle_t dev, uint32_t* period);

vl53l0x_err_t vl53l0x_set_xtalk_comp(vl53l0x_handle_t dev, bool enable);
vl53l0x_err_t vl53l0x_get_xtalk_comp(vl53l0x_handle_t dev, bool* enable);
vl53l0x_err_t vl53l0x_set_xtalk_comp_rate_mcps(vl53l0x_handle_t dev, fp1616_t rate);
vl53l0x_err_t vl53l0x_get_xtalk_comp_rate_mcps(vl53l0x_handle_t dev, fp1616_t* rate);

vl53l0x_err_t vl53l0x_set_ref_calibration(vl53l0x_handle_t dev, uint8_t vhv_settings,
                                          uint8_t phase_calibration);
vl53l0x_err_t vl53l0x_get_ref_calibration(vl53l0x_handle_t dev, uint8_t* vhv_settings,
                                          uint8_t* phase_calibration);

vl53l0x_err_t vl53l0x_get_limit_checks_number(vl53l0x_handle_t dev, uint16_t* number);
vl53l0x_err_t vl53l0x_get_limit_check_info(vl53l0x_handle_t dev, vl53l0x_check_t check, char* info);
vl53l0x_err_t vl53l0x_get_limit_check_status(vl53l0x_handle_t dev, vl53l0x_check_t check,
                                             bool* failed);
vl53l0x_err_t vl53l0x_set_limit_check(vl53l0x_handle_t dev, vl53l0x_check_t check, bool enabled);
vl53l0x_err_t vl53l0x_get_limit_check(vl53l0x_handle_t dev, vl53l0x_check_t check, bool* enabled);
vl53l0x_err_t vl53l0x_set_limit_check_value(vl53l0x_handle_t dev, vl53l0x_check_t check,
                                            fp1616_t value);
vl53l0x_err_t vl53l0x_get_limit_check_value(vl53l0x_handle_t dev, vl53l0x_check_t check,
                                            fp1616_t* value);
vl53l0x_err_t vl53l0x_get_limit_check_current(vl53l0x_handle_t dev, vl53l0x_check_t check,
                                              fp1616_t* current);

vl53l0x_err_t vl53l0x_set_wrap_around_check(vl53l0x_handle_t dev, bool enabled);
vl53l0x_err_t vl53l0x_get_wrap_around_check(vl53l0x_handle_t dev, bool* enabled);

vl53l0x_err_t vl53l0x_set_dmax_cal_params(vl53l0x_handle_t dev, uint16_t range_mm,
                                          fp1616_t signal_rate_rtn_mcps);
vl53l0x_err_t vl53l0x_get_dmax_cal_params(vl53l0x_handle_t dev, uint16_t* range_mm,
                                          fp1616_t* signal_rate_rtn_mcps);

vl53l0x_err_t vl53l0x_perform_single_meas(vl53l0x_handle_t dev);
vl53l0x_err_t vl53l0x_perform_ref_calibration(vl53l0x_handle_t dev, uint8_t* vhv_settings,
                                              uint8_t* phase_cal);
vl53l0x_err_t vl53l0x_perform_xtalk_calibration(vl53l0x_handle_t dev, fp1616_t xtalk_cal_distance,
                                                fp1616_t* xtalk_compensation_rate_mcps);
vl53l0x_err_t vl53l0x_perform_offset_calibration(vl53l0x_handle_t dev, fp1616_t cal_dist_mm,
                                                 int32_t* offset_um);
vl53l0x_err_t vl53l0x_start_meas(vl53l0x_handle_t dev);
vl53l0x_err_t vl53l0x_stop_meas(vl53l0x_handle_t dev);

vl53l0x_err_t vl53l0x_get_meas_data_ready(vl53l0x_handle_t dev, bool* ready);

vl53l0x_err_t vl53l0x_get_meas_ref_signal(vl53l0x_handle_t dev, fp1616_t* signal);
vl53l0x_err_t vl53l0x_get_ranging_meas_data(vl53l0x_handle_t             dev,
                                            vl53l0x_ranging_meas_data_t* data);
vl53l0x_err_t vl53l0x_get_hist_meas_data(vl53l0x_handle_t dev, vl53l0x_hist_meas_data_t* data);

vl53l0x_err_t vl53l0x_perform_single_ranging_meas(vl53l0x_handle_t             dev,
                                                  vl53l0x_ranging_meas_data_t* data);
vl53l0x_err_t vl53l0x_perform_single_hist_meas(vl53l0x_handle_t          dev,
                                               vl53l0x_hist_meas_data_t* data);

vl53l0x_err_t vl53l0x_set_roi_zones_number(vl53l0x_handle_t dev, uint8_t number);
vl53l0x_err_t vl53l0x_get_roi_zones_number(vl53l0x_handle_t dev, uint8_t* number);
vl53l0x_err_t vl53l0x_get_max_roi_zones_number(vl53l0x_handle_t dev, uint8_t* number);

vl53l0x_err_t vl53l0x_set_gpio_config(vl53l0x_handle_t dev, uint8_t pin,
                                      vl53l0x_dev_mode_t dev_mode, vl53l0x_gpio_func_t func,
                                      vl53l0x_interrupt_polarity_t polarity);
vl53l0x_err_t vl53l0x_get_gpio_config(vl53l0x_handle_t dev, uint8_t pin,
                                      vl53l0x_dev_mode_t* dev_mode, vl53l0x_gpio_func_t* func,
                                      vl53l0x_interrupt_polarity_t* polarity);
vl53l0x_err_t vl53l0x_set_interrupt_threshold(vl53l0x_handle_t dev, vl53l0x_dev_mode_t mode,
                                              fp1616_t threshold_low, fp1616_t threshold_high);
vl53l0x_err_t vl53l0x_get_interrupt_threshold(vl53l0x_handle_t dev, vl53l0x_dev_mode_t mode,
                                              fp1616_t* threshold_low, fp1616_t* threshold_high);

vl53l0x_err_t vl53l0x_get_stop_completed_status(vl53l0x_handle_t dev, uint32_t* status);

vl53l0x_err_t vl53l0x_clear_interrupt_mask(vl53l0x_handle_t dev, uint32_t mask);
vl53l0x_err_t vl53l0x_get_interrupt_mask_status(vl53l0x_handle_t dev, uint32_t* status);
vl53l0x_err_t vl53l0x_enable_interrupt_mask(vl53l0x_handle_t dev, uint32_t mask);

vl53l0x_err_t vl53l0x_set_spad_ambient_damper_threshold(vl53l0x_handle_t dev, uint16_t threshold);
vl53l0x_err_t vl53l0x_get_spad_ambient_damper_threshold(vl53l0x_handle_t dev, uint16_t* threshold);
vl53l0x_err_t vl53l0x_set_spad_ambient_damper_factor(vl53l0x_handle_t dev, uint16_t factor);
vl53l0x_err_t vl53l0x_get_spad_ambient_damper_factor(vl53l0x_handle_t dev, uint16_t* factor);
vl53l0x_err_t vl53l0x_perform_ref_spad_management(vl53l0x_handle_t dev, uint32_t* count,
                                                  bool* aperture);
vl53l0x_err_t vl53l0x_set_ref_spads(vl53l0x_handle_t dev, uint32_t count, bool aperture);
vl53l0x_err_t vl53l0x_get_ref_spads(vl53l0x_handle_t dev, uint32_t* count, bool* aperture);

#endif
