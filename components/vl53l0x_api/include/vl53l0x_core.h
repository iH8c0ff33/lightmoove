#ifndef _VL53L0X_CORE_H
#define _VL53L0X_CORE_H

#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

vl53l0x_err_t vl53l0x_reverse_bytes(uint8_t* data, uint32_t size);
vl53l0x_err_t vl53l0x_meas_poll_for_completion(vl53l0x_handle_t dev);

uint8_t vl53l0x_encode_vcsel_period(uint8_t pclks);
uint8_t vl53l0x_decode_vcsel_period(uint8_t reg);

uint32_t vl53l0x_isqrt(uint32_t x);
uint32_t vl53l0x_quadrature_sum(uint32_t a, uint32_t b);

vl53l0x_err_t vl53l0x_get_info_from_dev(vl53l0x_handle_t dev, uint8_t option);

vl53l0x_err_t _vl53l0x_set_vcsel_pulse_period(vl53l0x_handle_t            dev,
                                              vl53l0x_vcsel_period_type_t type, uint8_t pclks);
vl53l0x_err_t _vl53l0x_get_vcsel_pulse_period(vl53l0x_handle_t            dev,
                                              vl53l0x_vcsel_period_type_t type, uint8_t* pclks);

uint32_t vl53l0x_decode_timeout(uint16_t encoded);
uint16_t vl53l0x_encode_timeout(uint32_t macro_clks);

vl53l0x_err_t _vl53l0x_get_seq_step_timeout(vl53l0x_handle_t dev, vl53l0x_seq_step_t step,
                                   uint32_t* timeout_us);
vl53l0x_err_t _vl53l0x_set_seq_step_timeout(vl53l0x_handle_t dev, vl53l0x_seq_step_t step,
                                   uint32_t timeout_us);

vl53l0x_err_t _vl53l0x_set_meas_timing_budget_us(vl53l0x_handle_t dev, uint32_t budget);
vl53l0x_err_t _vl53l0x_get_meas_timing_budget_us(vl53l0x_handle_t dev, uint32_t* budget);

vl53l0x_err_t vl53l0x_load_tuning_settings(vl53l0x_handle_t dev, const uint8_t* buffer);

vl53l0x_err_t vl53l0x_calc_sigma_estimate(vl53l0x_handle_t             dev,
                                          vl53l0x_ranging_meas_data_t* meas_data,
                                          fp1616_t* estimate, uint32_t* dmax_mm);
vl53l0x_err_t vl53l0x_get_total_xtalk_rate(vl53l0x_handle_t             dev,
                                           vl53l0x_ranging_meas_data_t* meas_data,
                                           fp1616_t*                    rate_mcps);
vl53l0x_err_t _vl53l0x_get_total_signal_rate(vl53l0x_handle_t             dev,
                                             vl53l0x_ranging_meas_data_t* meas_data,
                                             fp1616_t*                    rate_mcps);

vl53l0x_err_t vl53l0x_get_pal_range_status(vl53l0x_handle_t dev, uint8_t dev_range_status,
                                           fp1616_t signal_rate, uint16_t effective_spad_rtn_count,
                                           vl53l0x_ranging_meas_data_t* meas_data,
                                           uint8_t*                     pal_range_status);
uint32_t      vl53l0x_calc_timeout_mclks(vl53l0x_handle_t dev, uint32_t timeout_period_us,
                                         uint8_t vcsel_period_pckls);

#endif  //_VL53L0X_CORE_H
