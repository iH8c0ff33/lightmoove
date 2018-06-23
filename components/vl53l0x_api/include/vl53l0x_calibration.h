#ifndef _VL53L0X_CALIBRATION_H
#define _VL53L0X_CALIBRATION_H

#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

vl53l0x_err_t _vl53l0x_perform_xtalk_calibration(vl53l0x_handle_t dev, fp1616_t xtalk_cal_distance,
                                                 fp1616_t *xtalk_compensation_rate_mcps);
vl53l0x_err_t _vl53l0x_perform_offset_calibration(vl53l0x_handle_t dev, fp1616_t cal_distance_mm,
                                                  int32_t *offset_um);

vl53l0x_err_t _vl53l0x_set_offset_calibration_data_um(vl53l0x_handle_t dev, int32_t data);
vl53l0x_err_t _vl53l0x_get_offset_calibration_data_um(vl53l0x_handle_t dev, int32_t *data);

vl53l0x_err_t _vl53l0x_apply_offset_adjustment(vl53l0x_handle_t dev);

vl53l0x_err_t _vl53l0x_perform_ref_spad_management(vl53l0x_handle_t dev, uint32_t *ref_spad_count,
                                                   bool *aperture);

vl53l0x_err_t _vl53l0x_set_ref_spads(vl53l0x_handle_t dev, uint32_t count, bool aperture);
vl53l0x_err_t _vl53l0x_get_ref_spads(vl53l0x_handle_t dev, uint32_t *count, bool *aperture);

vl53l0x_err_t _vl53l0x_perform_phase_calibration(vl53l0x_handle_t dev, uint8_t *phase_cal,
                                                 const bool get_data_enable,
                                                 const bool restore_config);

vl53l0x_err_t _vl53l0x_perform_ref_calibration(vl53l0x_handle_t dev, uint8_t *vhv_settings,
                                               uint8_t *phase_cal, uint8_t get_data_enable);

vl53l0x_err_t _vl53l0x_set_ref_calibration(vl53l0x_handle_t dev, uint8_t vhv_settings,
                                           uint8_t phase_cal);
vl53l0x_err_t _vl53l0x_get_ref_calibration(vl53l0x_handle_t dev, uint8_t *vhv_settings,
                                           uint8_t *phase_cal);

#endif  // _VL53L0X_CALIBRATION_H
