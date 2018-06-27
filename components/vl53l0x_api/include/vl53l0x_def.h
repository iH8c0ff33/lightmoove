#ifndef _VL53L0X_DEF_H
#define _VL53L0X_DEF_H

#define VL53L0X_MAX_STRING_LENGTH 32
#define VL53L0X_MAX_LOOPS 200

#include "esp_system.h"
#include "vl53l0x_device.h"
#include "vl53l0x_types.h"

typedef struct {
  char    name[VL53L0X_MAX_STRING_LENGTH];
  char    type[VL53L0X_MAX_STRING_LENGTH];
  uint8_t product_type;
  uint8_t product_rev_major;
  uint8_t product_rev_minor;
} vl53l0x_dev_info_t;

typedef enum {
  VL53L0X_OK                                   = 0,
  VL53L0X_ERR_CALIBRATION_WARNING              = -1,
  VL53L0X_ERR_MIN_CLIPPED                      = -2,
  VL53L0X_ERR_UNDEFINED                        = -3,
  VL53L0X_ERR_INVALID_PARAMS                   = -4,
  VL53L0X_ERR_NOT_SUPPORTED                    = -5,
  VL53L0X_ERR_RANGE_ERROR                      = -6,
  VL53L0X_ERR_TIMEOUT                          = -7,
  VL53L0X_ERR_MODE_NOT_SUPPORTED               = -8,
  VL53L0X_ERR_BUFFER_TOO_SMALL                 = -9,
  VL53L0X_ERR_GPIO_NOT_EXISTING                = -10,
  VL53L0X_ERR_GPIO_FUNCTIONALITY_NOT_SUPPORTED = -11,
  VL53L0X_ERR_INTERRUPT_NOT_CLEARED            = -12,
  VL53L0X_ERR_CONTROL_INTERFACE                = -20,
  VL53L0X_ERR_INVALID_COMMAND                  = -30,
  VL53L0X_ERR_DIVISION_BY_ZERO                 = -40,
  VL53L0X_ERR_REF_SPAD_INIT                    = -50,
  VL53L0X_ERR_NOT_IMPLEMENTED                  = -99,
} vl53l0x_err_t;

#define ERR_CHECK(x)                 \
  do {                               \
    vl53l0x_err_t st = (x);          \
    if (st != VL53L0X_OK) return st; \
  } while (0)

typedef enum {
  VL53L0X_DEVMODE_SINGLE_RANGING           = 0,
  VL53L0X_DEVMODE_CONTINUOUS_RANGING       = 1,
  VL53L0X_DEVMODE_SINGLE_HISTOGRAM         = 2,
  VL53L0X_DEVMODE_CONTINUOUS_TIMED_RANGING = 3,
  VL53L0X_DEVMODE_SINGLE_ALS               = 10,
  VL53L0X_DEVMODE_GPIO_DRIVE               = 20,
  VL53L0X_DEVMODE_GPIO_OSC                 = 21,
} vl53l0x_dev_mode_t;

typedef enum {
  VL53L0X_HISTMODE_DISABLED    = 0,
  VL53L0X_HISTMODE_REF_ONLY    = 1,
  VL53L0X_HISTMODE_RETURN_ONLY = 2,
  VL53L0X_HISTMODE_BOTH        = 3,
} vl53l0x_hist_mode_t;

typedef enum {
  VL53L0X_PWRMODE_STDBY_1 = 0,
  VL53L0X_PWRMODE_STDBY_2 = 1,
  VL53L0X_PWRMODE_IDLE_1  = 2,
  VL53L0X_PWRMODE_IDLE_2  = 3,
} vl53l0x_pwr_mode_t;

typedef struct {
  vl53l0x_dev_mode_t  dev_mode;
  vl53l0x_hist_mode_t hist_mode;

  uint32_t meas_timing_budget_us;
  uint32_t inter_meas_period_ms;
  bool     xtalk_compensation_enabled;
  uint16_t xtalk_compensation_range_mm;
  fp1616_t xtalk_compensation_rate_mcps;
  int32_t  range_offset_um;

  bool     limit_checks[VL53L0X_CHECKS_NUMBER];
  bool     limit_checks_status[VL53L0X_CHECKS_NUMBER];
  fp1616_t limit_checks_value[VL53L0X_CHECKS_NUMBER];
  bool     wrap_around_check_enable;
} vl53l0x_dev_params_t;

typedef enum {
  VL53L0X_STATE_PWRDOWN         = 0,
  VL53L0X_STATE_WAIT_STATICINIT = 1,
  VL53L0X_STATE_STDBY           = 2,
  VL53L0X_STATE_IDLE            = 3,
  VL53L0X_STATE_RUNNING         = 4,
  VL53L0X_STATE_UNKNOWN         = 98,
  VL53L0X_STATE_ERROR           = 99,
} vl53l0x_state_t;

typedef struct {
  int32_t amb_tuning_window_factor_K;
  int32_t ret_signal_at_0mm;
} vl53l0x_dmax_data_t;

typedef struct {
  uint32_t timestamp;
  uint32_t meas_time_us;

  uint16_t range_mm;

  uint16_t range_dmax_mm;

  fp1616_t signal_rate_rtn_mcps;
  fp1616_t ambient_rate_rtn_mcps;

  uint16_t effective_spad_rtn_count;
  // NOTE: divide by 256

  uint8_t range_fractional_part;
  uint8_t range_status;
} vl53l0x_ranging_meas_data_t;

#define VL53L0X_HIST_BUFFER_SIZE 24

typedef struct {
  uint32_t hist_data[VL53L0X_HIST_BUFFER_SIZE];
  uint8_t  hist_type;
  uint8_t  first_bin;
  uint8_t  buffer_size;
  uint8_t  bins_number;

  vl53l0x_err_t error_status;
} vl53l0x_hist_meas_data_t;

#define VL53L0X_REF_SPAD_BUFFER_SIZE 6

typedef struct {
  uint8_t ref_spad_enables[VL53L0X_REF_SPAD_BUFFER_SIZE];
  uint8_t ref_good_spad_map[VL53L0X_REF_SPAD_BUFFER_SIZE];
} vl53l0x_spad_data_t;

typedef struct {
  fp1616_t osc_freq_mhz;

  uint16_t last_encoded_timeout;

  vl53l0x_gpio_func_t pin_0_gpio_func;

  uint32_t final_range_timeout_us;
  uint8_t  final_range_vcsel_pulse_period;
  uint32_t pre_range_timeout_us;
  uint8_t  pre_range_vcsel_pulse_period;

  uint16_t sigma_est_ref_array;
  uint16_t sigma_est_eff_pulse_width;
  uint16_t sigma_est_eff_amb_width;

  bool     read_from_dev_done;
  uint8_t  module_id;
  uint8_t  revision;
  char     product_id[VL53L0X_MAX_STRING_LENGTH];
  uint8_t  ref_spad_count;
  bool     ref_spad_aperture;
  bool     ref_spad_initialised;
  uint32_t part_uid_lo;
  uint32_t part_uid_up;
  fp1616_t signal_rate_meas_fixed_400mm;
} vl53l0x_dev_spec_params_t;

typedef struct dev_data {
  vl53l0x_dmax_data_t         dmax_data;
  int32_t                     part2_part_offset_nvm_um;
  int32_t                     part2_part_offset_adj_nvm_um;
  vl53l0x_dev_params_t        current_params;
  vl53l0x_ranging_meas_data_t last_range_meas;
  vl53l0x_hist_meas_data_t    last_hist_meas;
  vl53l0x_dev_spec_params_t   dev_spec_params;
  vl53l0x_spad_data_t         spad_data;
  uint8_t                     seq_conf;
  bool                        range_fractional_enable;
  vl53l0x_state_t             pal_state;
  vl53l0x_pwr_mode_t          pwr_mode;
  uint16_t                    sigma_est_ref_array;
  uint16_t                    sigma_est_eff_pulse_width;
  uint16_t                    sigma_est_eff_amd_width;
  uint8_t                     stop_var;
  uint16_t                    target_ref_rate;
  fp1616_t                    sigma_est;
  fp1616_t                    last_signal_ref_mcps;
  uint8_t*                    tuning_settings;
  bool                        use_internal_tuning_settings;
  uint16_t                    lin_correct_gain;
  uint16_t                    dmax_cal_range_mm;
  fp1616_t                    dmax_cal_signal_rate_rtn_mcps;
} vl53l0x_dev_data_t;

typedef enum interrupt_polarity {
  VL53L0X_INTERRUPT_POLARITY_LOW  = 0,  // use for falling edge interrupt
  VL53L0X_INTERRUPT_POLARITY_HIGH = 1,  // use for rising edge interrupt
} vl53l0x_interrupt_polarity_t;

typedef enum {
  VL53L0X_VCSEL_PERIOD_PRE_RANGE,
  VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
} vl53l0x_vcsel_period_type_t;

typedef struct {
  bool tcc, msrc, dss, pre_range, final_range;
} vl53l0x_seq_steps_t;

typedef enum {
  VL53L0X_SEQSTEP_TCC         = 0,
  VL53L0X_SEQSTEP_DSS         = 1,
  VL53L0X_SEQSTEP_MSRC        = 2,
  VL53L0X_SEQSTEP_PRE_RANGE   = 3,
  VL53L0X_SEQSTEP_FINAL_RANGE = 4,
} vl53l0x_seq_step_t;

#define VL53L0X_SEQ_STEP_CHECKS_NUMBER 5

// MACROs
#define VL53L0X_FP1616_TO_FP97(x) (uint16_t)(((x) >> 9) & 0xffff)
#define VL53L0X_FP96_TO_FP1616(x) (fp1616_t)(Value << 9)

#define VL53L0X_FP1616_TO_FP88(x) (uint16_t)(((x) >> 8) & 0xFFFF)
#define VL53L0X_FP88_TO_FP1616(x) (fp1616_t)((x) << 8)

#define VL53L0X_FP1616_TO_FP412(x) (uint16_t)(((x) >> 4) & 0xFFFF)
#define VL53L0X_FP412_TO_FP1616(x) (fp1616_t)((x) << 4)

#define VL53L0X_FP1616_TO_FP313(x) (uint16_t)(((x) >> 3) & 0xFFFF)
#define VL53L0X_FP313_TO_FP1616(x) (fp1616_t)((x) << 3)

#define VL53L0X_FP1616_TO_FP08(x) (uint8_t)(((x) >> 8) & 0x00FF)
#define VL53L0X_FP08_TO_FP1616(x) (fp1616_t)((x) << 8)

#define VL53L0X_FP1616_TO_FP53(x) (uint8_t)(((x) >> 13) & 0x00FF)
#define VL53L0X_FP53_TO_FP1616(x) (fp1616_t)((x) << 13)

#define VL53L0X_FP1616_TO_FP102(x) (uint16_t)(((x) >> 14) & 0x0FFF)
#define VL53L0X_FP102_TO_FP1616(x) (fp1616_t)(Value << 12)

#endif
