#ifndef _VL53L0X_H
#define _VL53L0X_H

#include "esp_system.h"
#include "i2c_proto.h"

#define DEFAULT_I2C_ADDR 0x29

#define ERR_CHECK(x)                                                                               \
  do {                                                                                             \
    esp_err_t retval = (x);                                                                        \
    if (retval != ESP_OK)                                                                          \
      return retval;                                                                               \
  } while (0)

#define DECODE_VCSEL_PERIOD(x) (((x) + 1) << 1)
#define CALC_MACRO_PERIOD(period_pclks) ((((uint32_t)2304 * (period_pclks)*1655) + 500) / 1000)
#define DECODE_TIMEOUT(x) ((uint16_t)(((x)&0x00ff) << (uint16_t)(((x)&0xff00) >> 8)) + 1)
#define ENCODE_VCSEL_PERIOD(x) (((x) >> 1) - 1)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct vl53l0x *vl53l0x_handle_t;

typedef enum {
  SYSRANGE_START = 0x00,

  SYSTEM_THRESH_HIGH = 0x0C,
  SYSTEM_THRESH_LOW  = 0x0E,

  SYSTEM_SEQUENCE_CONFIG         = 0x01,
  SYSTEM_RANGE_CONFIG            = 0x09,
  SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

  SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

  GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

  SYSTEM_INTERRUPT_CLEAR = 0x0B,

  RESULT_INTERRUPT_STATUS = 0x13,
  RESULT_RANGE_STATUS     = 0x14,

  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN  = 0xC0,
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
  RESULT_CORE_RANGING_TOTAL_EVENTS_REF  = 0xD4,
  RESULT_PEAK_SIGNAL_RATE_REF           = 0xB6,

  ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

  I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

  MSRC_CONFIG_CONTROL = 0x60,

  PRE_RANGE_CONFIG_MIN_SNR           = 0x27,
  PRE_RANGE_CONFIG_VALID_PHASE_LOW   = 0x56,
  PRE_RANGE_CONFIG_VALID_PHASE_HIGH  = 0x57,
  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

  FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
  FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

  PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
  PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

  PRE_RANGE_CONFIG_VCSEL_PERIOD      = 0x50,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

  SYSTEM_HISTOGRAM_BIN                  = 0x81,
  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
  HISTOGRAM_CONFIG_READOUT_CTRL         = 0x55,

  FINAL_RANGE_CONFIG_VCSEL_PERIOD       = 0x70,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI  = 0x71,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO  = 0x72,
  CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

  MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

  SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
  IDENTIFICATION_MODEL_ID     = 0xC0,
  IDENTIFICATION_REVISION_ID  = 0xC2,

  OSC_CALIBRATE_VAL = 0xF8,

  GLOBAL_CONFIG_VCSEL_WIDTH        = 0x32,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

  GLOBAL_CONFIG_REF_EN_START_SELECT   = 0xB6,
  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
  DYNAMIC_SPAD_REF_EN_START_OFFSET    = 0x4F,
  POWER_MANAGEMENT_GO1_POWER_FORCE    = 0x80,

  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

  ALGO_PHASECAL_LIM            = 0x30,
  ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
} vl53l0x_reg_addr_t;

typedef struct {
  const char     addr;
  const char     port;
  const int      timeout;
  const uint16_t io_timeout_us;
} vl53l0x_config_t;

typedef struct {
  bool tcc, msrc, dss, pre_range, final_range;
} sequence_steps_t;

typedef struct {
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pckls;

  uint16_t msrc_dss_tcc_mckls, pre_range_mclks, final_range_mckls;
  uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} sequence_steps_timeouts_t;

typedef enum {
  VCSEL_PERIOD_PRE_RANGE,
  VCSEL_PERIOD_FINAL_RANGE,
} vcsel_period_t;

vl53l0x_handle_t vl53l0x_create(vl53l0x_config_t *config);

/**
 Init VL53L0X sensor

 @param vl53l0x vl53l0x handle
 @return result
 */
esp_err_t vl53l0x_init(vl53l0x_handle_t vl53l0x);
esp_err_t vl53l0x_set_signal_rate_limit(vl53l0x_handle_t vl53l0x, float limit);
esp_err_t vl53l0x_set_meas_timing_budget(vl53l0x_handle_t vl53l0x, uint32_t budget_us);
esp_err_t vl53l0x_set_vcsel_pulse_period(vl53l0x_handle_t vl53l0x, vcsel_period_t type,
                                         uint8_t period_pclks);
esp_err_t vl53l0x_read_range_continuous_mm(vl53l0x_handle_t vl53l0x, uint16_t *readout);
esp_err_t vl53l0x_read_range_single_mm(vl53l0x_handle_t vl53l0x, uint16_t *readout);

const uint8_t DEFAULT_TUNING[162];

#ifdef __cplusplus
}
#endif

#endif
