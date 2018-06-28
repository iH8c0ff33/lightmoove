#include "vl53l0x_core.h"
#include "vl53l0x.h"
#include "vl53l0x_calibration.h"

vl53l0x_err_t vl53l0x_reverse_bytes(uint8_t* data, uint32_t size) {
  uint8_t  tmp;
  uint32_t mirror_index, middle = size / 2;

  for (uint32_t index = 0; index < middle; index++) {
    mirror_index       = size - index - 1;
    tmp                = data[index];
    data[index]        = data[mirror_index];
    data[mirror_index] = tmp;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_meas_poll_for_completion(vl53l0x_handle_t dev) {
  bool    ready;
  uint8_t loops;
  do {
    ERR_CHECK(vl53l0x_get_meas_data_ready(dev, &ready));
    ERR_CHECK(vl53l0x_polling_delay(dev));
    loops++;
  } while (!ready && loops < VL53L0X_MAX_LOOPS);
  if (loops >= VL53L0X_MAX_LOOPS) return VL53L0X_ERR_TIMEOUT;

  return VL53L0X_OK;
}

uint8_t vl53l0x_encode_vcsel_period(uint8_t pclks) { return (pclks >> 1) - 1; }

uint8_t vl53l0x_decode_vcsel_period(uint8_t reg) { return (reg + 1) << 1; }

uint32_t vl53l0x_isqrt(uint32_t x) {
  uint32_t res = 0;
  uint32_t bit = 1 << 30;
  // the second-to-top bit is set

  // "bit" starts at the highest power of four <= the argument
  while (bit > x) bit >>= 2;

  while (bit != 0) {
    if (x >= res + bit) {
      x -= res + bit;
      res = (res >> 1) + bit;
    } else
      res >>= 1;

    bit >>= 2;
  }

  return res;
}

uint32_t vl53l0x_quadrature_sum(uint32_t a, uint32_t b) {
  return a > 65535 || b > 65535 ? 65535 : vl53l0x_isqrt(a * a + b * b);
}

vl53l0x_err_t vl53l0x_device_read_strobe(vl53l0x_handle_t dev) {
  ERR_CHECK(vl53l0x_write_8(dev, 0x83, 0x00));

  // polling, use timeout to avoid deadlock
  uint8_t strobe, loops = 0;
  do {
    ERR_CHECK(vl53l0x_read_8(dev, 0x83, &strobe));
    loops++;
  } while (strobe == 0x00 && loops < VL53L0X_MAX_LOOPS);
  if (loops >= VL53L0X_MAX_LOOPS) return VL53L0X_ERR_TIMEOUT;

  ERR_CHECK(vl53l0x_write_8(dev, 0x83, 0x01));

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_get_info_from_dev(vl53l0x_handle_t dev, uint8_t option) {
  uint8_t  ref_spad_count;
  bool     ref_spad_type;
  uint8_t  nvm_ref_good_spad_map[VL53L0X_REF_SPAD_BUFFER_SIZE];
  uint8_t  module_id, revision;
  char     product_id[19];
  uint32_t part_id_upper, part_id_lower;
  uint32_t signal_rate_meas_fixed_1104_400_mm, dist_meas_fixed_1104_400_mm,
      dist_meas_tgt_fixed_1104_mm = 400 << 4;
  int16_t  offset_um;
  uint32_t offset_fixed_1104_mm = 0;

  if (dev->data.dev_spec_params.read_from_dev_done != 7) {
    ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x01));
    ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
    ERR_CHECK(vl53l0x_write_8(dev, 0x00, 0x00));

    ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x06));
    ERR_CHECK(vl53l0x_update_8(dev, 0x83, 0xff, 0x04));
    ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x07));
    ERR_CHECK(vl53l0x_write_8(dev, 0x81, 0x01));

    ERR_CHECK(vl53l0x_polling_delay(dev));

    ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x01));

    if ((option & 1) == 1 && (dev->data.dev_spec_params.read_from_dev_done & 1) == 0) {
      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x6b));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      uint32_t dword;
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      ref_spad_count = (dword >> 8) & 0x7f;
      ref_spad_type  = (dword >> 15) & 0x01;

      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x24));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      nvm_ref_good_spad_map[0] = (dword >> 24) & 0xff;
      nvm_ref_good_spad_map[1] = (dword >> 16) & 0xff;
      nvm_ref_good_spad_map[2] = (dword >> 8) & 0xff;
      nvm_ref_good_spad_map[3] = dword & 0xff;

      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x25));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      nvm_ref_good_spad_map[4] = (dword >> 24) & 0xff;
      nvm_ref_good_spad_map[5] = (dword >> 16) & 0xff;
    }

    if ((option & 2) == 2 && (dev->data.dev_spec_params.read_from_dev_done & 2) == 0) {
      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x02));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_8(dev, 0x90, &module_id));

      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x7b));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_8(dev, 0x90, &revision));

      uint32_t dword;
      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x77));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      product_id[0] = (dword >> 25) & 0x7f;
      product_id[1] = (dword >> 18) & 0x7f;
      product_id[2] = (dword >> 11) & 0x7f;
      product_id[3] = (dword >> 3) & 0x7f;

      uint8_t byte = (dword & 0x0f) << 3;

      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x78));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      product_id[4] = (byte + ((dword >> 29) & 0x7f));
      product_id[5] = (dword >> 22) & 0x7f;
      product_id[6] = (dword >> 15) & 0x7f;
      product_id[7] = (dword >> 8) & 0x7f;
      product_id[8] = (dword >> 1) & 0x7f;

      byte = (dword & 0x01) << 6;

      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x79));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      product_id[9]  = (byte + ((dword >> 26) & 0x7f));
      product_id[10] = (dword >> 19) & 0x7f;
      product_id[11] = (dword >> 12) & 0x7f;
      product_id[12] = (dword >> 5) & 0x7f;

      byte = (dword & 0x1f) << 2;

      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x7a));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      product_id[13] = (byte + ((dword >> 30) & 0x7f));
      product_id[14] = (dword >> 16) & 0x7f;
      product_id[14] = (dword >> 9) & 0x7f;
      product_id[14] = (dword >> 2) & 0x7f;
      product_id[18] = '\0';
    }

    if ((option & 4) == 4 && (dev->data.dev_spec_params.read_from_dev_done & 4) == 0) {
      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x7b));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &part_id_upper));

      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x7c));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &part_id_lower));

      uint32_t dword;
      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x73));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      signal_rate_meas_fixed_1104_400_mm = (dword & 0xff) << 8;

      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x74));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      signal_rate_meas_fixed_1104_400_mm |= (dword & 0xff000000) >> 24;

      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x75));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      dist_meas_fixed_1104_400_mm = (dword & 0xff) << 8;

      ERR_CHECK(vl53l0x_write_8(dev, 0x94, 0x76));
      ERR_CHECK(vl53l0x_device_read_strobe(dev));
      ERR_CHECK(vl53l0x_read_32(dev, 0x90, &dword));

      dist_meas_fixed_1104_400_mm |= (dword & 0xff000000) >> 24;
    }

    ERR_CHECK(vl53l0x_write_8(dev, 0x81, 0x00));
    ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x06));
    ERR_CHECK(vl53l0x_update_8(dev, 0x83, 0xfb, 0x00));
    ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
    ERR_CHECK(vl53l0x_write_8(dev, 0x00, 0x01));

    ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
    ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x00));
  }

  if (dev->data.dev_spec_params.read_from_dev_done != 7) {
    // assign to variables
    if ((option & 1) == 1 && (dev->data.dev_spec_params.read_from_dev_done & 1) == 0) {
      dev->data.dev_spec_params.ref_spad_count    = ref_spad_count;
      dev->data.dev_spec_params.ref_spad_aperture = ref_spad_type;

      for (uint8_t i = 0; i < VL53L0X_REF_SPAD_BUFFER_SIZE; i++) {
        dev->data.spad_data.ref_good_spad_map[i] = nvm_ref_good_spad_map[i];
      }
    }

    if ((option & 2) == 2 && (dev->data.dev_spec_params.read_from_dev_done & 2) == 0) {
      dev->data.dev_spec_params.module_id = module_id;
      dev->data.dev_spec_params.revision  = revision;
      VL53L0X_COPYSTRING(dev->data.dev_spec_params.product_id, product_id);
    }

    if ((option & 4) == 4 && (dev->data.dev_spec_params.read_from_dev_done & 4) == 0) {
      dev->data.dev_spec_params.part_uid_up = part_id_upper;
      dev->data.dev_spec_params.part_uid_lo = part_id_lower;

      dev->data.dev_spec_params.signal_rate_meas_fixed_400mm =
          VL53L0X_FP97_TO_FP1616(signal_rate_meas_fixed_1104_400_mm);

      offset_um = 0;
      if (dist_meas_fixed_1104_400_mm != 0) {
        offset_fixed_1104_mm = dist_meas_fixed_1104_400_mm - dist_meas_tgt_fixed_1104_mm;
        offset_um            = (offset_fixed_1104_mm * 1000) >> 4;
        offset_um *= -1;
      }

      dev->data.part2_part_offset_adj_nvm_um = offset_um;
    }

    dev->data.dev_spec_params.read_from_dev_done |= option;
  }

  return VL53L0X_OK;
}

uint32_t calc_macro_period_ps(uint8_t vcsel_period_pclks) {
  uint64_t pll_period_ps      = 1655;
  uint32_t macro_period_vclks = 2304;

  return macro_period_vclks * vcsel_period_pclks * pll_period_ps;
}

uint16_t vl53l0x_encode_timeout(uint32_t macro_clks) {
  uint16_t msb = 0;
  uint32_t lsb = 0;

  if (macro_clks > 0) {
    lsb = macro_clks - 1;

    while (lsb & 0xffffff00 > 0) {
      lsb >>= 1;
      msb++;
    }
  }

  return (msb << 8) + (uint16_t)(lsb & 0xff);
}

uint32_t vl53l0x_decode_timeout(uint16_t encoded) {
  return ((uint32_t)(encoded & 0xff) << (uint32_t)((encoded & 0xff00) >> 8)) + 1;
}

uint32_t calc_timeout_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ps = calc_macro_period_ps(vcsel_period_pclks);
  macro_period_ps          = (macro_period_ps + 500) / 1000;

  return (uint32_t)(((timeout_period_us * 1000) + (macro_period_ps / 2)) / macro_period_ps);
}

uint32_t calc_timeout_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pckls) {
  uint32_t macro_period_ps = calc_macro_period_ps(vcsel_period_pckls);
  macro_period_ps          = (macro_period_ps + 500) / 1000;

  return ((timeout_period_mclks * macro_period_ps) + (macro_period_ps / 2)) / 1000;
}

vl53l0x_err_t _vl53l0x_get_seq_step_timeout(vl53l0x_handle_t dev, vl53l0x_seq_step_t step,
                                            uint32_t* timeout_us) {
  if (step >= VL53L0X_SEQ_STEP_CHECKS_NUMBER) return VL53L0X_ERR_INVALID_PARAMS;

  uint8_t  vcsel_pulse_period_pclk;
  uint16_t msrc_timeout_mclks, pre_range_timeout_mclks, final_range_timeout_mclks;

  vl53l0x_seq_steps_t seq_steps;

  uint8_t  byte;
  uint16_t word;

  switch (step) {
    case VL53L0X_SEQSTEP_TCC:
    case VL53L0X_SEQSTEP_DSS:
    case VL53L0X_SEQSTEP_MSRC:
      ERR_CHECK(vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                               &vcsel_pulse_period_pclk));
      ERR_CHECK(vl53l0x_read_8(dev, MSRC_CONFIG_TIMEOUT_MACROP, &byte));
      msrc_timeout_mclks = vl53l0x_decode_timeout(byte);

      *timeout_us = calc_timeout_us(msrc_timeout_mclks, vcsel_pulse_period_pclk);
      break;
    case VL53L0X_SEQSTEP_PRE_RANGE:
      // retrieve PRE-RANGE VCSEL period
      ERR_CHECK(vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                               &vcsel_pulse_period_pclk));

      // retrieve PRE-RANGE timeout in macro periods
      ERR_CHECK(vl53l0x_read_16(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &word));
      pre_range_timeout_mclks = vl53l0x_decode_timeout(word);

      *timeout_us = calc_timeout_us(pre_range_timeout_mclks, vcsel_pulse_period_pclk);
      break;
    case VL53L0X_SEQSTEP_FINAL_RANGE:
      ERR_CHECK(vl53l0x_get_seq_steps(dev, &seq_steps));

      // substract 0 in case pre_range is not enabled
      pre_range_timeout_mclks = 0;

      if (seq_steps.pre_range) {
        // TODO: is this needed?
        // retrieve PRE-RANGE VCSEL period
        ERR_CHECK(vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                                 &vcsel_pulse_period_pclk));

        // retrieve PRE-RANGE timeout in macro periods
        ERR_CHECK(vl53l0x_read_16(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &word));
        pre_range_timeout_mclks = vl53l0x_decode_timeout(word);
      }

      // retrieve FINAL-RANGE VCSEL period
      ERR_CHECK(vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                                               &vcsel_pulse_period_pclk));

      // retrieve FINAL-RANGE timeout in macro periods
      ERR_CHECK(vl53l0x_read_16(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &word));
      final_range_timeout_mclks = vl53l0x_decode_timeout(word);

      final_range_timeout_mclks -= pre_range_timeout_mclks;
      *timeout_us = calc_timeout_us(final_range_timeout_mclks, vcsel_pulse_period_pclk);
      break;
    default:
      return VL53L0X_ERR_INVALID_PARAMS;
  }

  return VL53L0X_OK;
}