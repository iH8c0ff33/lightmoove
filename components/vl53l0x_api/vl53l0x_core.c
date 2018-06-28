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
  uint8_t loops = 0;
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

  // if all three data has been read skip everything
  if (dev->data.dev_spec_params.read_from_dev_done != 0b111) {
    // this has to run for every data readout
    ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x01));
    ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
    ERR_CHECK(vl53l0x_write_8(dev, 0x00, 0x00));

    ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x06));
    ERR_CHECK(vl53l0x_update_8(dev, 0x83, 0xff, 0x04));
    ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x07));
    ERR_CHECK(vl53l0x_write_8(dev, 0x81, 0x01));

    ERR_CHECK(vl53l0x_polling_delay(dev));

    ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x01));

    // if option has set 0b001 and it has not been read already
    if ((dev->data.dev_spec_params.read_from_dev_done & 0b001) &&  //
        (option & 0b001)) {
      // begin read data
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
      // end read data

      // begin assign data
      dev->data.dev_spec_params.ref_spad_count    = ref_spad_count;
      dev->data.dev_spec_params.ref_spad_aperture = ref_spad_type;

      for (uint8_t i = 0; i < VL53L0X_REF_SPAD_BUFFER_SIZE; i++) {
        dev->data.spad_data.ref_good_spad_map[i] = nvm_ref_good_spad_map[i];
      }
      // end assign data
    }

    // if option has set 0b010 and it has not been read already
    if ((dev->data.dev_spec_params.read_from_dev_done & 0b010) &&  //
        (option & 0b010)) {
      // begin read data
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
      // end read data

      // begin assign data
      dev->data.dev_spec_params.module_id = module_id;
      dev->data.dev_spec_params.revision  = revision;
      VL53L0X_COPYSTRING(dev->data.dev_spec_params.product_id, product_id, 19);
      // end assign data
    }

    // if option has set 0b100 and is had not been read already
    if ((dev->data.dev_spec_params.read_from_dev_done & 0b100) &&  //
        (option & 0b100)) {
      // begin read data
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
      // end read data

      // begin assign data
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
      // end assign data
    }
  }

  // check new read bits in read_done
  dev->data.dev_spec_params.read_from_dev_done |= option;

  // this had to be done after every data readout
  ERR_CHECK(vl53l0x_write_8(dev, 0x81, 0x00));
  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x06));
  ERR_CHECK(vl53l0x_update_8(dev, 0x83, 0xfb, 0x00));
  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
  ERR_CHECK(vl53l0x_write_8(dev, 0x00, 0x01));

  ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
  ERR_CHECK(vl53l0x_write_8(dev, 0x80, 0x00));

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

    while ((lsb & 0xffffff00) > 0) {
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

vl53l0x_err_t _vl53l0x_set_seq_step_timeout(vl53l0x_handle_t dev, vl53l0x_seq_step_t step,
                                            uint32_t timeout_us) {
  if (step >= VL53L0X_SEQ_STEP_CHECKS_NUMBER) return VL53L0X_ERR_INVALID_PARAMS;

  uint8_t  vcsel_pulse_period_pclk;
  uint16_t msrc_timeout_mclks, pre_range_timeout_mclks, final_range_timeout_mclks;

  vl53l0x_seq_steps_t seq_steps;

  uint16_t word;

  uint8_t  msrc_encoded_timeout;
  uint16_t encoded_timeout;

  switch (step) {
    case VL53L0X_SEQSTEP_TCC:
    case VL53L0X_SEQSTEP_DSS:
    case VL53L0X_SEQSTEP_MSRC:
      // retrieve PRE-RANGE VCSEL period
      ERR_CHECK(vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                               &vcsel_pulse_period_pclk));
      msrc_timeout_mclks = calc_timeout_mclks(timeout_us, vcsel_pulse_period_pclk);

      msrc_encoded_timeout = msrc_timeout_mclks > 256 ? 255 : msrc_timeout_mclks - 1;
      dev->data.dev_spec_params.last_encoded_timeout = msrc_encoded_timeout;
      ERR_CHECK(vl53l0x_write_8(dev, MSRC_CONFIG_TIMEOUT_MACROP, msrc_encoded_timeout));
      break;
    case VL53L0X_SEQSTEP_PRE_RANGE:
      // retrieve PRE-RANGE VCSEL period
      ERR_CHECK(vl53l0x_get_vcsel_pulse_period(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                               &vcsel_pulse_period_pclk));
      pre_range_timeout_mclks = calc_timeout_mclks(timeout_us, vcsel_pulse_period_pclk);

      encoded_timeout = vl53l0x_encode_timeout(pre_range_timeout_mclks);
      dev->data.dev_spec_params.last_encoded_timeout = encoded_timeout;
      ERR_CHECK(vl53l0x_write_16(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encoded_timeout));
      dev->data.dev_spec_params.pre_range_timeout_us = timeout_us;
      break;
    case VL53L0X_SEQSTEP_FINAL_RANGE:
      /* For the final range timeout, the pre-range timeout
       * must be added. To do this both final and pre-range
       * timeouts must be expressed in macro periods MClks
       * because they have different vcsel periods. */
      ERR_CHECK(vl53l0x_get_seq_steps(dev, &seq_steps));
      // add 0 in case pre_range is not enabled
      pre_range_timeout_mclks = 0;

      if (seq_steps.pre_range) {
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

      // calculate FINAL-RANGE timeout in macro periods
      final_range_timeout_mclks = calc_timeout_mclks(timeout_us, vcsel_pulse_period_pclk);
      final_range_timeout_mclks += pre_range_timeout_mclks;

      encoded_timeout = vl53l0x_encode_timeout(final_range_timeout_mclks);
      dev->data.dev_spec_params.last_encoded_timeout = encoded_timeout;
      ERR_CHECK(vl53l0x_write_16(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encoded_timeout));
      dev->data.dev_spec_params.final_range_timeout_us = timeout_us;
      break;
    default:
      return VL53L0X_ERR_INVALID_PARAMS;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t _vl53l0x_set_vcsel_pulse_period(vl53l0x_handle_t            dev,
                                              vl53l0x_vcsel_period_type_t type, uint8_t pclks) {
  uint8_t min_pre_vcsel_period_pclk = 12, max_pre_vcsel_period_pclk = 18;
  uint8_t min_final_vcsel_period_pclk = 8, max_final_vcsel_period_pclk = 14;

  if ((pclks % 2) != 0)  // WARN: must be even
    return VL53L0X_ERR_INVALID_PARAMS;
  if (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE &&
      (pclks < min_pre_vcsel_period_pclk || pclks > max_pre_vcsel_period_pclk))
    return VL53L0X_ERR_INVALID_PARAMS;
  if (type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE &&
      (pclks < min_final_vcsel_period_pclk || pclks > max_final_vcsel_period_pclk))
    return VL53L0X_ERR_INVALID_PARAMS;

  if (type == VL53L0X_VCSEL_PERIOD_PRE_RANGE) {
    switch (pclks) {
      case 12:
        ERR_CHECK(vl53l0x_write_8(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18));
        ERR_CHECK(vl53l0x_write_8(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
        break;
      case 14:
        ERR_CHECK(vl53l0x_write_8(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30));
        ERR_CHECK(vl53l0x_write_8(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
        break;
      case 16:
        ERR_CHECK(vl53l0x_write_8(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40));
        ERR_CHECK(vl53l0x_write_8(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
        break;
      case 18:
        ERR_CHECK(vl53l0x_write_8(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50));
        ERR_CHECK(vl53l0x_write_8(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
        break;
      default:
        return VL53L0X_ERR_INVALID_PARAMS;
    }
  } else if (type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE) {
    switch (pclks) {
      case 8:
        ERR_CHECK(vl53l0x_write_8(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10));
        ERR_CHECK(vl53l0x_write_8(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));

        ERR_CHECK(vl53l0x_write_8(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02));
        ERR_CHECK(vl53l0x_write_8(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0c));

        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
        ERR_CHECK(vl53l0x_write_8(dev, ALGO_PHASECAL_LIM, 0x30));
        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
        break;
      case 10:
        ERR_CHECK(vl53l0x_write_8(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28));
        ERR_CHECK(vl53l0x_write_8(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));

        ERR_CHECK(vl53l0x_write_8(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03));
        ERR_CHECK(vl53l0x_write_8(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09));

        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
        ERR_CHECK(vl53l0x_write_8(dev, ALGO_PHASECAL_LIM, 0x20));
        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
        break;
      case 12:
        ERR_CHECK(vl53l0x_write_8(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38));
        ERR_CHECK(vl53l0x_write_8(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));

        ERR_CHECK(vl53l0x_write_8(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03));
        ERR_CHECK(vl53l0x_write_8(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08));

        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
        ERR_CHECK(vl53l0x_write_8(dev, ALGO_PHASECAL_LIM, 0x20));
        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
        break;
      case 14:
        ERR_CHECK(vl53l0x_write_8(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48));
        ERR_CHECK(vl53l0x_write_8(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));

        ERR_CHECK(vl53l0x_write_8(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03));
        ERR_CHECK(vl53l0x_write_8(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07));

        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x01));
        ERR_CHECK(vl53l0x_write_8(dev, ALGO_PHASECAL_LIM, 0x20));
        ERR_CHECK(vl53l0x_write_8(dev, 0xff, 0x00));
        break;
    }
  }

  // re-calculate and apply timeouts in macro periods
  uint8_t  encoded_vcsel_period = vl53l0x_encode_vcsel_period(pclks);
  uint32_t pre_range_timeout_us, final_range_timeout_us, msrc_timeout_us;
  switch (type) {
    case VL53L0X_VCSEL_PERIOD_PRE_RANGE:
      // get current timeouts
      ERR_CHECK(
          _vl53l0x_get_seq_step_timeout(dev, VL53L0X_SEQSTEP_PRE_RANGE, &pre_range_timeout_us));
      ERR_CHECK(_vl53l0x_get_seq_step_timeout(dev, VL53L0X_SEQSTEP_MSRC, &msrc_timeout_us));

      ERR_CHECK(vl53l0x_write_8(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD, encoded_vcsel_period));

      // re-calculate
      ERR_CHECK(
          _vl53l0x_set_seq_step_timeout(dev, VL53L0X_SEQSTEP_PRE_RANGE, pre_range_timeout_us));
      ERR_CHECK(_vl53l0x_set_seq_step_timeout(dev, VL53L0X_SEQSTEP_MSRC, msrc_timeout_us));

      dev->data.dev_spec_params.pre_range_vcsel_pulse_period = pclks;
      break;
    case VL53L0X_VCSEL_PERIOD_FINAL_RANGE:
      // get current timeout
      ERR_CHECK(
          _vl53l0x_get_seq_step_timeout(dev, VL53L0X_SEQSTEP_FINAL_RANGE, &final_range_timeout_us));

      ERR_CHECK(vl53l0x_write_8(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD, encoded_vcsel_period));

      // re-calculate
      ERR_CHECK(
          _vl53l0x_set_seq_step_timeout(dev, VL53L0X_SEQSTEP_FINAL_RANGE, final_range_timeout_us));

      dev->data.dev_spec_params.final_range_vcsel_pulse_period = pclks;
      break;
    default:
      return VL53L0X_ERR_INVALID_PARAMS;
  }

  // re-apply timing budget
  ERR_CHECK(vl53l0x_set_meas_timing_budget_us(dev, dev->data.current_params.meas_timing_budget_us));

  /* perform new phase calibration
   * get_data = false, restore_config = true */
  uint8_t phase_cal = 0;
  ERR_CHECK(_vl53l0x_perform_phase_calibration(dev, &phase_cal, false, true));

  return VL53L0X_OK;
}

vl53l0x_err_t _vl53l0x_get_vcsel_pulse_period(vl53l0x_handle_t            dev,
                                              vl53l0x_vcsel_period_type_t type, uint8_t* pclks) {
  uint8_t encoded_vcsel_period;

  if (type != VL53L0X_VCSEL_PERIOD_PRE_RANGE && type != VL53L0X_VCSEL_PERIOD_FINAL_RANGE)
    return VL53L0X_ERR_INVALID_PARAMS;

  ERR_CHECK(vl53l0x_read_8(dev,
                           type == VL53L0X_VCSEL_PERIOD_PRE_RANGE  //
                               ? PRE_RANGE_CONFIG_VCSEL_PERIOD
                               : FINAL_RANGE_CONFIG_VCSEL_PERIOD,
                           &encoded_vcsel_period));

  *pclks = vl53l0x_decode_vcsel_period(encoded_vcsel_period);

  return VL53L0X_OK;
}

vl53l0x_err_t _vl53l0x_set_meas_timing_budget_us(vl53l0x_handle_t dev, uint32_t budget) {
  uint32_t final_range_timing_budget_us;

  uint32_t msrc_dcc_tcc_timeout_us = 2000;
  uint32_t start_overhead_us       = 1320;
  uint32_t end_overhead_us         = 960;
  uint32_t msrc_overhead_us        = 660;
  uint32_t tcc_overhead_us         = 590;
  uint32_t dss_overhead_us         = 690;
  uint32_t pre_range_overhead_us   = 660;
  uint32_t final_range_overhead_us = 550;
  uint32_t pre_range_timeout_us    = 0;

  uint32_t min_timing_budget_us = 20000;
  uint32_t sub_timeout          = 0;

  if (budget < min_timing_budget_us) return VL53L0X_ERR_INVALID_PARAMS;

  // substract mandatory phases overhead
  final_range_timing_budget_us = budget - (start_overhead_us + end_overhead_us);

  // get enabled steps
  vl53l0x_seq_steps_t steps;
  ERR_CHECK(vl53l0x_get_seq_steps(dev, &steps));

  if (steps.tcc || steps.msrc || steps.dss) {
    // TCC, MSRC and DSS use the same timeout
    ERR_CHECK(_vl53l0x_get_seq_step_timeout(dev, VL53L0X_SEQSTEP_MSRC, &msrc_dcc_tcc_timeout_us));

    if (steps.tcc) {
      sub_timeout = msrc_dcc_tcc_timeout_us + tcc_overhead_us;

      // requested timeout too big
      if (sub_timeout >= final_range_timing_budget_us) return VL53L0X_ERR_INVALID_PARAMS;
      final_range_timing_budget_us -= sub_timeout;
    }

    if (steps.dss) {
      sub_timeout = 2 * (msrc_dcc_tcc_timeout_us + dss_overhead_us);

      if (sub_timeout >= final_range_timing_budget_us) return VL53L0X_ERR_INVALID_PARAMS;
      final_range_timing_budget_us -= sub_timeout;
    } else if (steps.msrc) {
      sub_timeout = msrc_dcc_tcc_timeout_us + msrc_overhead_us;

      if (sub_timeout >= final_range_timing_budget_us) return VL53L0X_ERR_INVALID_PARAMS;
      final_range_timing_budget_us -= sub_timeout;
    }
  }

  if (steps.pre_range) {
    ERR_CHECK(_vl53l0x_get_seq_step_timeout(dev, VL53L0X_SEQSTEP_PRE_RANGE, &pre_range_timeout_us));

    sub_timeout = pre_range_timeout_us + pre_range_overhead_us;

    if (sub_timeout >= final_range_timing_budget_us) return VL53L0X_ERR_INVALID_PARAMS;
    final_range_timing_budget_us -= sub_timeout;
  }

  if (steps.final_range) {
    /* NOTE: the final range timeout is determined now as the remaining time
     * after all the other timeouts/overheads.
     * WARN: If there is no room for final range timeout and error will be
     * thrown, otherwise the remaining time will be used as final range
     * timeout. */
    final_range_timing_budget_us -= final_range_overhead_us;

    ERR_CHECK(_vl53l0x_set_seq_step_timeout(dev, VL53L0X_SEQSTEP_FINAL_RANGE,
                                            final_range_timing_budget_us));
    dev->data.current_params.meas_timing_budget_us = budget;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t _vl53l0x_get_meas_timing_budget_us(vl53l0x_handle_t dev, uint32_t* budget) {
  uint32_t msrc_dcc_tcc_timeout_us = 2000;
  uint32_t start_overhead_us       = 1910;
  uint32_t end_overhead_us         = 960;
  uint32_t msrc_overhead_us        = 660;
  uint32_t tcc_overhead_us         = 590;
  uint32_t dss_overhead_us         = 690;
  uint32_t pre_range_overhead_us   = 660;
  uint32_t final_range_overhead_us = 550;

  uint32_t final_range_timeout_us;
  uint32_t pre_range_timeout_us;

  // start and end overhead are always present
  *budget = start_overhead_us + end_overhead_us;

  vl53l0x_seq_steps_t steps;
  ERR_CHECK(vl53l0x_get_seq_steps(dev, &steps));

  if (steps.tcc || steps.msrc || steps.dss) {
    ERR_CHECK(_vl53l0x_get_seq_step_timeout(dev, VL53L0X_SEQSTEP_MSRC, &msrc_dcc_tcc_timeout_us));

    if (steps.tcc) *budget += msrc_dcc_tcc_timeout_us + tcc_overhead_us;

    if (steps.dss)
      *budget += 2 * (msrc_dcc_tcc_timeout_us + dss_overhead_us);
    else if (steps.msrc)
      *budget += msrc_dcc_tcc_timeout_us + msrc_overhead_us;
  }

  if (steps.pre_range) {
    ERR_CHECK(_vl53l0x_get_seq_step_timeout(dev, VL53L0X_SEQSTEP_PRE_RANGE, &pre_range_timeout_us));

    *budget += pre_range_timeout_us + pre_range_overhead_us;
  }

  if (steps.final_range) {
    ERR_CHECK(
        _vl53l0x_get_seq_step_timeout(dev, VL53L0X_SEQSTEP_FINAL_RANGE, &final_range_timeout_us));

    *budget += final_range_timeout_us + final_range_overhead_us;
  }

  dev->data.current_params.meas_timing_budget_us = *budget;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_load_tuning_settings(vl53l0x_handle_t dev, const uint8_t* buffer) {
  size_t i = 0;
  while (buffer[i] != 0xff && buffer[i + 1] != 0xff) {
    ERR_CHECK(vl53l0x_write_8(dev, buffer[i], buffer[i + 1]));
    i += 2;
  }

  return VL53L0X_OK;
}

vl53l0x_err_t _vl53l0x_get_total_xtalk_rate(vl53l0x_handle_t             dev,
                                            vl53l0x_ranging_meas_data_t* meas_data,
                                            fp1616_t*                    rate_mcps) {
  bool enabled;
  ERR_CHECK(vl53l0x_get_xtalk_comp(dev, &enabled));
  if (enabled) {
    // fp1616 * fp8:8 = fp0824
    fp1616_t total_xtalk_mcps =
        meas_data->effective_spad_rtn_count * dev->data.current_params.xtalk_compensation_rate_mcps;

    // fp0824 >> 8 = fp1616
    *rate_mcps = (total_xtalk_mcps + 0x80) >> 8;
  } else
    *rate_mcps = 0;

  return VL53L0X_OK;
}

vl53l0x_err_t _vl53l0x_get_total_signal_rate(vl53l0x_handle_t             dev,
                                             vl53l0x_ranging_meas_data_t* meas_data,
                                             fp1616_t*                    rate_mcps) {
  *rate_mcps = meas_data->signal_rate_rtn_mcps;

  fp1616_t total_xtalk_mcps;
  ERR_CHECK(_vl53l0x_get_total_xtalk_rate(dev, meas_data, &total_xtalk_mcps));

  *rate_mcps += total_xtalk_mcps;

  return VL53L0X_OK;
}

vl53l0x_err_t _vl53l0x_calc_dmax(vl53l0x_handle_t dev,  //
                                 fp1616_t         total_signal_rate_mcps,
                                 fp1616_t total_corr_signal_rate_mcps, fp1616_t pw_mult,
                                 uint32_t sigma_est_p1, fp1616_t sigma_est_p2,
                                 uint32_t peak_vcsel_duration_us, uint32_t* dmax_mm) {
  const uint32_t SIGMA_LIMIT                = 18;
  const fp1616_t SIGNAL_LIMIT               = 0x4000;      // 0x25
  const fp1616_t SIGMA_EST_REF              = 0x00000042;  // 0.001
  const uint32_t AMB_EFF_WIDTH_SIGMA_EST_NS = 6;
  const uint32_t AMB_EFF_WIDTH_DMAX_NS      = 7;

  fp1616_t min_signal_needed;
  fp1616_t min_signal_needed_p1;
  fp1616_t min_signal_needed_p2;
  fp1616_t min_signal_needed_p3;
  fp1616_t min_signal_needed_p4;
  fp1616_t sigma_limit_tmp;
  fp1616_t sigma_est_sq_tmp;
  fp1616_t signal_limit_tmp;
  fp1616_t signal_at_0_mm;
  fp1616_t dmax_dark;
  fp1616_t dmax_ambient;
  fp1616_t dmax_dark_tmp;
  fp1616_t sigma_est_p2_tmp;

  // u32 * fp1616 = fp1616
  signal_at_0_mm = dev->data.dmax_cal_range_mm * dev->data.dmax_cal_signal_rate_rtn_mcps;

  // fp1616 >> 8 = fp2408
  signal_at_0_mm = (signal_at_0_mm + 0x80) >> 8;
  signal_at_0_mm *= dev->data.dmax_cal_range_mm;

  min_signal_needed_p1 = 0;
  if (total_corr_signal_rate_mcps > 0) {
    // shift by 10 bits to increase resolution before division
    uint32_t signal_rate_tmp_mcps = total_signal_rate_mcps << 10;

    // add rounding value prior to division
    min_signal_needed_p1 = signal_rate_tmp_mcps + (total_corr_signal_rate_mcps / 2);

    // fp0626/fp1616 = fp2210;
    min_signal_needed_p1 /= total_corr_signal_rate_mcps;

    // apply factored version of speed of light (correction at the end)
    min_signal_needed_p1 *= 3;

    // fp2210 * fp2210 = fp1220
    min_signal_needed_p1 *= min_signal_needed_p1;

    // fp1220 >> 16 = fp2804
    min_signal_needed_p1 = (min_signal_needed_p1 + 0x8000) >> 16;
  }

  min_signal_needed_p2 = pw_mult * sigma_est_p1;

  // fp1616 >> 16 = u32
  min_signal_needed_p2 = (min_signal_needed_p2 + 0x8000) >> 16;

  // u32*u32 = u32
  min_signal_needed_p2 *= min_signal_needed_p2;

  /* Check sigmaEstimateP2
   * If this value is too high there is not enough signal rate
   * to calculate dmax value so set a suitable value to ensure
   * a very small dmax.
   */
  sigma_est_p2_tmp = (sigma_est_p2 + 0x8000) >> 16;
  sigma_est_p2_tmp =
      (sigma_est_p2_tmp + AMB_EFF_WIDTH_SIGMA_EST_NS / 2) / AMB_EFF_WIDTH_SIGMA_EST_NS;
  sigma_est_p2_tmp *= AMB_EFF_WIDTH_DMAX_NS;

  if (sigma_est_p2_tmp > 0xffff) {
    min_signal_needed_p3 = 0xfff00000;
  } else {
    /* DMAX uses a different ambient width from sigma, so apply
     * correction.
     * Perform division before multiplication to prevent overflow.
     */
    sigma_est_p2 = (sigma_est_p2 + AMB_EFF_WIDTH_SIGMA_EST_NS / 2) / AMB_EFF_WIDTH_SIGMA_EST_NS;
    sigma_est_p2 *= AMB_EFF_WIDTH_DMAX_NS;

    // fp1616 >> 16 = u32
    min_signal_needed_p3 = (sigma_est_p2 + 0x8000) >> 16;
    min_signal_needed_p3 *= min_signal_needed_p3;
  }

  // fp1814 / u32 = fp1814
  sigma_limit_tmp = ((SIGMA_LIMIT << 14) + 500) / 1000;

  // fp1814 * fp1814 = fp3628 := fp0428
  sigma_limit_tmp *= sigma_limit_tmp;

  // fp1616 * fp1616 = fp3232
  sigma_est_sq_tmp = SIGMA_EST_REF * SIGMA_EST_REF;

  // fp3232 >> 4 = fp0428
  sigma_est_sq_tmp = (sigma_est_sq_tmp + 0x08) >> 4;

  // fp0428 - fp0428 = fp0428
  sigma_limit_tmp -= sigma_est_sq_tmp;

  // u32 * fp0428 = fp0428
  min_signal_needed_p4 = 4 * 12 * sigma_limit_tmp;

  // fp0428 >> 14 = fp1814
  min_signal_needed_p4 = (min_signal_needed_p4 + 0x2000) > 14;

  // u32 + u32 = u32
  min_signal_needed = (min_signal_needed_p2 + min_signal_needed_p3);

  // u32 / u32 = u32
  min_signal_needed += (peak_vcsel_duration_us / 2);
  min_signal_needed /= peak_vcsel_duration_us;

  // u32 << 14 = fp1814
  min_signal_needed <<= 14;

  // fp1814 / fp1814 = u32
  min_signal_needed += min_signal_needed_p4 / 2;
  min_signal_needed /= min_signal_needed_p4;

  // fp3200 * fp2804 := fp2804
  min_signal_needed *= min_signal_needed_p1;

  // apply corrections
  min_signal_needed = (min_signal_needed + 500) / 1000;
  min_signal_needed <<= 4;

  min_signal_needed = (min_signal_needed + 500) / 1000;

  // fp1616 >> 8 = fp2408
  signal_limit_tmp = (SIGNAL_LIMIT + 0x80) >> 8;

  // fp2408 / fp2408 = u32
  if (signal_limit_tmp != 0)
    dmax_dark_tmp = (signal_at_0_mm + (signal_limit_tmp / 2)) / signal_limit_tmp;
  else
    dmax_dark_tmp = 0;

  dmax_dark = vl53l0x_isqrt(dmax_dark_tmp);

  // fp2408 / fp2408 = u32
  if (min_signal_needed != 0)
    dmax_ambient = (signal_at_0_mm + min_signal_needed / 2) / min_signal_needed;
  else
    dmax_ambient = 0;

  dmax_ambient = vl53l0x_isqrt(dmax_ambient);

  *dmax_mm = dmax_dark > dmax_ambient ? dmax_ambient : dmax_dark;

  return VL53L0X_OK;
}

vl53l0x_err_t vl53l0x_calc_sigma_estimate(vl53l0x_handle_t             dev,
                                          vl53l0x_ranging_meas_data_t* meas_data,
                                          fp1616_t* estimate, uint32_t* dmax_mm) {
  const uint32_t PULSE_EFF_WIDTH_C_NS    = 800;
  const uint32_t AMBIENT_EFF_WIDTH_C_NS  = 600;
  const fp1616_t SIGMA_EST_REF           = 0x00000042;  // 0.001
  const uint32_t VCSEL_PULSE_WIDTH_PS    = 4700;
  const fp1616_t SIGMA_EST_MAX           = 0x028F87AE;
  const fp1616_t SIGMA_EST_RTN_MAX       = 0xf000;
  const fp1616_t AMB_TO_SIGNAL_RATIO_MAX = 0xf0000000 / AMBIENT_EFF_WIDTH_C_NS;
  const fp1616_t TOF_PER_MM_PS           = 0x0006999a;  // 6.6 ps
  const uint32_t ROUND_PARAM_16B         = 0x00008000;
  const fp1616_t MAX_XTALK_KCPS          = 0x00320000;
  const uint32_t PLL_PERIOD_PS           = 1655;

  uint32_t vcsel_total_events_rtn;
  fp1616_t sigma_est_p1;
  fp1616_t sigma_est_p2;
  fp1616_t sigma_est_p3;
  fp1616_t deltaT_ps;
  fp1616_t pw_mult;
  fp1616_t sigma_est_rtn;
  fp1616_t sigma_est;
  fp1616_t xtalk_correction;
  fp1616_t ambient_rate_kcps;
  fp1616_t peak_signal_rate_kcps;
  uint32_t xtalk_comp_rate_kcps;

  ambient_rate_kcps = (meas_data->ambient_rate_rtn_mcps * 1000) >> 16;

  fp1616_t corr_signal_rate_mcps = meas_data->signal_rate_rtn_mcps;
  fp1616_t total_signal_rate_mcps;
  ERR_CHECK(_vl53l0x_get_total_signal_rate(dev, meas_data, &total_signal_rate_mcps));
  fp1616_t xtalk_comp_rate_mcps;
  ERR_CHECK(_vl53l0x_get_total_xtalk_rate(dev, meas_data, &xtalk_comp_rate_mcps));

  // signal rate meas is peak, not avg
  peak_signal_rate_kcps = (total_signal_rate_mcps * 1000);
  peak_signal_rate_kcps = (peak_signal_rate_kcps + 0x8000) >> 16;

  xtalk_comp_rate_kcps = xtalk_comp_rate_mcps * 1000;

  if (xtalk_comp_rate_kcps > MAX_XTALK_KCPS) xtalk_comp_rate_kcps = MAX_XTALK_KCPS;

  // calculate final range macro periods
  uint32_t final_range_mpclks =
      calc_timeout_mclks(dev->data.dev_spec_params.final_range_timeout_us,
                         dev->data.dev_spec_params.final_range_vcsel_pulse_period);

  // calculate pre range macro periods
  uint32_t pre_range_mpclks =
      calc_timeout_mclks(dev->data.dev_spec_params.pre_range_timeout_us,
                         dev->data.dev_spec_params.pre_range_vcsel_pulse_period);

  uint32_t vcsel_width = dev->data.dev_spec_params.final_range_vcsel_pulse_period == 8 ? 2 : 3;

  uint32_t peak_vcsel_duration_us = vcsel_width * 2048 * (pre_range_mpclks + final_range_mpclks);

  peak_vcsel_duration_us = (peak_vcsel_duration_us + 500) / 1000;
  peak_vcsel_duration_us *= PLL_PERIOD_PS;
  peak_vcsel_duration_us = (peak_vcsel_duration_us + 500) / 1000;

  // fp1616 >> 8 = fp2408
  total_signal_rate_mcps = (total_signal_rate_mcps + 0x80) >> 8;

  // fp2408 * u32 = fp2408
  vcsel_total_events_rtn = total_signal_rate_mcps * peak_vcsel_duration_us;

  // fp2408 >> 8 = u32
  vcsel_total_events_rtn = (vcsel_total_events_rtn + 0x80) >> 8;

  // fp2404 << 8 = fp1616
  total_signal_rate_mcps <<= 8;

  if (peak_signal_rate_kcps == 0) {
    *estimate           = SIGMA_EST_MAX;
    dev->data.sigma_est = SIGMA_EST_MAX;
    *dmax_mm            = 0;
  } else {
    if (vcsel_total_events_rtn < 1) vcsel_total_events_rtn = 1;

    sigma_est_p1 = PULSE_EFF_WIDTH_C_NS;

    // ((fp1616 << 16) * u32) / u32 = fp1616
    sigma_est_p2 = (ambient_rate_kcps << 16) / peak_signal_rate_kcps;

    // clip to prevent overflow
    if (sigma_est_p2 > AMB_TO_SIGNAL_RATIO_MAX) {
      sigma_est_p2 = AMB_TO_SIGNAL_RATIO_MAX;
    }
    sigma_est_p2 *= AMBIENT_EFF_WIDTH_C_NS;

    sigma_est_p3 = 2 * vl53l0x_isqrt(vcsel_total_events_rtn * 12);

    // u32 * fp1616 = fp1616
    deltaT_ps = meas_data->range_mm * TOF_PER_MM_PS;

    fp1616_t diff1_mcps = (((peak_signal_rate_kcps << 16) - xtalk_comp_rate_kcps) + 500) / 1000;

    fp1616_t diff2_mcps = (((peak_signal_rate_kcps << 16) + xtalk_comp_rate_kcps) + 500) / 1000;

    diff1_mcps <<= 8;

    // NOTE: removed abs
    // fp0824/fp1616 = fp2408
    xtalk_correction = diff1_mcps / diff2_mcps;

    // fp2408 << 8 = fp1616
    xtalk_correction <<= 8;

    // fp1616 / u32 = fp1616
    pw_mult = deltaT_ps / VCSEL_PULSE_WIDTH_PS;

    // fp1616 * fp1616 = fp3232
    pw_mult *= ((1 << 16) - xtalk_correction);

    // fp3232 >> 16 = fp1616
    pw_mult = (pw_mult + ROUND_PARAM_16B) >> 16;

    // fp1616 + fp1616 = fp1616
    pw_mult += 1 << 16;

    /* now the value is 1.xx, if we square it we'll exceed 32 bits. So we first
     * shift the value right by 1 before squaring */
    pw_mult >>= 1;
    // fp1715* fp1715 = fp3430
    pw_mult *= pw_mult;
    // fp3430 >> 14 = fp1616
    pw_mult >>= 14;

    // fp1616 * u32 = fp1616
    fp1616_t sqr1 = pw_mult * sigma_est_p1;

    // fp1616 >> 16 = fp3200
    sqr1 = (sqr1 + 0x8000) >> 16;

    // u32 * u32 = u64
    sqr1 *= sqr1;

    fp1616_t sqr2 = sigma_est_p2;

    // fp1616 >> 16 = u32
    sqr2 = (sqr2 + 0x8000) >> 16;

    // u32 * u32 = u64
    sqr2 *= sqr2;

    // u64 + u64 = u64 (u65)
    fp1616_t sqr_sum = sqr1 + sqr2;

    // sqrt(u64) = u32
    fp1616_t sqrt_result_c_ns = vl53l0x_isqrt(sqr_sum);

    // u32 << 16 = fp1616
    sqrt_result_c_ns <<= 16;

    sigma_est_rtn = (((sqrt_result_c_ns + 50) / 100) / sigma_est_p3);
    sigma_est_rtn *= VL53L0X_SPEED_OF_LIGHT_IN_AIR;
    sigma_est_rtn = (sigma_est_rtn + 5000) / 10000;

    if (sigma_est_rtn > SIGMA_EST_RTN_MAX) sigma_est_rtn = SIGMA_EST_RTN_MAX;

    // fp1616 * fp1616 = fp3232
    sqr1 = sigma_est_rtn * sigma_est_rtn;

    // fp1616 * fp1616 = fp3232
    sqr2 = SIGMA_EST_REF * SIGMA_EST_REF;

    // sqrt(fp3232) = fp1616
    fp1616_t sqrt_result = vl53l0x_isqrt(sqr1 + sqr2);

    sigma_est = 1000 * sqrt_result;

    if (peak_signal_rate_kcps < 1 || vcsel_total_events_rtn < 1 || sigma_est > SIGMA_EST_MAX)
      sigma_est = SIGMA_EST_MAX;

    *estimate           = sigma_est;
    dev->data.sigma_est = sigma_est;
    ERR_CHECK(_vl53l0x_calc_dmax(dev, total_signal_rate_mcps, corr_signal_rate_mcps, pw_mult,
                                 sigma_est_p1, sigma_est_p2, peak_vcsel_duration_us, dmax_mm));
  }

  return VL53L0X_OK;
}
