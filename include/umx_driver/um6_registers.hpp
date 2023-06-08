// Copyright (c) 2023, Clearpath Robotics, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Clearpath Robotics, Inc nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 *
 *  \file
 *  \brief      Provides the Registers class, which initializes with a suite
 *              of accessors suitable for reading and writing the UM6 registers,
 *              including byte-order conversion and scaling handled.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Hilary Luo <hluo@clearpathrobotics.com> (updated to ROS 2 and combined UM6 and UM7)
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef UMX_DRIVER__UM6_REGISTERS_HPP_
#define UMX_DRIVER__UM6_REGISTERS_HPP_

#if __APPLE__
#include <machine/endian.h>
#else
#include <endian.h>
#endif

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <stdexcept>
#include <string>

#include "umx_driver/um6_firmware_registers.hpp"

#define TO_RADIANS (M_PI / 180.0)
#define TO_DEGREES (180.0 / M_PI)

// This excludes the command registers, which are always sent
// and received with no data.
#define NUM_REGISTERS (DATA_REG_START_ADDRESS + DATA_ARRAY_SIZE)


namespace um6
{

inline void memcpy_network(void * dest, void * src, size_t count)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
  uint8_t * d = reinterpret_cast<uint8_t *>(dest);
  uint8_t * s = reinterpret_cast<uint8_t *>(src);
  for (uint8_t i = 0; i < count; i++) {
    d[i] = s[count - (i + 1)];
  }
#else
  // Copy bytes without reversing.
#warning Big-endian implementation is untested.
  memcpy(dest, src, count);
#endif
}

class Registers;

/**
 * This class provides an accessor of fields contained in one or more
 * consecutive UM6 registers. Each register is nominally a uint32_t,
 * but XYZ vectors are stored as a pair of int16_t values in one
 * register and one in the following register. Other values are
 * stored as int32_t representation or float32s.
 *
 * This class takes care of the necessary transformations to simplify
 * the actual "business logic" of the driver.
 */
class Accessor_
{
public:
  Accessor_(
    Registers * registers, uint8_t register_index, uint8_t register_width, uint8_t array_length)
  : index(register_index), width(register_width), length(array_length), registers_(registers)
  {}

  void * raw() const;

  /**
   * Number/address of the register in the array of uint32s which is
   * shared with the UM6 firmware. */
  const uint8_t index;

  /**
   * Width of the sub-register field, in bytes, either 2 or 4. */
  const uint8_t width;

  /**
   * Length of how many sub-register fields comprise this accessor. Not
   * required to stay within the bounds of a single register. */
  const uint16_t length;

private:
  Registers * registers_;
};

template<typename RegT>
class Accessor : public Accessor_
{
public:
  Accessor(
    Registers * registers, uint8_t register_index, uint8_t array_length = 0,
    double scale_factor = 1.0)
  : Accessor_(registers, register_index, sizeof(RegT), array_length), scale_(scale_factor)
  {
  }

  RegT get(uint8_t field) const
  {
    RegT * raw_ptr = reinterpret_cast<RegT *>(raw());
    RegT value;
    memcpy_network(&value, raw_ptr + field, sizeof(value));
    return value;
  }

  double get_scaled(uint16_t field) const
  {
    return get(field) * scale_;
  }

  void set(uint8_t field, RegT value) const
  {
    RegT * raw_ptr = reinterpret_cast<RegT *>(raw());
    memcpy_network(raw_ptr + field, &value, sizeof(value));
  }

  void set_scaled(uint16_t field, double value) const
  {
    set(field, value / scale_);
  }

private:
  const double scale_;
};

class Registers
{
public:
  Registers()
  : gyro_raw(this, UM6_GYRO_RAW_XY, 3),
    accel_raw(this, UM6_ACCEL_RAW_XY, 3),
    mag_raw(this, UM6_MAG_RAW_XY, 3),
    gyro(this, UM6_GYRO_PROC_XY, 3, 0.0610352 * TO_RADIANS),
    accel(this, UM6_ACCEL_PROC_XY, 3, 0.00179639),
    mag(this, UM6_MAG_PROC_XY, 3, 0.000305176),
    euler(this, UM6_EULER_PHI_THETA, 3, 0.0109863 * TO_RADIANS),
    quat(this, UM6_QUAT_AB, 4, 0.0000335693),
    covariance(this, UM6_ERROR_COV_00, 16),
    temperature(this, UM6_TEMPERATURE, 1),
    communication(this, UM6_COMMUNICATION, 1),
    misc_config(this, UM6_MISC_CONFIG, 1),
    status(this, UM6_STATUS, 1),
    mag_ref(this, UM6_MAG_REF_X, 3),
    accel_ref(this, UM6_ACCEL_REF_X, 3),
    gyro_bias(this, UM6_GYRO_BIAS_XY, 3),
    accel_bias(this, UM6_ACCEL_BIAS_XY, 3),
    mag_bias(this, UM6_MAG_BIAS_XY, 3),
    cmd_zero_gyros(this, UM6_ZERO_GYROS),
    cmd_reset_ekf(this, UM6_RESET_EKF),
    cmd_set_accel_ref(this, UM6_SET_ACCEL_REF),
    cmd_set_mag_ref(this, UM6_SET_MAG_REF)
  {
    memset(raw_, 0, sizeof(raw_));
  }

  // Data
  const Accessor<int16_t> gyro_raw, accel_raw, mag_raw, gyro, accel, mag, euler, quat;
  const Accessor<float> covariance, temperature;

  // Configs
  const Accessor<uint32_t> communication, misc_config, status;
  const Accessor<float> mag_ref, accel_ref;
  const Accessor<int16_t> gyro_bias, accel_bias, mag_bias;

  // Commands
  const Accessor<uint32_t> cmd_zero_gyros, cmd_reset_ekf, cmd_set_accel_ref, cmd_set_mag_ref;

  void write_raw(uint8_t register_index, std::string data)
  {
    if ((register_index - 1) + (data.length() / 4 - 1) >= NUM_REGISTERS) {
      throw std::range_error("Index and length write beyond boundaries of register array.");
    }
    memcpy(&raw_[register_index], data.c_str(), data.length());
  }

private:
  uint32_t raw_[NUM_REGISTERS];

  friend class Accessor_;
};
}  // namespace um6

#endif  // UMX_DRIVER__UM6_REGISTERS_HPP_
