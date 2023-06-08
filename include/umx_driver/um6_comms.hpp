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
 *  \brief      Comms class definition. Does not manage the serial connection
 *              itself, but takes care of reading and writing to UM6.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Hilary Luo <hluo@clearpathrobotics.com> (updated to ROS 2 and combined UM6 and UM7)
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef UMX_DRIVER__UM6_COMMS_HPP_
#define UMX_DRIVER__UM6_COMMS_HPP_

#include <arpa/inet.h>
#include <stdint.h>

#include <string>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

#include "umx_driver/um6_registers.hpp"

namespace um6
{

class SerialTimeout : public std::exception {};

class BadChecksum : public std::exception {};

class Comms
{
public:
  explicit Comms(serial::Serial * s)
  : serial_(s), first_spin_(true)
  {
  }

  /**
   * Returns -1 if the serial port timed out before receiving a packet
   * successfully, or if there was a bad checksum or any other error.
   * Otherwise, returns the 8-bit register number of the successfully
   * returned packet.
   */
  int16_t receive(Registers * r);

  void send(const Accessor_ & a) const;

  bool sendWaitAck(const Accessor_ & a);

  static const uint8_t PACKET_HAS_DATA;
  static const uint8_t PACKET_IS_BATCH;
  static const uint8_t PACKET_BATCH_LENGTH_MASK;
  static const uint8_t PACKET_BATCH_LENGTH_OFFSET;

  static std::string checksum(const std::string & s);

  static std::string message(uint8_t address, std::string data);

private:
  serial::Serial * serial_;
  bool first_spin_;
};
}  // namespace um6

#endif  // UMX_DRIVER__UM6_COMMS_HPP_
