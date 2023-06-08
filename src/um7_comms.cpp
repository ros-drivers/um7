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
 *  \brief      Implementation of Comms class methods to handle reading and
 *              writing to the UM7 serial interface.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Hilary Luo <hluo@clearpathrobotics.com> (updated to ROS 2 and combined UM6 and UM7)
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include "umx_driver/um7_comms.hpp"

namespace um7
{

const uint8_t Comms::PACKET_HAS_DATA = 1 << 7;
const uint8_t Comms::PACKET_IS_BATCH = 1 << 6;
const uint8_t Comms::PACKET_BATCH_LENGTH_MASK = 0x0F;
const uint8_t Comms::PACKET_BATCH_LENGTH_OFFSET = 2;

int16_t Comms::receive(Registers * registers = NULL)
{
  // Search the serial stream for a start-of-packet sequence.
  try {
    size_t available = serial_->available();
    if (available > 500) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger(
          "um7_driver"),
        "Serial read buffer is " << available << ", now flushing in an attempt to catch up.");
      serial_->flushInput();
    }

    // Optimistically assume that the next five bytes on the wire are a packet header.
    uint8_t header_bytes[5];
    serial_->read(header_bytes, 5);

    uint8_t type, address;
    if (memcmp(header_bytes, "snp", 3) == 0) {
      // Optimism win.
      type = header_bytes[3];
      address = header_bytes[4];
    } else {
      // Optimism fail. Search the serial stream for a header.
      std::string snp;
      serial_->readline(snp, 500, "snp");
      if (!boost::algorithm::ends_with(snp, "snp")) {throw SerialTimeout();}
      if ((snp.length() > 3) && !first_spin_) {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("um7_driver"),
          "Discarded " << 5 + snp.length() - 3 << " junk byte(s) preceeding packet.");
      }
      if (serial_->read(&type, 1) != 1) {throw SerialTimeout();}
      if (serial_->read(&address, 1) != 1) {throw SerialTimeout();}
    }

    first_spin_ = false;

    uint16_t checksum_calculated = 's' + 'n' + 'p' + type + address;
    std::string data;
    if (type & PACKET_HAS_DATA) {
      uint8_t data_length = 1;
      if (type & PACKET_IS_BATCH) {
        data_length = (type >> PACKET_BATCH_LENGTH_OFFSET) & PACKET_BATCH_LENGTH_MASK;
        RCLCPP_DEBUG(
          rclcpp::get_logger("um7_driver"),
          "Received packet %02x with batched (%d) data.", address, data_length);
      } else {
        RCLCPP_DEBUG(
          rclcpp::get_logger("um7_driver"),
          "Received packet %02x with non-batched data.", address);
      }

      // Read data bytes initially into a buffer so that we can compute the checksum.
      if (serial_->read(data, data_length * 4) != data_length * 4) {throw SerialTimeout();}
      BOOST_FOREACH(uint8_t ch, data)
      {
        checksum_calculated += ch;
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("um7_driver"), "Received packet %02x without data.", address);
    }

    // Compare computed checksum with transmitted value.
    uint16_t checksum_transmitted;
    if (serial_->read(reinterpret_cast<uint8_t *>(&checksum_transmitted), 2) != 2) {
      throw SerialTimeout();
    }
    checksum_transmitted = ntohs(checksum_transmitted);
    if (checksum_transmitted != checksum_calculated) {
      throw BadChecksum();
    }

    // Copy data from checksum buffer into registers, if specified.
    // Note that byte-order correction (as necessary) happens at access-time
    if ((data.length() > 0) && registers) {
      registers->write_raw(address, data);
    }

    // Successful packet read, return address byte.
    return address;
  } catch (const SerialTimeout & e) {
    RCLCPP_WARN(rclcpp::get_logger("um7_driver"), "Timed out waiting for packet from device.");
  } catch (const BadChecksum & e) {
    RCLCPP_WARN(rclcpp::get_logger("um7_driver"), "Discarding packet due to bad checksum.");
  }
  return -1;
}

std::string Comms::checksum(const std::string & s)
{
  uint16_t checksum = 0;
  BOOST_FOREACH(uint8_t ch, s)
  {
    checksum += ch;
  }
  checksum = htons(checksum);
  RCLCPP_DEBUG(
    rclcpp::get_logger("um7_driver"),
    "Computed checksum on string of length %zd as %04x.", s.length(), checksum);
  std::string out(2, 0);
  memcpy(&out[0], &checksum, 2);
  return out;
}

std::string Comms::message(uint8_t address, std::string data)
{
  uint8_t type = 0;
  if (data.length() > 0) {
    type |= PACKET_HAS_DATA;
  }
  if (data.length() > 4) {
    type |= PACKET_IS_BATCH;
    type |= (data.length() / 4) << PACKET_BATCH_LENGTH_OFFSET;
  }

  std::stringstream ss(std::stringstream::out | std::stringstream::binary);
  ss << "snp" << type << address << data;
  std::string output = ss.str();
  std::string c = checksum(output);
  ss << c;
  output = ss.str();
  RCLCPP_DEBUG(
    rclcpp::get_logger("um7_driver"),
    "Generated message %02x of overall length %zd.", address, output.length());
  return output;
}

void Comms::send(const Accessor_ & a) const
{
  std::string data(reinterpret_cast<char *>(a.raw()), a.length * 4);
  serial_->write(message(a.index, data));
}

bool Comms::sendWaitAck(const Accessor_ & a)
{
  const uint8_t tries = 5;
  for (uint8_t t = 0; t < tries; t++) {
    send(a);
    const uint8_t listens = 20;
    for (uint8_t i = 0; i < listens; i++) {
      int16_t received = receive();
      if (received == a.index) {
        RCLCPP_DEBUG(rclcpp::get_logger("um7_driver"), "Message %02x ack received.", received);
        return true;
      } else if (received == -1) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("um7_driver"),
          "Serial read timed out waiting for ack. Attempting to retransmit.");
        break;
      }
    }
  }
  return false;
}
}  // namespace um7
