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
 *  \brief      Testing the functionality of the serial communication by emulating a virtual device.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Hilary Luo <hluo@clearpathrobotics.com> (updated to ROS 2 and combined UM6 and UM7)
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <fcntl.h>
#include <gtest/gtest.h>
#include "serial/serial.h"

#include "umx_driver/um6_comms.hpp"
#include "umx_driver/um6_registers.hpp"

class FakeSerial : public ::testing::Test
{
protected:
  /**
   * What's going on here? The posix_openpt() call establishes a pseudo terminal
   * and gives us a fd for the other end of it. So we can connect up a Serial
   * instance to the pty, and have full control over reading-from and writing-to
   * the driver.
   */
  virtual void SetUp()
  {
    ASSERT_NE(-1, master_fd = posix_openpt(O_RDWR | O_NOCTTY | O_NDELAY));
    ASSERT_NE(-1, grantpt(master_fd));
    ASSERT_NE(-1, unlockpt(master_fd));
    ASSERT_TRUE((ser_name = ptsname(master_fd)) != NULL);
    ser.setPort(ser_name);
    ser.open();
    ASSERT_TRUE(ser.isOpen()) << "Couldn't open Serial connection to pseudoterminal.";
  }

  void write_serial(const std::string & msg)
  {
    write(master_fd, msg.c_str(), msg.length());
  }

  virtual void TearDown()
  {
    ser.close();
    close(master_fd);
  }

  serial::Serial ser;

private:
  int master_fd;
  char * ser_name;
};

TEST_F(FakeSerial, basic_message_rx)
{
  // Send message from device which should write four bytes to the raw magnetometer's first register
  std::string msg(um6::Comms::message(UM6_MAG_RAW_XY, std::string("\x1\x2\x3\x4")));
  write_serial(msg);

  um6::Comms sensor(&ser);
  um6::Registers registers;
  ASSERT_EQ(UM6_MAG_RAW_XY, sensor.receive(&registers)) << "Didn't return ID of arriving message.";
  EXPECT_EQ(0x0102, registers.mag_raw.get(0));
}

TEST_F(FakeSerial, batch_message_rx)
{
  // Send message from device which should write four bytes to the raw accelerometer's registers.
  std::string msg(um6::Comms::message(UM6_ACCEL_RAW_XY, std::string("\x5\x6\x7\x8\x9\xa\0\0", 8)));
  write_serial(msg);

  um6::Comms sensor(&ser);
  um6::Registers registers;
  ASSERT_EQ(
    UM6_ACCEL_RAW_XY,
    sensor.receive(&registers)) << "Didn't return ID of arriving message.";
  EXPECT_EQ(0x0506, registers.accel_raw.get(0));
  EXPECT_EQ(0x0708, registers.accel_raw.get(1));
  EXPECT_EQ(0x090a, registers.accel_raw.get(2));
}

TEST_F(FakeSerial, bad_checksum_message_rx)
{
  // Generate message, then twiddle final byte.
  std::string msg(um6::Comms::message(UM6_MAG_RAW_XY, std::string("\x1\x2\x3\x4")));
  msg[msg.length() - 1]++;
  write_serial(msg);

  um6::Comms sensor(&ser);
  um6::Registers registers;
  EXPECT_EQ(-1, sensor.receive(&registers)) << "Didn't properly ignore bad checksum message.";
}

TEST_F(FakeSerial, garbage_bytes_preceeding_message_rx)
{
  // Generate message, then prepend junk.
  std::string msg(um6::Comms::message(UM6_COMMUNICATION, std::string()));
  msg = "ssssssnsnsns" + msg;
  write_serial(msg);

  um6::Comms sensor(&ser);
  EXPECT_EQ(
    UM6_COMMUNICATION,
    sensor.receive(NULL)) << "Didn't handle garbage prepended to message.";
}

TEST_F(FakeSerial, timeout_message_rx)
{
  std::string msg("snp\x12\x45");
  write_serial(msg);
  um6::Comms sensor(&ser);
  EXPECT_EQ(-1, sensor.receive(NULL))
    << "Didn't properly time out in the face of a partial message.";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
