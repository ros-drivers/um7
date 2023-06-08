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
 *  \brief      Testing the functionality of the Registors and Accessor classes.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Hilary Luo <hluo@clearpathrobotics.com> (updated to ROS 2 and combined UM6 and UM7)
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <arpa/inet.h>
#include <gtest/gtest.h>

#include "umx_driver/um7_registers.hpp"


TEST(ByteOrder, compare_with_htons)
{
  // Arbitrary, just try a selection of values.
  for (uint16_t host_num = 0; host_num < 50000; host_num += 71) {
    uint16_t net_num = htons(host_num);
    uint16_t memcpy_num = 0;
    um7::memcpy_network(&memcpy_num, &host_num, sizeof(host_num));
    EXPECT_EQ(memcpy_num, net_num);
  }
}

TEST(ByteOrder, compare_with_htonl)
{
  for (uint32_t host_num = 0; host_num < 4000000000; host_num += 1299827) {
    uint32_t net_num = htonl(host_num);
    uint32_t memcpy_num = 0;
    um7::memcpy_network(&memcpy_num, &host_num, sizeof(host_num));
    EXPECT_EQ(memcpy_num, net_num);
  }
}

TEST(Accessor, basic_int)
{
  um7::Registers r;
  r.write_raw(5, "\x01\x02\x03\x04\x05\x06");

  um7::Accessor<uint16_t> u16(&r, 5, 3);
  EXPECT_EQ(0x0102, u16.get(0));
  EXPECT_EQ(0x0304, u16.get(1));
  EXPECT_EQ(0x0506, u16.get(2));

  um7::Accessor<uint16_t> u16n(&r, 6, 1);
  EXPECT_EQ(0x0506, u16n.get(0));

  um7::Accessor<uint32_t> u32(&r, 5, 3);
  EXPECT_EQ(0x01020304, u32.get(0));
}

TEST(Accessor, basic_float)
{
  um7::Registers r;
  r.write_raw(10, "\x01\x02\x03\x04\x05\x06\x07\x08");

  um7::Accessor<float> f(&r, 10, 2);
  union {
    float val;
    uint32_t bytes;
  };
  bytes = 0x01020304;
  EXPECT_FLOAT_EQ(val, f.get(0));
  bytes = 0x05060708;
  EXPECT_FLOAT_EQ(val, f.get(1));
}

TEST(Accessor, scaled_int)
{
  um7::Registers r;
  r.write_raw(11, "\x01\x02\x03\x04");

  const float scale(0.001);
  um7::Accessor<int16_t> i16(&r, 11, 2, scale);
  EXPECT_FLOAT_EQ(scale * 0x0102, i16.get_scaled(0));
  EXPECT_FLOAT_EQ(scale * 0x0304, i16.get_scaled(1));
}

TEST(Accessor, set_float)
{
  um7::Registers r;
  r.mag_bias.set(0, 0.123);
  r.mag_bias.set_scaled(1, 0.987);
  r.mag_bias.set_scaled(2, 0.555);

  float check;
  um7::memcpy_network(&check, reinterpret_cast<float *>(r.mag_bias.raw()), 4);
  EXPECT_FLOAT_EQ(0.123, check);
  um7::memcpy_network(&check, reinterpret_cast<float *>(r.mag_bias.raw()) + 1, 4);
  EXPECT_FLOAT_EQ(0.987, check);
  um7::memcpy_network(&check, reinterpret_cast<float *>(r.mag_bias.raw()) + 2, 4);
  EXPECT_FLOAT_EQ(0.555, check);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
