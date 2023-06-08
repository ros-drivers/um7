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
 *  \brief      Main entry point for UM6 driver. Handles serial connection
 *              details, as well as all ROS message stuffing, parameters,
 *              topics, etc.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com> (original code for UM6 in ROS1)
 *  \author     Hilary Luo <hluo@clearpathrobotics.com> (updated to ROS 2 and combined UM6 and UM7)
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 */

#ifndef UMX_DRIVER__UM6_DRIVER_HPP_
#define UMX_DRIVER__UM6_DRIVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/float32.hpp"
#include "umx_driver/um6_comms.hpp"
#include "umx_driver/um6_registers.hpp"
#include "umx_driver/srv/um6_reset.hpp"

namespace um6
{

class Um6Driver : public rclcpp::Node
{
public:
  Um6Driver();

  void update_loop(void);

private:
  void publish_msgs(um6::Registers & r);

  void configure_sensor();

  template<typename RegT>
  void send_command(const um6::Accessor<RegT> & reg, std::string human_name);

  bool handle_reset_service(
    const std::shared_ptr<umx_driver::srv::Um6Reset::Request> req,
    std::shared_ptr<umx_driver::srv::Um6Reset::Response> resp);

  // ROS2 Interfaces
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;

  serial::Serial serial_;
  std::shared_ptr<um6::Comms> sensor_;
  sensor_msgs::msg::Imu imu_msg_;
  std::mutex mutex_;
  bool tf_ned_to_enu_;
};
}  // namespace um6

#endif  // UMX_DRIVER__UM6_DRIVER_HPP_
