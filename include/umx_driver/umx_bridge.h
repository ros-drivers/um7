/**
 *
 *  \file
 *  \brief      Main entry point for UM7 driver. Handles serial connection
 *              details, as well as all ROS message stuffing, parameters,
 *              topics, etc.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com> (original code for UM6/7 in ROS1)
 *  \author     Alex Brown <rbirac@cox.net>		    (adapted to UM7)
 *  \author     Hilary Luo <hluo@clearpathrobotics.com> (updated to ROS 2 and combined UM6 and UM7)
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. OR ALEX BROWN BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef UMX_DRIVER_H_
#define UMX_DRIVER_H_

#include <thread>
#include <string>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/float32.hpp"
#include "umx_driver/comms.h"
#include "umx_driver/registers.h"
#include "umx_driver/srv/reset.hpp"

//#include "std_msgs/msg/Header.h" // ?

namespace umx
{

namespace OutputAxisOptions
{
  enum OutputAxisOption
  {
    DEFAULT, ENU, ROBOT_FRAME
  };
}
typedef OutputAxisOptions::OutputAxisOption OutputAxisOption;

class UmxDriver : public rclcpp::Node
{
public:
  UmxDriver(); // const rclcpp::NodeOptions & options);

  void update_loop(void);

private:
  void publish_msgs(umx::Registers& r);

  void configure_sensor();

  template<typename RegT>
  void send_command(const umx::Accessor<RegT>& reg, std::string human_name);

  bool handle_reset_service(
    const std::shared_ptr<umx_driver::srv::Reset::Request> req,
    std::shared_ptr<umx_driver::srv::Reset::Response> resp);

  // ROS2 Interfaces
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;

  OutputAxisOption axes_;
  serial::Serial serial_;
  std::shared_ptr<umx::Comms> sensor_;
  sensor_msgs::msg::Imu imu_msg_;

};

}  // namespace umx

#endif  // UMX_DRIVER_H_
