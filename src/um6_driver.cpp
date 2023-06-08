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
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Hilary Luo <hluo@clearpathrobotics.com> (updated to ROS 2 and combined UM6 and UM7)
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include "umx_driver/um6_driver.hpp"

// Don't try to be too clever. Arrival of this message triggers
// us to publish everything we have.
const uint8_t TRIGGER_PACKET = UM6_TEMPERATURE;

namespace um6
{

/**
 * Function generalizes the process of commanding the UM6 via one of its command
 * registers.
 */
template<typename RegT>
void Um6Driver::send_command(const um6::Accessor<RegT> & reg, std::string human_name)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Sending command: " << human_name);
  if (!sensor_->sendWaitAck(reg)) {
    throw std::runtime_error(human_name + " command to device failed.");
  }
}


/**
 * Send configuration messages to the UM6, critically, to turn on the value outputs
 * which we require, and inject necessary configuration parameters.
 */
void Um6Driver::configure_sensor()
{
  um6::Registers r;

  // Enable outputs we need.
  const uint8_t UM6_BAUD_115200 = 0x5;
  uint32_t comm_reg = UM6_BROADCAST_ENABLED |
    UM6_GYROS_PROC_ENABLED | UM6_ACCELS_PROC_ENABLED | UM6_MAG_PROC_ENABLED |
    UM6_QUAT_ENABLED | UM6_EULER_ENABLED | UM6_COV_ENABLED | UM6_TEMPERATURE_ENABLED |
    UM6_BAUD_115200 << UM6_BAUD_START_BIT;
  // set the broadcast rate of the device
  int rate;
  this->get_parameter("update_rate", rate);
  if (rate < 20 || rate > 60) {
    RCLCPP_WARN(this->get_logger(), "Potentially unsupported update rate of %d", rate);
  }
  // converting from desired rate to broadcast_rate as defined in UM6 datasheet
  uint32_t rate_bits = (uint32_t) ((rate - 20) * 255.0 / 280.0);
  RCLCPP_INFO(this->get_logger(), "Setting update rate to %uHz", rate);
  comm_reg |= (rate_bits & UM6_SERIAL_RATE_MASK);
  r.communication.set(0, comm_reg);
  if (!sensor_->sendWaitAck(r.communication)) {
    throw std::runtime_error("Unable to set communication register.");
  }

  // Optionally disable mag and accel updates in the sensor's EKF.
  bool mag_updates, accel_updates;


  this->get_parameter("mag_updates", mag_updates);
  this->get_parameter("accel_updates", accel_updates);

  uint32_t misc_config_reg = UM6_QUAT_ESTIMATE_ENABLED;
  if (mag_updates) {
    misc_config_reg |= UM6_MAG_UPDATE_ENABLED;
  } else {
    RCLCPP_WARN(this->get_logger(), "Excluding magnetometer updates from EKF.");
  }
  if (accel_updates) {
    misc_config_reg |= UM6_ACCEL_UPDATE_ENABLED;
  } else {
    RCLCPP_WARN(this->get_logger(), "Excluding accelerometer updates from EKF.");
  }

  r.misc_config.set(0, misc_config_reg);
  if (!sensor_->sendWaitAck(r.misc_config)) {
    throw std::runtime_error("Unable to set misc config register.");
  }

  // Optionally disable the gyro reset on startup. A user might choose to do this
  // if there's an external process which can ascertain when the vehicle is stationary
  // and periodically call the /reset service.
  bool zero_gyros;
  this->get_parameter("zero_gyros", zero_gyros);
  if (zero_gyros) {send_command(r.cmd_zero_gyros, "zero gyroscopes");}
}


bool Um6Driver::handle_reset_service(
  const std::shared_ptr<umx_driver::srv::Um6Reset::Request> req,
  std::shared_ptr<umx_driver::srv::Um6Reset::Response> resp)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  (void)resp;
  um6::Registers r;
  try {
    if (req->zero_gyros) {send_command(r.cmd_zero_gyros, "zero gyroscopes");}
    if (req->reset_ekf) {send_command(r.cmd_reset_ekf, "reset EKF");}
    if (req->set_mag_ref) {send_command(r.cmd_set_mag_ref, "set magnetometer reference");}
    if (req->set_accel_ref) {send_command(r.cmd_set_accel_ref, "set accelerometer reference");}
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
  }
  RCLCPP_INFO(this->get_logger(), "Reset service completed");
  return true;
}

/**
 * Uses the register accessors to grab data from the IMU, and populate
 * the ROS messages which are output.
 */
void Um6Driver::publish_msgs(um6::Registers & r)
{
  imu_msg_.header.stamp = this->now();

  if (imu_pub_->get_subscription_count() > 0) {
    // IMU reports a 4x4 wxyz covariance, ROS requires only 3x3 xyz.
    // NED -> ENU conversion req'd?
    imu_msg_.orientation_covariance[0] = r.covariance.get_scaled(5);
    imu_msg_.orientation_covariance[1] = r.covariance.get_scaled(6);
    imu_msg_.orientation_covariance[2] = r.covariance.get_scaled(7);
    imu_msg_.orientation_covariance[3] = r.covariance.get_scaled(9);
    imu_msg_.orientation_covariance[4] = r.covariance.get_scaled(10);
    imu_msg_.orientation_covariance[5] = r.covariance.get_scaled(11);
    imu_msg_.orientation_covariance[6] = r.covariance.get_scaled(13);
    imu_msg_.orientation_covariance[7] = r.covariance.get_scaled(14);
    imu_msg_.orientation_covariance[8] = r.covariance.get_scaled(15);

    // NED -> ENU conversion (x = y, y = x, z = -z)
    if (tf_ned_to_enu_) {
      imu_msg_.orientation.x = r.quat.get_scaled(2);
      imu_msg_.orientation.y = r.quat.get_scaled(1);
      imu_msg_.orientation.z = -r.quat.get_scaled(3);
      imu_msg_.orientation.w = r.quat.get_scaled(0);

      imu_msg_.angular_velocity.x = r.gyro.get_scaled(1);
      imu_msg_.angular_velocity.y = r.gyro.get_scaled(0);
      imu_msg_.angular_velocity.z = -r.gyro.get_scaled(2);

      imu_msg_.linear_acceleration.x = r.accel.get_scaled(1);
      imu_msg_.linear_acceleration.y = r.accel.get_scaled(0);
      imu_msg_.linear_acceleration.z = -r.accel.get_scaled(2);
    } else {
      imu_msg_.orientation.w = r.quat.get_scaled(0);
      imu_msg_.orientation.x = r.quat.get_scaled(1);
      imu_msg_.orientation.y = r.quat.get_scaled(2);
      imu_msg_.orientation.z = r.quat.get_scaled(3);

      imu_msg_.angular_velocity.x = r.gyro.get_scaled(0);
      imu_msg_.angular_velocity.y = r.gyro.get_scaled(1);
      imu_msg_.angular_velocity.z = r.gyro.get_scaled(2);

      imu_msg_.linear_acceleration.x = r.accel.get_scaled(0);
      imu_msg_.linear_acceleration.y = r.accel.get_scaled(1);
      imu_msg_.linear_acceleration.z = r.accel.get_scaled(2);
    }

    imu_pub_->publish(imu_msg_);
  }

  // Magnetometer.  transform to ROS axes
  if (mag_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header = imu_msg_.header;

    if (tf_ned_to_enu_) {
      mag_msg.magnetic_field.x = r.mag.get_scaled(1);
      mag_msg.magnetic_field.y = r.mag.get_scaled(0);
      mag_msg.magnetic_field.z = -r.mag.get_scaled(2);
    } else {
      mag_msg.magnetic_field.x = r.mag.get_scaled(0);
      mag_msg.magnetic_field.y = r.mag.get_scaled(1);
      mag_msg.magnetic_field.z = r.mag.get_scaled(2);
    }

    mag_pub_->publish(mag_msg);
  }

  // Euler attitudes.  transform to ROS axes
  if (rpy_pub_->get_subscription_count() > 0) {
    geometry_msgs::msg::Vector3Stamped rpy_msg;
    rpy_msg.header = imu_msg_.header;

    if (tf_ned_to_enu_) {
      rpy_msg.vector.x = r.euler.get_scaled(1);
      rpy_msg.vector.y = r.euler.get_scaled(0);
      rpy_msg.vector.z = -r.euler.get_scaled(2);
    } else {
      rpy_msg.vector.x = r.euler.get_scaled(0);
      rpy_msg.vector.y = r.euler.get_scaled(1);
      rpy_msg.vector.z = r.euler.get_scaled(2);
    }

    rpy_pub_->publish(rpy_msg);
  }

  // Temperature
  if (temperature_pub_->get_subscription_count() > 0) {
    std_msgs::msg::Float32 temp_msg;
    temp_msg.data = r.temperature.get_scaled(0);
    temperature_pub_->publish(temp_msg);
  }
}

Um6Driver::Um6Driver()
: rclcpp::Node("um6_driver")
{
  // Load parameters
  std::string port = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  int32_t baud = this->declare_parameter<int32_t>("baud", 115200);

  // Parameters for configure_sensor
  this->declare_parameter<int>("update_rate", 20);
  this->declare_parameter<bool>("mag_updates", true);
  this->declare_parameter<bool>("accel_updates", true);
  this->declare_parameter<bool>("zero_gyros", true);

  serial_.setPort(port);
  serial_.setBaudrate(baud);
  serial::Timeout to = serial::Timeout(100, 100, 0, 100, 0);
  serial_.setTimeout(to);

  imu_msg_.header.frame_id = this->declare_parameter<std::string>("frame_id", "imu_link");
  // Defaults obtained experimentally from hardware, no device spec exists
  double linear_acceleration_stdev =
    this->declare_parameter<double>("linear_acceleration_stdev", 0.06);
  double angular_velocity_stdev =
    this->declare_parameter<double>("angular_velocity_stdev", 0.005);

  double linear_acceleration_var = linear_acceleration_stdev * linear_acceleration_stdev;
  double angular_velocity_var = angular_velocity_stdev * angular_velocity_stdev;

  // Enable converting from NED to ENU by default

  tf_ned_to_enu_ = this->declare_parameter<bool>("tf_ned_to_enu", true);

  imu_msg_.linear_acceleration_covariance[0] = linear_acceleration_var;
  imu_msg_.linear_acceleration_covariance[4] = linear_acceleration_var;
  imu_msg_.linear_acceleration_covariance[8] = linear_acceleration_var;

  imu_msg_.angular_velocity_covariance[0] = angular_velocity_var;
  imu_msg_.angular_velocity_covariance[4] = angular_velocity_var;
  imu_msg_.angular_velocity_covariance[8] = angular_velocity_var;

  // Create ROS interfaces
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 1);
  rpy_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy", 1);
  temperature_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu/temperature", 1);
}

void Um6Driver::update_loop(void)
{
  bool first_failure = true;
  while (rclcpp::ok()) {
    try {
      if (serial_.isOpen()) {
        serial_.close();
      }
      serial_.open();
    } catch (const serial::IOException & e) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "um6_driver was unable to connect to port: " << serial_.getPort());
    }
    if (serial_.isOpen()) {
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "um6_driver successfully connected to serial port: " << serial_.getPort());
      first_failure = true;
      try {
        sensor_.reset(new um6::Comms(&serial_));
        configure_sensor();
        um6::Registers registers;
        auto service = this->create_service<umx_driver::srv::Um6Reset>(
          "imu/reset", std::bind(
            &Um6Driver::handle_reset_service, this, std::placeholders::_1, std::placeholders::_2));

        while (rclcpp::ok()) {
          int16_t input = 0;
          {
            const std::lock_guard<std::mutex> lock(mutex_);
            input = sensor_->receive(&registers);
          }
          // triggered by arrival of last message packet
          if (input == TRIGGER_PACKET) {
            // Triggered by arrival of final message in group.
            publish_msgs(registers);
          }
        }
      } catch (const std::exception & e) {
        if (serial_.isOpen()) {serial_.close();}
        RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
        RCLCPP_INFO(this->get_logger(), "Attempting reconnection after error.");
        rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
      }
    } else {
      RCLCPP_WARN_STREAM_EXPRESSION(
        this->get_logger(),
        first_failure, "Could not connect to serial device "
          << serial_.getPort() << ". Trying again every 1 second.");
      first_failure = false;
      rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
    }
  }
}
}  // namespace um6

/**
 * Node entry-point. Handles ROS setup, and serial port connection/reconnection.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<um6::Um6Driver>();

  // start up a new thread that spins the node for service requests
  std::promise<void> stop_async_spinner;
  std::thread async_spinner_thread(
    [stop_token = stop_async_spinner.get_future(), node]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      executor.spin_until_future_complete(stop_token);
    });

  while (rclcpp::ok()) {
    try {
      node->update_loop();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
    }
  }

  stop_async_spinner.set_value();
  async_spinner_thread.join();

  return 0;
}
