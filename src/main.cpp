/**
 *
 *  \file
 *  \brief      Main entry point for UM7 driver. Handles serial connection
 *              details, as well as all ROS message stuffing, parameters,
 *              topics, etc.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com> (original code for UM6)
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  \author     Alex Brown <rbirac@cox.net>		    (adapted to UM7)
 *  \copyright  Copyright (c) 2015, Alex Brown.
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
#include <string>

#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "um7/comms.h"
#include "um7/registers.h"
#include "um7/Reset.h"

const char VERSION[10] = "0.0.2";   // um7_driver version

// Don't try to be too clever. Arrival of this message triggers
// us to publish everything we have.
const uint8_t TRIGGER_PACKET = DREG_EULER_PHI_THETA;

/**
 * Function generalizes the process of writing an XYZ vector into consecutive
 * fields in UM7 registers.
 */
template<typename RegT>
void configureVector3(um7::Comms* sensor, const um7::Accessor<RegT>& reg,
                      std::string param, std::string human_name)
{
  if (reg.length != 3)
  {
    throw std::logic_error("configureVector3 may only be used with 3-field registers!");
  }

  if (ros::param::has(param))
  {
    double x, y, z;
    ros::param::get(param + "/x", x);
    ros::param::get(param + "/y", y);
    ros::param::get(param + "/z", z);
    ROS_INFO_STREAM("Configuring " << human_name << " to ("
                    << x << ", " << y << ", " << z << ")");
    reg.set_scaled(0, x);
    reg.set_scaled(1, y);
    reg.set_scaled(2, z);
    if (sensor->sendWaitAck(reg))
    {
      throw std::runtime_error("Unable to configure vector.");
    }
  }
}

/**
 * Function generalizes the process of commanding the UM7 via one of its command
 * registers.
 */
template<typename RegT>
void sendCommand(um7::Comms* sensor, const um7::Accessor<RegT>& reg, std::string human_name)
{
  ROS_INFO_STREAM("Sending command: " << human_name);
  if (!sensor->sendWaitAck(reg))
  {
    throw std::runtime_error("Command to device failed.");
  }
}


/**
 * Send configuration messages to the UM7, critically, to turn on the value outputs
 * which we require, and inject necessary configuration parameters.
 */
void configureSensor(um7::Comms* sensor, ros::NodeHandle *private_nh)
{
  um7::Registers r;

  uint32_t comm_reg = (BAUD_115200 << COM_BAUD_START);
  r.communication.set(0, comm_reg);
  if (!sensor->sendWaitAck(r.comrate2))
  {
    throw std::runtime_error("Unable to set CREG_COM_SETTINGS.");
  }

  // set the broadcast rate of the device
  int rate;
  private_nh->param<int>("update_rate", rate, 20);
  if (rate < 20 || rate > 255)
  {
    ROS_WARN("Potentially unsupported update rate of %d", rate);
  }

  uint32_t rate_bits = static_cast<uint32_t>(rate);
  ROS_INFO("Setting update rate to %uHz", rate);
  uint32_t raw_rate = (rate_bits << RATE2_ALL_RAW_START);
  r.comrate2.set(0, raw_rate);
  if (!sensor->sendWaitAck(r.comrate2))
  {
    throw std::runtime_error("Unable to set CREG_COM_RATES2.");
  }

  uint32_t proc_rate = (rate_bits << RATE4_ALL_PROC_START);
  r.comrate4.set(0, proc_rate);
  if (!sensor->sendWaitAck(r.comrate4))
  {
    throw std::runtime_error("Unable to set CREG_COM_RATES4.");
  }

  uint32_t misc_rate = (rate_bits << RATE5_EULER_START) | (rate_bits << RATE5_QUAT_START);
  r.comrate5.set(0, misc_rate);
  if (!sensor->sendWaitAck(r.comrate5))
  {
    throw std::runtime_error("Unable to set CREG_COM_RATES5.");
  }

  uint32_t health_rate = (5 << RATE6_HEALTH_START);  // note:  5 gives 2 hz rate
  r.comrate6.set(0, health_rate);
  if (!sensor->sendWaitAck(r.comrate6))
  {
    throw std::runtime_error("Unable to set CREG_COM_RATES6.");
  }


  // Options available using parameters)
  uint32_t misc_config_reg = 0;  // initialize all options off

  // Optionally disable mag updates in the sensor's EKF.
  bool mag_updates;
  private_nh->param<bool>("mag_updates", mag_updates, true);
  if (mag_updates)
  {
    misc_config_reg |= MAG_UPDATES_ENABLED;
  }
  else
  {
    ROS_WARN("Excluding magnetometer updates from EKF.");
  }

  // Optionally enable quaternion mode .
  bool quat_mode;
  private_nh->param<bool>("quat_mode", quat_mode, true);
  if (quat_mode)
  {
    misc_config_reg |= QUATERNION_MODE_ENABLED;
  }
  else
  {
    ROS_WARN("Excluding quaternion mode.");
  }

  r.misc_config.set(0, misc_config_reg);
  if (!sensor->sendWaitAck(r.misc_config))
  {
    throw std::runtime_error("Unable to set CREG_MISC_SETTINGS.");
  }

  // Optionally disable performing a zero gyros command on driver startup.
  bool zero_gyros;
  private_nh->param<bool>("zero_gyros", zero_gyros, true);
  if (zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");
}


bool handleResetService(um7::Comms* sensor,
    const um7::Reset::Request& req, const um7::Reset::Response& resp)
{
  um7::Registers r;
  if (req.zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");
  if (req.reset_ekf) sendCommand(sensor, r.cmd_reset_ekf, "reset EKF");
  if (req.set_mag_ref) sendCommand(sensor, r.cmd_set_mag_ref, "set magnetometer reference");
  return true;
}

/**
 * Uses the register accessors to grab data from the IMU, and populate
 * the ROS messages which are output.
 */
void publishMsgs(um7::Registers& r, ros::NodeHandle* imu_nh, sensor_msgs::Imu& imu_msg, bool tf_ned_to_enu)
{
  static ros::Publisher imu_pub = imu_nh->advertise<sensor_msgs::Imu>("data", 1, false);
  static ros::Publisher mag_pub = imu_nh->advertise<geometry_msgs::Vector3Stamped>("mag", 1, false);
  static ros::Publisher rpy_pub = imu_nh->advertise<geometry_msgs::Vector3Stamped>("rpy", 1, false);
  static ros::Publisher temp_pub = imu_nh->advertise<std_msgs::Float32>("temperature", 1, false);

  if (imu_pub.getNumSubscribers() > 0)
  {
    // body-fixed frame NED to ENU: (x y z)->(x -y -z) or (w x y z)->(x -y -z w)
    // world frame      NED to ENU: (x y z)->(y  x -z) or (w x y z)->(y  x -z w)
    if (tf_ned_to_enu)
    {
      // world frame
      imu_msg.orientation.w =  r.quat.get_scaled(2);
      imu_msg.orientation.x =  r.quat.get_scaled(1);
      imu_msg.orientation.y = -r.quat.get_scaled(3);
      imu_msg.orientation.z =  r.quat.get_scaled(0);

      // body-fixed frame
      imu_msg.angular_velocity.x =  r.gyro.get_scaled(0);
      imu_msg.angular_velocity.y = -r.gyro.get_scaled(1);
      imu_msg.angular_velocity.z = -r.gyro.get_scaled(2);

      // body-fixed frame
      imu_msg.linear_acceleration.x =  r.accel.get_scaled(0);
      imu_msg.linear_acceleration.y = -r.accel.get_scaled(1);
      imu_msg.linear_acceleration.z = -r.accel.get_scaled(2);
    }
    else
    {
      imu_msg.orientation.w = r.quat.get_scaled(0);
      imu_msg.orientation.x = r.quat.get_scaled(1);
      imu_msg.orientation.y = r.quat.get_scaled(2);
      imu_msg.orientation.z = r.quat.get_scaled(3);

      imu_msg.angular_velocity.x = r.gyro.get_scaled(0);
      imu_msg.angular_velocity.y = r.gyro.get_scaled(1);
      imu_msg.angular_velocity.z = r.gyro.get_scaled(2);

      imu_msg.linear_acceleration.x = r.accel.get_scaled(0);
      imu_msg.linear_acceleration.y = r.accel.get_scaled(1);
      imu_msg.linear_acceleration.z = r.accel.get_scaled(2);
    }

    imu_pub.publish(imu_msg);
  }

  // Magnetometer.  transform to ROS axes
  if (mag_pub.getNumSubscribers() > 0)
  {
    geometry_msgs::Vector3Stamped mag_msg;
    mag_msg.header = imu_msg.header;

    if (tf_ned_to_enu)
    {
      // world frame
      mag_msg.vector.x =  r.mag.get_scaled(1);
      mag_msg.vector.y =  r.mag.get_scaled(0);
      mag_msg.vector.z = -r.mag.get_scaled(2);
    }
    else
    {
      mag_msg.vector.x = r.mag.get_scaled(0);
      mag_msg.vector.y = r.mag.get_scaled(1);
      mag_msg.vector.z = r.mag.get_scaled(2);
    }

    mag_pub.publish(mag_msg);
  }

  // Euler attitudes.  transform to ROS axes
  if (rpy_pub.getNumSubscribers() > 0)
  {
    geometry_msgs::Vector3Stamped rpy_msg;
    rpy_msg.header = imu_msg.header;

    if (tf_ned_to_enu)
    {
      // world frame
      rpy_msg.vector.x =  r.euler.get_scaled(1);
      rpy_msg.vector.y =  r.euler.get_scaled(0);
      rpy_msg.vector.z = -r.euler.get_scaled(2);
    }
    else
    {
      rpy_msg.vector.x = r.euler.get_scaled(0);
      rpy_msg.vector.y = r.euler.get_scaled(1);
      rpy_msg.vector.z = r.euler.get_scaled(2);
    }

    rpy_pub.publish(rpy_msg);
  }

  // Temperature
  if (temp_pub.getNumSubscribers() > 0)
  {
    std_msgs::Float32 temp_msg;
    temp_msg.data = r.temperature.get_scaled(0);
    temp_pub.publish(temp_msg);
  }
}

/**
 * Node entry-point. Handles ROS setup, and serial port connection/reconnection.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "um7_driver");

  // Load parameters from private node handle.
  std::string port;
  int32_t baud;

  ros::NodeHandle imu_nh("imu"), private_nh("~");
  private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
  private_nh.param<int32_t>("baud", baud, 115200);

  serial::Serial ser;
  ser.setPort(port);
  ser.setBaudrate(baud);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  ser.setTimeout(to);

  sensor_msgs::Imu imu_msg;
  double linear_acceleration_stdev, angular_velocity_stdev;
  private_nh.param<std::string>("frame_id", imu_msg.header.frame_id, "imu_link");
  // Defaults obtained experimentally from hardware, no device spec exists
  private_nh.param<double>("linear_acceleration_stdev", linear_acceleration_stdev, (4.0 * 1e-3f * 9.80665));
  private_nh.param<double>("angular_velocity_stdev", angular_velocity_stdev, (0.06 * 3.14159 / 180.0));

  double linear_acceleration_cov = linear_acceleration_stdev * linear_acceleration_stdev;
  double angular_velocity_cov = angular_velocity_stdev * angular_velocity_stdev;

  // From the UM7 datasheet for the dynamic accuracy from the EKF.
  double orientation_x_stdev, orientation_y_stdev, orientation_z_stdev;
  private_nh.param<double>("orientation_x_stdev", orientation_x_stdev, (3.0 * 3.14159 / 180.0));
  private_nh.param<double>("orientation_y_stdev", orientation_y_stdev, (3.0 * 3.14159 / 180.0));
  private_nh.param<double>("orientation_z_stdev", orientation_z_stdev, (5.0 * 3.14159 / 180.0));

  double orientation_x_covar = orientation_x_stdev * orientation_x_stdev;
  double orientation_y_covar = orientation_y_stdev * orientation_y_stdev;
  double orientation_z_covar = orientation_z_stdev * orientation_z_stdev;

  // Enable converting from NED to ENU by default
  bool tf_ned_to_enu;
  private_nh.param<bool>("tf_ned_to_enu", tf_ned_to_enu, true);

  // These values do not need to be converted
  imu_msg.linear_acceleration_covariance[0] = linear_acceleration_cov;
  imu_msg.linear_acceleration_covariance[4] = linear_acceleration_cov;
  imu_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;

  imu_msg.angular_velocity_covariance[0] = angular_velocity_cov;
  imu_msg.angular_velocity_covariance[4] = angular_velocity_cov;
  imu_msg.angular_velocity_covariance[8] = angular_velocity_cov;

  imu_msg.orientation_covariance[0] = orientation_x_covar;
  imu_msg.orientation_covariance[4] = orientation_y_covar;
  imu_msg.orientation_covariance[8] = orientation_z_covar;

  // Real Time Loop
  bool first_failure = true;
  while (ros::ok())
  {
    try
    {
      ser.open();
    }
    catch (const serial::IOException& e)
    {
        ROS_WARN("um7_driver was unable to connect to port %s.", port.c_str());
    }
    if (ser.isOpen())
    {
      ROS_INFO("um7_driver successfully connected to serial port %s.", port.c_str());
      first_failure = true;
      try
      {
        um7::Comms sensor(&ser);
        configureSensor(&sensor, &private_nh);
        um7::Registers registers;
        ros::ServiceServer srv = imu_nh.advertiseService<um7::Reset::Request, um7::Reset::Response>(
            "reset", boost::bind(handleResetService, &sensor, _1, _2));

        while (ros::ok())
        {
          // triggered by arrival of last message packet
          if (sensor.receive(&registers) == TRIGGER_PACKET)
          {
            // Triggered by arrival of final message in group.
            imu_msg.header.stamp = ros::Time::now();
            publishMsgs(registers, &imu_nh, imu_msg, tf_ned_to_enu);
            ros::spinOnce();
          }
        }
      }
      catch(const std::exception& e)
      {
        if (ser.isOpen()) ser.close();
        ROS_ERROR_STREAM(e.what());
        ROS_INFO("Attempting reconnection after error.");
        ros::Duration(1.0).sleep();
      }
    }
    else
    {
      ROS_WARN_STREAM_COND(first_failure, "Could not connect to serial device "
                           << port << ". Trying again every 1 second.");
      first_failure = false;
      ros::Duration(1.0).sleep();
    }
  }
}
