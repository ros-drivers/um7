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
 *  \brief   Copied directly from the UM6_config.h file, available online here:
 *           http://sourceforge.net/p/um6firmware/code/34/tree/trunk/UM6%20Firmware/UM6_config.h#l14
 *  \author  CH Robotics
 */

#ifndef UMX_DRIVER__UM6_FIRMWARE_REGISTERS_HPP_
#define UMX_DRIVER__UM6_FIRMWARE_REGISTERS_HPP_

// Define the firmware revision
#define UM6_FIRMWARE_REVISION        (('U' << 24) | ('M' << 16) | ('2' << 8) | 'B')

// CONFIG_ARRAY_SIZE and DATA_ARRAY_SIZE specify the number of 32 bit configuration and data
// registers used by the firmware
// (Note: The term "register" is used loosely here.  These "registers" are not actually registers
// in the same sense of a microcontroller register.  They are simply index locations into arrays
// stored in global memory.  Data and configuration parameters are stored in arrays because it
// allows a common communication protocol to be used to access all data and configuration.  The
// software communicating with the sensor needs only specify the register address, and the
// communication software running on the sensor knows exactly where to find it - it needn't know
// what the data is.  The software communication with the sensor, on the other hand, needs to know
// what it is asking for (naturally...) This setup makes it easy to make more data immediately
// available when needed - simply increase the array size, add code in the firmware that writes
// data to the new array location, and then make updates to the firmware definition on the PC side.
#define CONFIG_ARRAY_SIZE           59
#define DATA_ARRAY_SIZE             48
#define COMMAND_COUNT               10

#define CONFIG_REG_START_ADDRESS    0
#define DATA_REG_START_ADDRESS      85
#define COMMAND_START_ADDRESS       170

// These preprocessor definitions make it easier to access specific configuration parameters in code
// They specify array locations associated with each register name.  Note that in the comments
// below, many of the values are said to be 32-bit IEEE floating point.  Obviously this isn't
// directly the case, since the arrays are actually 32-bit unsigned integer arrays.  Bit for bit,
// the data does correspond to the correct floating point value.  Since you can't cast ints as
// floats, special conversion has to happen to copy the float data to and from the array.
// Starting with configuration register locations...
#define UM6_COMMUNICATION        CONFIG_REG_START_ADDRESS        // Stores settings in bits
#define UM6_MISC_CONFIG          (CONFIG_REG_START_ADDRESS + 1)  // Stores settings in bits
// Mag reference values are stored as 32-bit IEEE floating point values
// (these reflect data AFTER scale factors and compensation are applied)
#define UM6_MAG_REF_X            (CONFIG_REG_START_ADDRESS + 2)
#define UM6_MAG_REF_Y            (CONFIG_REG_START_ADDRESS + 3)
#define UM6_MAG_REF_Z            (CONFIG_REG_START_ADDRESS + 4)
// Accel reference values are stored as 32-bit IEEE floating point values
// (these reflect data AFTER scale factors and compensation are applied)
#define UM6_ACCEL_REF_X          (CONFIG_REG_START_ADDRESS + 5)
#define UM6_ACCEL_REF_Y          (CONFIG_REG_START_ADDRESS + 6)
#define UM6_ACCEL_REF_Z          (CONFIG_REG_START_ADDRESS + 7)
// Variances are stored as 32-bit IEEE floating point values.
#define UM6_EKF_MAG_VARIANCE     (CONFIG_REG_START_ADDRESS + 8)
#define UM6_EKF_ACCEL_VARIANCE   (CONFIG_REG_START_ADDRESS + 9)
#define UM6_EKF_PROCESS_VARIANCE (CONFIG_REG_START_ADDRESS + 10)
// Gyro biases are stored as 16-bit signed integers.  Bias correction is applied
// BEFORE scale factors are applied
#define UM6_GYRO_BIAS_XY         (CONFIG_REG_START_ADDRESS + 11)
#define UM6_GYRO_BIAS_Z          (CONFIG_REG_START_ADDRESS + 12)
// Accel biases are stored as 16-bit signed integers.  Bias correction is applied
// BEFORE scale factors are applied
#define UM6_ACCEL_BIAS_XY        (CONFIG_REG_START_ADDRESS + 13)
#define UM6_ACCEL_BIAS_Z         (CONFIG_REG_START_ADDRESS + 14)
// Mag biases are stored as 16-bit signed integers.  Bias correction is applied
// BEFORE magnetometer adjustment
#define UM6_MAG_BIAS_XY          (CONFIG_REG_START_ADDRESS + 15)
#define UM6_MAG_BIAS_Z           (CONFIG_REG_START_ADDRESS + 16)
// The accelerometer alignment matrix is a 3x3 matrix with 32-bit IEEE floating point entries
#define UM6_ACCEL_CAL_00         (CONFIG_REG_START_ADDRESS + 17)
#define UM6_ACCEL_CAL_01         (CONFIG_REG_START_ADDRESS + 18)
#define UM6_ACCEL_CAL_02         (CONFIG_REG_START_ADDRESS + 19)
#define UM6_ACCEL_CAL_10         (CONFIG_REG_START_ADDRESS + 20)
#define UM6_ACCEL_CAL_11         (CONFIG_REG_START_ADDRESS + 21)
#define UM6_ACCEL_CAL_12         (CONFIG_REG_START_ADDRESS + 22)
#define UM6_ACCEL_CAL_20         (CONFIG_REG_START_ADDRESS + 23)
#define UM6_ACCEL_CAL_21         (CONFIG_REG_START_ADDRESS + 24)
#define UM6_ACCEL_CAL_22         (CONFIG_REG_START_ADDRESS + 25)
// The gyro alignment matrix is a 3x3 matrix with 32-bit IEEE floating point entries
#define UM6_GYRO_CAL_00          (CONFIG_REG_START_ADDRESS + 26)
#define UM6_GYRO_CAL_01          (CONFIG_REG_START_ADDRESS + 27)
#define UM6_GYRO_CAL_02          (CONFIG_REG_START_ADDRESS + 28)
#define UM6_GYRO_CAL_10          (CONFIG_REG_START_ADDRESS + 29)
#define UM6_GYRO_CAL_11          (CONFIG_REG_START_ADDRESS + 30)
#define UM6_GYRO_CAL_12          (CONFIG_REG_START_ADDRESS + 31)
#define UM6_GYRO_CAL_20          (CONFIG_REG_START_ADDRESS + 32)
#define UM6_GYRO_CAL_21          (CONFIG_REG_START_ADDRESS + 33)
#define UM6_GYRO_CAL_22          (CONFIG_REG_START_ADDRESS + 34)
// The magnetometer calibration matrix is a 3x3 matrix with 32-bit IEEE floating point entries
#define UM6_MAG_CAL_00           (CONFIG_REG_START_ADDRESS + 35)
#define UM6_MAG_CAL_01           (CONFIG_REG_START_ADDRESS + 36)
#define UM6_MAG_CAL_02           (CONFIG_REG_START_ADDRESS + 37)
#define UM6_MAG_CAL_10           (CONFIG_REG_START_ADDRESS + 38)
#define UM6_MAG_CAL_11           (CONFIG_REG_START_ADDRESS + 39)
#define UM6_MAG_CAL_12           (CONFIG_REG_START_ADDRESS + 40)
#define UM6_MAG_CAL_20           (CONFIG_REG_START_ADDRESS + 41)
#define UM6_MAG_CAL_21           (CONFIG_REG_START_ADDRESS + 42)
#define UM6_MAG_CAL_22           (CONFIG_REG_START_ADDRESS + 43)
// Terms used for gyro temperature compensation.  Each item is a floating point number, to be
// applied to the raw data directly.
#define UM6_GYROX_BIAS_0         (CONFIG_REG_START_ADDRESS + 44)
#define UM6_GYROX_BIAS_1         (CONFIG_REG_START_ADDRESS + 45)
#define UM6_GYROX_BIAS_2         (CONFIG_REG_START_ADDRESS + 46)
#define UM6_GYROX_BIAS_3         (CONFIG_REG_START_ADDRESS + 47)
#define UM6_GYROY_BIAS_0         (CONFIG_REG_START_ADDRESS + 48)
#define UM6_GYROY_BIAS_1         (CONFIG_REG_START_ADDRESS + 49)
#define UM6_GYROY_BIAS_2         (CONFIG_REG_START_ADDRESS + 50)
#define UM6_GYROY_BIAS_3         (CONFIG_REG_START_ADDRESS + 51)
#define UM6_GYROZ_BIAS_0         (CONFIG_REG_START_ADDRESS + 52)
#define UM6_GYROZ_BIAS_1         (CONFIG_REG_START_ADDRESS + 53)
#define UM6_GYROZ_BIAS_2         (CONFIG_REG_START_ADDRESS + 54)
#define UM6_GYROZ_BIAS_3         (CONFIG_REG_START_ADDRESS + 55)
#define UM6_GPS_HOME_LAT         (CONFIG_REG_START_ADDRESS + 56)
#define UM6_GPS_HOME_LONG        (CONFIG_REG_START_ADDRESS + 57)
#define UM6_GPS_HOME_ALTITUDE    (CONFIG_REG_START_ADDRESS + 58)

// Now for data register locations.
// In the communication protocol, data registers are labeled with number ranging from 128 to 255.
// The value of 128 will be subtracted from these numbers to produce the actual array index
// labeled below

// Status register defines error codes with individual bits
#define UM6_STATUS           DATA_REG_START_ADDRESS
// Raw gyro data is stored in 16-bit signed integers
#define UM6_GYRO_RAW_XY      (DATA_REG_START_ADDRESS + 1)
#define UM6_GYRO_RAW_Z       (DATA_REG_START_ADDRESS + 2)
// Raw accel data is stored in 16-bit signed integers
#define UM6_ACCEL_RAW_XY     (DATA_REG_START_ADDRESS + 3)
#define UM6_ACCEL_RAW_Z      (DATA_REG_START_ADDRESS + 4)
// Raw mag data is stored in 16-bit signed integers
#define UM6_MAG_RAW_XY       (DATA_REG_START_ADDRESS + 5)
#define UM6_MAG_RAW_Z        (DATA_REG_START_ADDRESS + 6)
// Processed gyro data has scale factors applied and alignment correction performed.
// Data is 16-bit signed integer.
#define UM6_GYRO_PROC_XY     (DATA_REG_START_ADDRESS + 7)
#define UM6_GYRO_PROC_Z      (DATA_REG_START_ADDRESS + 8)
// Processed accel data has scale factors applied and alignment correction performed.
// Data is 16-bit signed integer.
#define UM6_ACCEL_PROC_XY    (DATA_REG_START_ADDRESS + 9)
#define UM6_ACCEL_PROC_Z     (DATA_REG_START_ADDRESS + 10)
// Processed mag data has scale factors applied and alignment correction performed.
// Data is 16-bit signed integer.
#define UM6_MAG_PROC_XY      (DATA_REG_START_ADDRESS + 11)
#define UM6_MAG_PROC_Z       (DATA_REG_START_ADDRESS + 12)
// Euler angles are 32-bit IEEE floating point
#define UM6_EULER_PHI_THETA  (DATA_REG_START_ADDRESS + 13)
#define UM6_EULER_PSI        (DATA_REG_START_ADDRESS + 14)
// Quaternions are 16-bit signed integers.
#define UM6_QUAT_AB          (DATA_REG_START_ADDRESS + 15)
#define UM6_QUAT_CD          (DATA_REG_START_ADDRESS + 16)
// Error covariance is a 4x4 matrix of 32-bit IEEE floating point values
#define UM6_ERROR_COV_00     (DATA_REG_START_ADDRESS + 17)
#define UM6_ERROR_COV_01     (DATA_REG_START_ADDRESS + 18)
#define UM6_ERROR_COV_02     (DATA_REG_START_ADDRESS + 19)
#define UM6_ERROR_COV_03     (DATA_REG_START_ADDRESS + 20)
#define UM6_ERROR_COV_10     (DATA_REG_START_ADDRESS + 21)
#define UM6_ERROR_COV_11     (DATA_REG_START_ADDRESS + 22)
#define UM6_ERROR_COV_12     (DATA_REG_START_ADDRESS + 23)
#define UM6_ERROR_COV_13     (DATA_REG_START_ADDRESS + 24)
#define UM6_ERROR_COV_20     (DATA_REG_START_ADDRESS + 25)
#define UM6_ERROR_COV_21     (DATA_REG_START_ADDRESS + 26)
#define UM6_ERROR_COV_22     (DATA_REG_START_ADDRESS + 27)
#define UM6_ERROR_COV_23     (DATA_REG_START_ADDRESS + 28)
#define UM6_ERROR_COV_30     (DATA_REG_START_ADDRESS + 29)
#define UM6_ERROR_COV_31     (DATA_REG_START_ADDRESS + 30)
#define UM6_ERROR_COV_32     (DATA_REG_START_ADDRESS + 31)
#define UM6_ERROR_COV_33     (DATA_REG_START_ADDRESS + 32)
#define UM6_TEMPERATURE      (DATA_REG_START_ADDRESS + 33)  // Temp measured by onboard sensors
#define UM6_GPS_LONGITUDE    (DATA_REG_START_ADDRESS + 34)  // GPS longitude
#define UM6_GPS_LATITUDE     (DATA_REG_START_ADDRESS + 35)  // GPS latitude
#define UM6_GPS_ALTITUDE     (DATA_REG_START_ADDRESS + 36)  // GPS altitude
#define UM6_GPS_POSITION_N   (DATA_REG_START_ADDRESS + 37)  // GPS position (meters north of home)
#define UM6_GPS_POSITION_E   (DATA_REG_START_ADDRESS + 38)  // GPS position (meters east of home)
#define UM6_GPS_POSITION_H   (DATA_REG_START_ADDRESS + 39)  // GPS position (meters east of home)
// GPS course over ground in degrees and speed in m/s
#define UM6_GPS_COURSE_SPEED (DATA_REG_START_ADDRESS + 40)
// Summarizes GPS satellite data (mode, satellite count, HDOP, and VDOP)
#define UM6_GPS_SAT_SUMMARY  (DATA_REG_START_ADDRESS + 41)
#define UM6_GPS_SAT_1_2      (DATA_REG_START_ADDRESS + 42)  // ID and SNR for tracked satellites
#define UM6_GPS_SAT_3_4      (DATA_REG_START_ADDRESS + 43)  // ID and SNR for tracked satellites
#define UM6_GPS_SAT_5_6      (DATA_REG_START_ADDRESS + 44)  // ID and SNR for tracked satellites
#define UM6_GPS_SAT_7_8      (DATA_REG_START_ADDRESS + 45)  // ID and SNR for tracked satellites
#define UM6_GPS_SAT_9_10     (DATA_REG_START_ADDRESS + 46)  // ID and SNR for tracked satellites
#define UM6_GPS_SAT_11_12    (DATA_REG_START_ADDRESS + 47)  // ID and SNR for tracked satellites

// Define some stuff to organize the contents of the UM6_GPS_SAT_SUMMARY register
#define UM6_GPS_MODE_START_BIT    30
#define UM6_GPS_MODE_MASK        0x03        // 2 bits

#define UM6_GPS_SAT_COUNT_START_BIT    26
#define UM6_GPS_SAT_COUNT_MASK        0x0F    // 4 bits

#define UM6_GPS_HDOP_START_BIT        16
#define UM6_GPS_HDOP_MASK            0x03FF    // 10 bits

#define UM6_GPS_VDOP_START_BIT        6
#define UM6_GPS_VDOP_MASK            0x03FF    // 10 bits

// Finally, define some non-register registers... sometimes commands must be sent to the sensor -
// commands that don't involve the transmission of any data.  Like, for example, a command to zero
// rate gyros.  Or whatever.  These commands are given "register" addresses so that they can be sent
// using the same communication framework used to set and read registers.  The only difference is
// that when a command is received and no data is attached, the communication code doesn't set any
// registers.
//
// The communication code will do two things for every packet received:
// 1. Copy data to the relevant register if data was provided in the packet
// 2. Call a "dispatch packet" function that performs additional functions if the packet requires it
// Step 2 is what handles commands and causes status packets to be returned.

// Causes the UM6 to report the firmware revision
#define UM6_GET_FW_VERSION     COMMAND_START_ADDRESS
// Causes the UM6 to write all configuration values to FLASH
#define UM6_FLASH_COMMIT       (COMMAND_START_ADDRESS + 1)
// Causes the UM6 to start a zero gyros command
#define UM6_ZERO_GYROS         (COMMAND_START_ADDRESS + 2)
// Causes the UM6 to reset the EKF
#define UM6_RESET_EKF          (COMMAND_START_ADDRESS + 3)
// Causes the UM6 to transmit a data packet containing data from all enabled channels
#define UM6_GET_DATA           (COMMAND_START_ADDRESS + 4)
// Causes the UM6 to set the current measured accel data to the reference vector
#define UM6_SET_ACCEL_REF      (COMMAND_START_ADDRESS + 5)
// Causes the UM6 to set the current measured magnetometer data to the reference vector
#define UM6_SET_MAG_REF        (COMMAND_START_ADDRESS + 6)
// Causes the UM6 to load default factory settings
#define UM6_RESET_TO_FACTORY   (COMMAND_START_ADDRESS + 7)
// Causes the UM6 to save the current settings to the factory flash location
#define UM6_SAVE_FACTORY       (COMMAND_START_ADDRESS + 8)
// Causes the UM6 to save the current GPS position as the "home" position
// (used to compute relative position)
#define UM6_SET_HOME_POSITION  (COMMAND_START_ADDRESS + 9)

#define UM6_USE_CONFIG_ADDRESS     0
#define UM6_USE_FACTORY_ADDRESS    1

#define UM6_BAD_CHECKSUM           253  // Sent if the UM6 receives a packet with a bad checksum
#define UM6_UNKNOWN_ADDRESS        254  // Sent if the UM6 receives a packet with an unknown address
#define UM6_INVALID_BATCH_SIZE     255  // Sent if a requested batch read or write operation would
                                        // go beyond the bounds of the config or data array

// Now make even more definitions for writing data to specific registers
// Start with the UM6_COMMUNICATION register.
// These definitions specify what individual bits in the regist mean
#define UM6_BROADCAST_ENABLED        (1 << 30)  // Enable serial data transmission
#define UM6_GYROS_RAW_ENABLED        (1 << 29)  // Enable transmission of raw gyro data
#define UM6_ACCELS_RAW_ENABLED       (1 << 28)  // Enable transmission of raw accelerometer data
#define UM6_MAG_RAW_ENABLED          (1 << 27)  // Enable transmission of raw magnetometer data
// Enable transmission of processed gyro data (biases removed, scale factor applied, rotation
// correction applied)
#define UM6_GYROS_PROC_ENABLED       (1 << 26)
// Enable transmission of processed accel data (biases removed, scale factor applied, rotation
// correction applied)
#define UM6_ACCELS_PROC_ENABLED      (1 << 25)
// Enable transmission of processed mag data (biases removed, scale factor applied, rotation
// correction applied)
#define UM6_MAG_PROC_ENABLED         (1 << 24)
#define UM6_QUAT_ENABLED             (1 << 23)  // Enable transmission of quaternion data
#define UM6_EULER_ENABLED            (1 << 22)  // Enable transmission of euler angle data
#define UM6_COV_ENABLED              (1 << 21)  // Enable transmission of state covariance data
#define UM6_TEMPERATURE_ENABLED      (1 << 20)  // Enable transmission of gyro temperature readings
// Enable transmission of latitude and longitude datav
#define UM6_GPS_POSITION_ENABLED     (1 << 19)
// Enable transmission of computed North and East position (with respect to home position)
#define UM6_GPS_REL_POSITION_ENABLED (1 << 18)
// Enable transmission of computed GPS course and speed
#define UM6_GPS_COURSE_SPEED_ENABLED (1 << 17)
// Enable transmission of satellite summary data (count, HDOP, VDP, mode)
#define UM6_GPS_SAT_SUMMARY_ENABLED  (1 << 16)
// Enable transmission of satellite data (ID and SNR of each satellite)
#define UM6_GPS_SAT_DATA_ENABLED     (1 << 15)

#define UM6_GPS_BAUD_RATE_MASK       (0x07)
#define UM6_GPS_BAUD_START_BIT       11

// Mask specifying the number of bits used to set the serial baud rate
#define UM6_BAUD_RATE_MASK           (0x07)
#define UM6_BAUD_START_BIT           8       // Specifies the start location of the baud rate bits

// Mask specifying which bits in this register are used to indicate the broadcast frequency
#define UM6_SERIAL_RATE_MASK         (0x000FF)

// MISC Configuration register
#define UM6_MAG_UPDATE_ENABLED       (1 << 31)    // Enable magnetometer-based updates in the EKF
#define UM6_ACCEL_UPDATE_ENABLED     (1 << 30)     // Enable accelerometer-based updates in the EKF
#define UM6_GYRO_STARTUP_CAL         (1 << 29)    // Enable automatic gyro calibration on startup
#define UM6_QUAT_ESTIMATE_ENABLED    (1 << 28)    // Enable quaternion-based state estimation

// UM6 Status Register
#define UM6_MAG_INIT_FAILED          (1 << 31)  // Indicates magnetometer initialization failed
#define UM6_ACCEL_INIT_FAILED        (1 << 30)  // Indicates accelerometer initialization failed
#define UM6_GYRO_INIT_FAILED         (1 << 29)  // Indicates gyro initialization failed
#define UM6_GYRO_ST_FAILED_X         (1 << 28)  // Indicates that the x-axis gyro self test failed
#define UM6_GYRO_ST_FAILED_Y         (1 << 27)  // Indicates that the y-axis gyro self test failed
#define UM6_GYRO_ST_FAILED_Z         (1 << 26)  // Indicates that the z-axis gyro self test failed
#define UM6_ACCEL_ST_FAILED_X        (1 << 25)  // Indicates that the x-axis accel self test failed
#define UM6_ACCEL_ST_FAILED_Y        (1 << 24)  // Indicates that the y-axis accel self test failed
#define UM6_ACCEL_ST_FAILED_Z        (1 << 23)  // Indicates that the z-axis accel self test failed
#define UM6_MAG_ST_FAILED_X          (1 << 22)  // Indicates that the x-axis mag self test failed
#define UM6_MAG_ST_FAILED_Y          (1 << 21)  // Indicates that the y-axis mag self test failed
#define UM6_MAG_ST_FAILED_Z          (1 << 20)  // Indicates that the z-axis mag self test failed
// Indicates that there was an i2c bus error while communicating with the rate gyros
#define UM6_I2C_GYRO_BUS_ERROR       (1 << 19)
// Indicates that there was an i2c bus error while communicating with the accelerometers
#define UM6_I2C_ACCEL_BUS_ERROR      (1 << 18)
// Indicates that there was an i2c bus error while communicating with the magnetometer
#define UM6_I2C_MAG_BUS_ERROR        (1 << 17)
// Indicates that the EKF estimate failed and had to be restarted
#define UM6_EKF_DIVERGENT            (1 << 16)
// Inidicates that the rate gyros failed to signal new data for longer than expected
#define UM6_GYRO_UNRESPONSIVE        (1 << 15)
// Indicates that the accelerometer failed to signal new data for longer than expected
#define UM6_ACCEL_UNRESPONSIVE       (1 << 14)
// Indicates that the magnetometer failed to signal new data for longer than expected
#define UM6_MAG_UNRESPONSIVE         (1 << 13)
// Indicates that a write to flash command failed to complete properly
#define UM6_FLASH_WRITE_FAILED       (1 << 12)

#define UM6_SELF_TEST_COMPLETE       (1 << 0)   // Indicates that a self-test was completed

#endif  // UMX_DRIVER__UM6_FIRMWARE_REGISTERS_HPP_
