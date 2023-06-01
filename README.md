umx_driver: um6 / um7
===

ROS driver for the CH Robotics UM6 and UM7 inertial measurement units (IMU).
Supports standard data and mag topics as well as providing temperature and rpy outputs.

See the ROS wiki for details on hardware installation and ROS 1 software implementation:  http://wiki.ros.org/um7

## ROS 2 Software - Build from Source:
This driver is built on an updated version of the [original serial library](https://github.com/wjwwood/serial) that has been [updated for ROS 2](https://github.com/RoverRobotics-forks/serial-ros2).

```bash
git clone --branch master https://github.com/RoverRobotics-forks/serial-ros2.git
git clone --branch ros2 https://github.com/ros-drivers/um7.git
colcon build --packages-select serial umx_driver
```

### Run the Driver
For UM7:
```bash
source install/setup.bash
ros2 run umx_driver um7_driver --ros-args -p port:=/dev/ttyUSB0
```
For UM6:
```bash
source install/setup.bash
ros2 run umx_driver um6_driver --ros-args -p port:=/dev/ttyUSB0
```
>**Note**: The "port" assignment actually defaults to "/dev/ttyUSB0", so if your sensor is on that port, the parameter setting shown above is unnecessary. Replace "ttyUSB0" with the port number of your UM7 device.

## Nodes and Topics
>**Note**: The same topic and service names are used for both the um6_driver and um7_driver and thus only one can be run at a time without additional namespacing.

- imu/data (sensor_msgs/msg/Imu)
- imu/mag (sensor_msgs/msg/magnetic_field)
- imu/rpy (geometry_msgs/msg/Vector3Stamped)
- imu/temperature (std_msgs/msg/Float32)

## Parameters & Commands
The parameters are the same as the ROS 1 driver.

## Reset Service
- imu/reset