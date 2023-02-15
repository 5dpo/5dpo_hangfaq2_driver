# 5dpo_q2_ros_driver

**Version 2.0.0**

This repository implements a driver within a ROS package to communicate with the
firmware present in the 5dpo Hangfa Q2 Discovery robot. The driver is required
for communicating with the robot and have available all its different functions.

The serial communication is handled by
[Boost.Asio](https://www.boost.org/doc/libs/1_80_0/doc/html/boost_asio.html).
This communication is based on the example `4_callback` provided in the
[serial-port](https://github.com/fedetft/serial-port) GitHub repository.

**With this version, it is possible to do:**

- Communicate with Arduino Mega 2560 using Boost.Asio
  ([sdpo_ros_serial_port](https://github.com/5dpo/5dpo_ros_serial_port))
- Subscribe motors angular speed reference
- Publish encoders data (encoders + wheels angular speed)
- Read encoders
- Set motors speed
- Reset driver upon reset signal
- Watchdog timer to monitor the motors angular speed reference

**The next version will add these features:**

- Publish optionally the odometry data

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [sdpo_ros_interfaces_hw](https://github.com/5dpo/5dpo_ros_interfaces)
- [sdpo_ros_serial_port](https://github.com/5dpo/5dpo_ros_serial_port)
- [serial_communication_channels](https://github.com/5dpo/serial_communication_channels)

### Parameters

- enc_ticks_per_rev (`float = 48.0`): resolution of the encoder (ticks/rot)
- gear_reduction (`float = 64.0`): reduction ratio of the transmissions
  (\[gear_reduction:1\])
- serial_port_name (`std::string = "/dev/ttyACM0"`): name of the serial port

### Subscribes

- motors_ref
  ([mot_ref.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_ref.msg))

### Publishes

- motors_encoders
  ([mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg))

### Services

None.

### Actions

None.

## Usage

### Build

```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src

# Clone repository
cd ~/catkin_ws/src
git clone git@github.com:5dpo/5dpo_q2_ros_driver.git

# Build
cd ..
catkin build
```

### Launch

```sh
roslaunch sdpo_q2_ros_driver sdpo_q2_ros_driver.launch
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira ([gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [inesctec](mailto:heber.m.sobreira@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
