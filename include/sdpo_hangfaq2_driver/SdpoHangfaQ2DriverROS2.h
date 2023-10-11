#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sdpo_drivers_interfaces/msg/mot_enc_array.hpp>
#include <sdpo_drivers_interfaces/msg/mot_ref_array.hpp>

#include "sdpo_hangfaq2_driver/SdpoHangfaQ2.h"

namespace sdpo_hangfaq2_driver {

const double kWatchdogMotWRef = 0.5;

class SdpoHangfaQ2DriverROS2 : public rclcpp::Node
{

 private:

  rclcpp::Publisher<sdpo_drivers_interfaces::msg::MotEncArray>::SharedPtr
      pub_mot_enc_;

  rclcpp::Subscription<sdpo_drivers_interfaces::msg::MotRefArray>::SharedPtr
      sub_mot_ref_;



  rclcpp::TimerBase::SharedPtr serial_port_timer_;



  rclcpp::Time sample_time_;



  SdpoHangfaQ2 rob_;



  double encoder_res_;
  double gear_reduction_;
  std::string serial_port_name_;



  bool serial_comms_first_fault_;





 public:

  SdpoHangfaQ2DriverROS2();



 private:

  void getParam();

  void checkSerialComms();

  void run();

  void pubMotEnc();
  void subMotRef(
      const sdpo_drivers_interfaces::msg::MotRefArray::SharedPtr msg);

};

} // namespace sdpo_hangfaq2_driver
