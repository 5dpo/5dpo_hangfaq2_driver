#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sdpo_drivers_interfaces/msg/mot_enc_array.hpp>
#include <sdpo_drivers_interfaces/msg/mot_ref_array.hpp>

#include "sdpo_hangfaq2_driver/SdpoHangfaQ2.h"

namespace sdpo_hangfaq2_driver {

const double kWatchdogMotWRef = 0.2;

class SdpoHangfaQ2DriverROS : public rclcpp::Node
{

 private:

  rclcpp::Publisher<sdpo_drivers_interfaces::msg::MotEncArray>::SharedPtr
      pub_mot_enc_;

  rclcpp::Subscription<sdpo_drivers_interfaces::msg::MotRefArray>::SharedPtr
      sub_mot_ref_;



  rclcpp::Time sample_time_;



  SdpoHangfaQ2 rob_;



  rclcpp::Parameter encoder_res_;
  rclcpp::Parameter gear_reduction_;
  rclcpp::Parameter serial_port_name_;





 public:

  SdpoHangfaQ2DriverROS();



 private:

  void getParam();

  void pubMotEnc();
  void subMotRef(
      const sdpo_drivers_interfaces::msg::MotRefArray::SharedPtr msg);

};

} // namespace sdpo_hangfaq2_driver
