#include "sdpo_hangfaq2_driver/SdpoHangfaQ2DriverROS2.h"

namespace sdpo_hangfaq2_driver {

using namespace std::chrono_literals;

SdpoHangfaQ2DriverROS2::SdpoHangfaQ2DriverROS2()
    : Node("sdpo_hangfaq2_driver") , serial_comms_first_fault_(true) {

  this->declare_parameter<double>("encoder_res", 48.0);
  this->declare_parameter<double>("gear_reduction", 64.0);
  this->declare_parameter<std::string>("serial_port_name", "/dev/ttyACM0");

  getParam();



  sample_time_ = this->now();



  pub_mot_enc_ = this->create_publisher
      <sdpo_drivers_interfaces::msg::MotEncArray>("motors_enc", 10);

  sub_mot_ref_ = this->create_subscription
      <sdpo_drivers_interfaces::msg::MotRefArray>(
          "motors_ref", 1,
          std::bind(&SdpoHangfaQ2DriverROS2::subMotRef, this,
              std::placeholders::_1));



  rob_.setSerialPortName(serial_port_name_.get_value<std::string>());
  rob_.openSerial();

  rob_.run = std::bind(&SdpoHangfaQ2DriverROS2::run, this);
  rob_.init();



  serial_port_timer_ = this->create_wall_timer(
      1s, std::bind(&SdpoHangfaQ2DriverROS2::checkSerialComms, this));
}



void SdpoHangfaQ2DriverROS2::getParam() {

  encoder_res_ = this->get_parameter("encoder_res");

  gear_reduction_ = this->get_parameter("gear_reduction");

  serial_port_name_ = this->get_parameter("serial_port_name");



  for (auto& m : rob_.mot) {

    m.encoder_res = encoder_res_.get_value<double>();

    m.gear_reduction = gear_reduction_.get_value<double>();

  }



  RCLCPP_INFO(this->get_logger(),
              "Encoder resolution: %lf (ticks/rev)", rob_.mot[0].encoder_res);

  RCLCPP_INFO(this->get_logger(),
              "Gear reduction ratio: %lf (n:1)", rob_.mot[0].gear_reduction);

  RCLCPP_INFO(this->get_logger(),
              "Serial port: %s",
              serial_port_name_.get_value<std::string>().c_str());

}



void SdpoHangfaQ2DriverROS2::checkSerialComms() {

  if (!rob_.isSerialOpen())
  {
    if (serial_comms_first_fault_)
    {
      serial_comms_first_fault_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "Couldn't open the serial port %s. Will retry every second.",
                  serial_port_name_.get_value<std::string>().c_str());
    }

    rob_.mtx_.lock();
    rob_.stopMotors();
    rob_.mtx_.unlock();

    rob_.closeSerial();
    rob_.openSerial();

    if (rob_.isSerialOpen())
    {
      serial_comms_first_fault_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Opened serial port %s.",
                  serial_port_name_.get_value<std::string>().c_str());

      rob_.init();
    }
  }

}



void SdpoHangfaQ2DriverROS2::run() {

  try
  {
    if (rclcpp::Duration(this->now() - sample_time_) >
        rclcpp::Duration::from_seconds(kWatchdogMotWRef))
    {
      rob_.mtx_.lock();
      rob_.stopMotors();
      rob_.mtx_.unlock();
    }
  }
  catch (std::exception& e)
  {
    RCLCPP_WARN(this->get_logger(),
                "Not possible to check the driver timeout condition (%s)",
                e.what());

    sample_time_ = this->now();

    return;
  }

  pubMotEnc();

}



void SdpoHangfaQ2DriverROS2::pubMotEnc() {

  sdpo_drivers_interfaces::msg::MotEncArray msg;



  msg.stamp = this->now();
  msg.mot_enc.resize(4);



  rob_.mtx_.lock();

  for (int i = 0; i < 4; i++) {

    msg.mot_enc[i].enc_delta = rob_.mot[i].getEncTicksDeltaPub();

    msg.mot_enc[i].ticks_per_rev =
        rob_.mot[i].encoder_res * rob_.mot[i].gear_reduction;

    msg.mot_enc[i].ang_speed = rob_.mot[i].w;

  }

  rob_.mtx_.unlock();



  pub_mot_enc_->publish(msg);

}



void SdpoHangfaQ2DriverROS2::subMotRef(
    const sdpo_drivers_interfaces::msg::MotRefArray::SharedPtr msg) {

  if (msg->ang_speed_ref.size() >= 4) {

    rob_.mtx_.lock();

    for (int i = 0; i < 4; i++) {
      rob_.mot[i].w_r = msg->ang_speed_ref[i].ref;
    }

    rob_.mtx_.unlock();



    sample_time_ = msg->stamp;

  }

}

} // namespace sdpo_hangfaq2_driver
