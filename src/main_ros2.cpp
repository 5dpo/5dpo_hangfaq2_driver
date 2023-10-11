#include "sdpo_hangfaq2_driver/SdpoHangfaQ2DriverROS2.h"

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<sdpo_hangfaq2_driver::SdpoHangfaQ2DriverROS2>());

  rclcpp::shutdown();

  return 0;

}
