#include "sdpo_hangfaq2_driver/SdpoHangfaQ2DriverROS.h"

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<sdpo_hangfaq2_driver::SdpoHangfaQ2DriverROS>());

  rclcpp::shutdown();

  return 0;

}
