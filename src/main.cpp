#include "sdpo_q2_ros_driver/SdpoQ2ROSDriver.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sdpo_q2_ros_driver");

  sdpo_q2_ros_driver::SdpoQ2ROSDriver q2_ros_driver;
  q2_ros_driver.run();

  return 0;
}
