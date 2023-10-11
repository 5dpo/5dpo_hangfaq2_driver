#include "sdpo_hangfaq2_driver/SdpoHangfaQ2DriverROS1.h"

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "sdpo_hangfaq2_driver");

  sdpo_hangfaq2_driver::SdpoHangfaQ2DriverROS1 node;

  ros::spin();

  ros::shutdown();

  return 0;

}
