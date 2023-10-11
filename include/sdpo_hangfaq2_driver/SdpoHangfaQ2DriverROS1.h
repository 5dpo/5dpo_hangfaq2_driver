#pragma once

#include <ros/ros.h>
#include <sdpo_drivers_interfaces/MotEncArrayROS1.h>
#include <sdpo_drivers_interfaces/MotRefArrayROS1.h>

#include "sdpo_hangfaq2_driver/SdpoHangfaQ2.h"

namespace sdpo_hangfaq2_driver {

const double kWatchdogMotWRef = 0.5;

class SdpoHangfaQ2DriverROS1
{

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Publisher pub_mot_enc_;
  ros::Subscriber sub_mot_ref_;



  ros::Timer serial_port_timer_;



  ros::Time sample_time_;



  SdpoHangfaQ2 rob_;



  double encoder_res_;
  double gear_reduction_;
  std::string serial_port_name_;



  bool serial_comms_first_fault_;





 public:

  SdpoHangfaQ2DriverROS1();



 private:

  void getParam();

  void checkSerialComms();

  void run();

  void pubMotEnc();
  void subMotRef(
      const sdpo_drivers_interfaces::MotRefArrayROS1::ConstPtr& msg);

};

} // namespace sdpo_hangfaq2_driver
