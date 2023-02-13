#include "sdpo_q2_ros_driver/SdpoQ2ROSDriver.h"

namespace sdpo_q2_ros_driver {

SdpoQ2ROSDriver::SdpoQ2ROSDriver() : loop_rate_(kCtrlFreq) {
  readParam();

  rob_.setSerialPortName(serial_port_name_);
  rob_.openSerial();

  pub_mot_enc_ = nh.advertise<sdpo_ros_interfaces_hw::mot_enc_array>(
      "motors_encoders", 1);
  sub_mot_ref_ = nh.subscribe("motors_ref", 1,
                              &SdpoQ2ROSDriver::subMotRef, this);
}

void SdpoQ2ROSDriver::run() {
  sample_time_prev_ = ros::Time::now();

  while (ros::ok()) {
    if (sample_time_ - sample_time_prev_ > ros::Duration(kWatchdogMotWRef)) {
      rob_.mtx_.lock();
      rob_.stopMotors();
      rob_.mtx_.unlock();
    }

    if (!rob_.isSerialOpen()) {
      ROS_WARN("[sdpo_q2_ros_driver] Serial port %s not available",
               serial_port_name_.c_str());
      rob_.mtx_.lock();
      rob_.stopMotors();
      rob_.mtx_.unlock();
      rob_.closeSerial();
      rob_.openSerial();
    } else {
      pubMotEnc();
    }

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

bool SdpoQ2ROSDriver::readParam() {
  ros::NodeHandle nh_private("~");

  for (auto& m : rob_.mot) {
    nh_private.param<double>("encoder_res", m.encoder_res, 48.0);
    nh_private.param<double>("gear_reduction", m.gear_reduction, 64.0);
  }
  nh_private.param<std::string>("serial_port_name", serial_port_name_,
                                "/dev/ttyACM0");

  auto print_is_default_param_set =
      [&nh_private](const std::string& param_name) {
    if (!nh_private.hasParam(param_name)) {
      ROS_INFO("[sdpo_q2_ros_driver] Parameter %s not set in the "
               "parameter server (using default value)",
               param_name.c_str());
    }
  };

  print_is_default_param_set("encoder_res");
  ROS_INFO("[sdpo_q2_ros_driver] Encoder resolution: %lf (ticks/rev)",
           rob_.mot[0].encoder_res);

  print_is_default_param_set("gear_reduction");
  ROS_INFO("[sdpo_q2_ros_driver] Gear reduction ratio: %lf (n:1)",
           rob_.mot[0].gear_reduction);

  print_is_default_param_set("serial_port_name");
  ROS_INFO("[sdpo_q2_ros_driver] Serial port: %s", serial_port_name_.c_str());

  return true;
}

void SdpoQ2ROSDriver::pubMotEnc() {
  sdpo_ros_interfaces_hw::mot_enc_array msg;

  msg.stamp = ros::Time::now();
  msg.mot_enc_array_data.resize(4);

  rob_.mtx_.lock();
  for (int i = 0; i < 4; i++) {
    msg.mot_enc_array_data[i].encoder_counter = rob_.mot[i].enc_ticks;
    msg.mot_enc_array_data[i].ticks_per_rev =
        rob_.mot[i].encoder_res * rob_.mot[i].gear_reduction;
    msg.mot_enc_array_data[i].angular_speed = rob_.mot[i].w;
  }
  rob_.mtx_.unlock();

  pub_mot_enc_.publish(msg);
}

void SdpoQ2ROSDriver::subMotRef(const sdpo_ros_interfaces_hw::mot_ref& msg) {
  if (msg.angular_speed_ref.size() >= 4) {
    rob_.mtx_.lock();
    for (int i = 0; i < 4; i++) {
      rob_.mot[i].w_r = msg.angular_speed_ref[i];
    }
    rob_.mtx_.unlock();

    sample_time_prev_ = sample_time_;
    sample_time_ = ros::Time::now();
  }
}

} // namespace sdpo_q2_ros_driver
