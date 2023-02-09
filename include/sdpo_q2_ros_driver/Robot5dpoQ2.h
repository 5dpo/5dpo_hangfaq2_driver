#pragma once

#include <ros/ros.h>

#include "sdpo_q2_ros_driver/SerialChannelsConfig.h"

namespace sdpo_q2_ros_driver {

const int kCtrlFreq = 25;
const int kMotCtrlFreq = 100;

struct Motor {
 public:
  double encoder_res = 1;
  double gear_reduction = 1;
  int32_t enc_ticks = 0;
  int32_t enc_ticks_prev = 0;
  int32_t enc_ticks_delta = 0;
  double w_r = 0;
  double w = 0;
  double sample_time = 0;
  double sample_time_prev = 0;
  double sample_period = 0;

 public:
  void setEncTicksDelta(const int32_t delta_enc_ticks);
  void setEncTicks(const int32_t total_enc_ticks);

  void setWr(const double w_ref);

  void setSampleTime(const int32_t time_sample);

 private:
  void setW();
};

} // namespace sdpo_q2_ros_driver
