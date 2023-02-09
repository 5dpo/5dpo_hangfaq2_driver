#include "sdpo_q2_ros_driver/Robot5dpoQ2.h"

namespace sdpo_q2_ros_driver {

void Motor::setEncTicksDelta(const int32_t delta_enc_ticks) {
  enc_ticks_delta = delta_enc_ticks;
  enc_ticks_prev = enc_ticks;
  enc_ticks += enc_ticks_delta;
  setW();
}

void Motor::setEncTicks(const int32_t total_enc_ticks) {
  enc_ticks_prev = enc_ticks;
  enc_ticks = total_enc_ticks;
  enc_ticks_delta = enc_ticks - enc_ticks_prev;
  setW();
}

void Motor::setWr(const double w_ref) {
  w_r = w_ref;
}

void Motor::setSampleTime(const int32_t time_sample) {
  sample_time_prev = sample_time;
  sample_time = ((double) time_sample) / 1.0e6;   // us > s
  sample_period = sample_time - sample_time_prev;
}

void Motor::setW() {
  w = 2 * M_PIf64 * enc_ticks_delta * kMotCtrlFreq /
      (gear_reduction * encoder_res);
}

} // namespace sdpo_q2_ros_driver