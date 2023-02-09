#include "sdpo_q2_ros_driver/Robot5dpoQ2.h"

#include <exception>

namespace sdpo_q2_ros_driver {

void Motor::setEncoderRes(const double enc_res) {
  if (enc_res <= 0) {
    throw std::invalid_argument(
        "Motor::setEncoderRes: "
        "resolution of the encoder must be greater than 0 (enc ticks / rev)");
  }
  encoder_res = enc_res;
}

void Motor::setGearReduction(const double gear_ratio) {
  if (gear_ratio <= 0) {
    throw std::invalid_argument(
        "Motor::setGearReduction: "
        "gear reduction ratio of the motor must be greater than 0 ([n:1])");
  }
  gear_reduction = gear_ratio;
}

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