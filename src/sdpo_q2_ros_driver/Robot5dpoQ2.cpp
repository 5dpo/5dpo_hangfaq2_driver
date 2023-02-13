#include "sdpo_q2_ros_driver/Robot5dpoQ2.h"

#include <exception>
#include <utility>
#include <vector>

namespace sdpo_q2_ros_driver {

const unsigned int kSerialBaudRate = 115200;
const auto kSerialDataBits = boost::asio::serial_port_base::character_size(8);
const auto kSerialStopBits = boost::asio::serial_port_base::stop_bits::one;
const auto kSerialParity = boost::asio::serial_port_base::parity::none;
const auto kSerialFlowCtrl = boost::asio::serial_port_base::flow_control::none;

void Motor::setEncoderRes(const double& enc_res) {
  if (enc_res <= 0) {
    throw std::invalid_argument(
        "Motor::setEncoderRes: "
        "resolution of the encoder must be greater than 0 (enc ticks / rev)");
  }
  encoder_res = enc_res;
}

void Motor::setGearReduction(const double& gear_ratio) {
  if (gear_ratio <= 0) {
    throw std::invalid_argument(
        "Motor::setGearReduction: "
        "gear reduction ratio of the motor must be greater than 0 ([n:1])");
  }
  gear_reduction = gear_ratio;
}

void Motor::setEncTicksDelta(const int32_t& delta_enc_ticks) {
  enc_ticks_delta = delta_enc_ticks;
  enc_ticks_prev = enc_ticks;
  enc_ticks += enc_ticks_delta;
  setW();
}

void Motor::setEncTicks(const int32_t& total_enc_ticks) {
  enc_ticks_prev = enc_ticks;
  enc_ticks = total_enc_ticks;
  enc_ticks_delta = enc_ticks - enc_ticks_prev;
  setW();
}

void Motor::setWr(const double& w_ref) {
  w_r = w_ref;
}

void Motor::setSampleTime(const int32_t& time_sample) {
  sample_time_prev = sample_time;
  sample_time = ((double) time_sample) / 1.0e6;   // us > s
  sample_period = sample_time - sample_time_prev;
}

void Motor::setW() {
  w = 2 * M_PI * enc_ticks_delta * kMotCtrlFreq /
      (gear_reduction * encoder_res);
}

Robot5dpoQ2::Robot5dpoQ2() : serial_async_(nullptr) {
  serial_cfg_ = InitCommunications();
}

Robot5dpoQ2::Robot5dpoQ2(std::string serial_port_name)
    : serial_async_(nullptr),
      serial_port_name_(std::move(serial_port_name)) {
  serial_cfg_ = InitCommunications();
}

Robot5dpoQ2::~Robot5dpoQ2() {
  closeSerial();
}

bool Robot5dpoQ2::openSerial() {
  try {
    serial_async_ = new CallbackAsyncSerial(
        serial_port_name_, kSerialBaudRate,
        boost::asio::serial_port_base::parity(kSerialParity),
        kSerialDataBits,
        boost::asio::serial_port_base::flow_control(kSerialFlowCtrl),
        boost::asio::serial_port_base::stop_bits(kSerialStopBits));
    serial_async_->setCallback(
        std::bind(&Robot5dpoQ2::rcvSerialData, this,
                  std::placeholders::_1, std::placeholders::_2));
    return true;
  } catch (boost::system::system_error& e) {
    serial_async_ = nullptr;
    std::cerr << "[sdpo_q2_ros_driver] Error when opening the serial device "
              << serial_port_name_ << std::endl;
    return false;
  }
}

void Robot5dpoQ2::closeSerial() {
  if (serial_async_) {
    try {
      serial_async_->close();
    } catch(...) {
      std::cerr << "[sdpo_q2_ros_driver] Error when closing the serial device "
                << serial_port_name_ << std::endl;
    }
    delete serial_async_;
  }
}

bool Robot5dpoQ2::isSerialOpen() {
  return serial_async_ ?
         !(serial_async_->errorStatus() || (!serial_async_->isOpen())) :
         false;
}

void Robot5dpoQ2::setSerialPortName(const std::string& serial_port_name) {
  serial_port_name_ = serial_port_name;
}

void Robot5dpoQ2::stopMotors() {
  for(auto& m : mot) {
    m.w_r = 0;
  }
}

void Robot5dpoQ2::rcvSerialData(const char *data, unsigned int len) {
  std::vector<char> vec(data, data + len);
  char channel;

  for(auto& c : vec) {
    if (!((c < 32) || (c >= 0x7f))) { // ignore non-ascii char
      channel = ProcessChannelsSerialData(c);

      if (channel > 0) {
        switch (channel) {
        case 'g':
          sendSerialData();
          mtx_.lock();
          mot[0].setEncTicks(serial_cfg_->channel_g);
          mtx_.unlock();
          break;

        case 'h':
          mtx_.lock();
          mot[1].setEncTicks(serial_cfg_->channel_h);
          mtx_.unlock();
          break;

        case 'i':
          mtx_.lock();
          mot[2].setEncTicks(serial_cfg_->channel_i);
          mtx_.unlock();
          break;

        case 'j':
          mtx_.lock();
          mot[3].setEncTicks(serial_cfg_->channel_j);
          mtx_.unlock();
          break;

        case 'k':
          mtx_.lock();
          for(auto& m : mot) {
            m.setSampleTime(serial_cfg_->channel_k);
          }
          mtx_.unlock();
          break;

        default:
          break;
        }
      }
    }
  }
}

void Robot5dpoQ2::sendSerialData() {
  mtx_.lock();
  serial_cfg_->channel_G = static_cast<float>(mot[0].w_r);
  serial_cfg_->channel_H = static_cast<float>(mot[1].w_r);
  serial_cfg_->channel_I = static_cast<float>(mot[2].w_r);
  serial_cfg_->channel_J = static_cast<float>(mot[3].w_r);
  mtx_.unlock();

  serial_async_->writeString(SendChannel('G'));
  serial_async_->writeString(SendChannel('H'));
  serial_async_->writeString(SendChannel('I'));
  serial_async_->writeString(SendChannel('J'));
}

} // namespace sdpo_q2_ros_driver