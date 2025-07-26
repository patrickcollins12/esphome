#include "pid_control.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pid_control {

static const char *const TAG = "pid.control";

void PIDControl::setup() {
  this->sensor_->add_on_state_callback([this](float state) {
    if (this->is_pid_enabled_()) {
      this->update_pid_(state);
    } else {
      this->write_output_(0.0f);
    }
  });
}

void PIDControl::loop() {
  // The PID logic is handled in the sensor callback
}

bool PIDControl::is_pid_enabled_() {
  if (this->enable_switch_ == nullptr) {
    return true;
  }
  return this->enable_switch_->state;
}

void PIDControl::write_output_(float value) {
  if (this->increase_output_ != nullptr) {
    this->increase_output_->set_level(value > 0.0f ? value : 0.0f);
  }
  if (this->decrease_output_ != nullptr) {
    this->decrease_output_->set_level(value < 0.0f ? -value : 0.0f);
  }
}

}  // namespace pid_control
}  // namespace esphome
