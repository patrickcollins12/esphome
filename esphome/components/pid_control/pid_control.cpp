#include "pid_control.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pid_control {

static const char *const TAG = "pid_control";

void PIDControl::setup() {
  ESP_LOGI(TAG, "Setting up pid_control");

  this->sensor_->add_on_state_callback([this](float state) {
    ESP_LOGI(TAG, "Call back from sensor");
    if (is_switch_enabled()) {
      state = 30;
      // only publish if state/current value has changed in two digits of precision
      this->do_publish_ = roundf(state * 100) != roundf(this->current_value_ * 100);
      this->current_value_ = state;
      ESP_LOGI(TAG, "Updating PID %f %f", state, target_value_);

      this->update_pid_();
    }
  });
  this->current_value_ = this->sensor_->state;
  // this->target_value_ = this->default_target_value_;
}

// if no switch is configured, then it is permanently on
// if a switch if configured, get it's state to determine
// whether it is on of off
bool PIDControl::is_switch_enabled() {
  if (this->enable_switch_ == nullptr) {
    return true;
  } else {
    if (this->enable_switch_->state == true) {
      return true;
    }
  }
  return false;
}

}  // namespace pid_control
}  // namespace esphome
