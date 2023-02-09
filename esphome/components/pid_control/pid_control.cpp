#include "pid_control.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pid_control {

static const char *const TAG = "pid_control";

void PIDControl::setup() {
  this->sensor_->add_on_state_callback([this](float state) {
    // only publish if state/current value has changed in two digits of precision
    this->do_publish_ = roundf(state * 100) != roundf(this->current_value_ * 100);
    this->current_value_ = state;
    // TODO:: only update if enabled (not disabled)
    this->update_pid_();
  });
  this->current_value_ = this->sensor_->state;
  this->target_value_ = this->default_target_value_;
}

}  // namespace pid_control
}  // namespace esphome
