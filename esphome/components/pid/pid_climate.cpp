#include "pid_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pid {

static const char *const TAG = "pid.climate";

void PIDClimate::setup() {
  this->sensor_->add_on_state_callback([this](float state) {
    if (this->mode == climate::CLIMATE_MODE_OFF) {
      this->write_output_(0.0f);
    } else {
      this->target_value_ = this->target_temperature;
      this->update_pid_(state);
    }
  });
  this->target_temperature = this->default_target_temperature_;
}

void PIDClimate::dump_config() {
  LOG_CLIMATE("", "PID Climate", this);
  // Implementation of dump_config
}

climate::ClimateTraits PIDClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_current_temperature(true);
  traits.set_supports_cool_mode(this->cool_output_ != nullptr);
  traits.set_supports_heat_mode(this->heat_output_ != nullptr);
  traits.set_supports_auto_mode(this->cool_output_ != nullptr && this->heat_output_ != nullptr);
  traits.set_supports_two_point_target_temperature(false);
  return traits;
}

void PIDClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    this->mode = *call.get_mode();
  }
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
  }
}

void PIDClimate::write_output_(float value) {
  if (this->cool_output_ != nullptr) {
    this->cool_output_->set_level(value < 0.0f ? -value : 0.0f);
  }
  if (this->heat_output_ != nullptr) {
    this->heat_output_->set_level(value > 0.0f ? value : 0.0f);
  }
}

}  // namespace pid
}  // namespace esphome
