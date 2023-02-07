#include "pid_control.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pid_control {

static const char *const TAG = "pid";

void PID::setup() {
  this->sensor_->add_on_state_callback([this](float state) {
    // only publish if state/current value has changed in two digits of precision
    this->do_publish_ = roundf(state * 100) != roundf(this->current_value_ * 100);
    this->current_value_ = state;
    this->update_pid_();
  });
  this->current_value_ = this->sensor_->state;
  // restore set points
  // auto restore = this->restore_state_();
  // if (restore.has_value()) {
  //   restore->to_call(this).perform();
  // } else {
  // restore from defaults, change_away handles those for us
  // if (supports_heat_() && supports_cool_()) {
  //   this->mode = climate::CLIMATE_MODE_HEAT_COOL;
  // } else if (supports_cool_()) {
  //   this->mode = climate::CLIMATE_MODE_COOL;
  // } else if (supports_heat_()) {
  //   this->mode = climate::CLIMATE_MODE_HEAT;
  // }
  this->target_value_ = this->default_target_value_;
}

// void PID::control(const climate::ClimateCall &call) {
//   if (call.get_mode().has_value())
//     this->mode = *call.get_mode();
//   if (call.get_target_value().has_value())
//     this->target_value = *call.get_target_value();

//   // If switching to off mode, set output immediately
//   if (this->mode == climate::CLIMATE_MODE_OFF)
//     this->write_output_(0.0f);

//   this->publish_state();
// }
// climate::ClimateTraits PID::traits() {
//   auto traits = climate::ClimateTraits();
//   traits.set_supports_current_value(true);
//   traits.set_supports_two_point_target_value(false);

//   traits.set_supported_modes({climate::CLIMATE_MODE_OFF});
//   if (supports_cool_())
//     traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
//   if (supports_heat_())
//     traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
//   if (supports_heat_() && supports_cool_())
//     traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);

//   traits.set_supports_action(true);
//   return traits;
// }
void PID::dump_config() {
  // TODO
  // LOG_CLIMATE("", "PID", this);
  ESP_LOGCONFIG(TAG, "  Control Parameters:");
  ESP_LOGCONFIG(TAG, "    kp: %.5f, ki: %.5f, kd: %.5f, output samples: %d", controller_.kp_, controller_.ki_,
                controller_.kd_, controller_.output_samples_);

  if (controller_.threshold_low_ == 0 && controller_.threshold_high_ == 0) {
    ESP_LOGCONFIG(TAG, "  Deadband disabled.");
  } else {
    ESP_LOGCONFIG(TAG, "  Deadband Parameters:");
    ESP_LOGCONFIG(TAG, "    threshold: %0.5f to %0.5f, multipliers(kp: %.5f, ki: %.5f, kd: %.5f), output samples: %d",
                  controller_.threshold_low_, controller_.threshold_high_, controller_.kp_multiplier_,
                  controller_.ki_multiplier_, controller_.kd_multiplier_, controller_.deadband_output_samples_);
  }

  if (this->autotuner_ != nullptr) {
    this->autotuner_->dump_config();
  }
}
void PID::write_output_(float value) {
  this->output_value_ = value;

  // // first ensure outputs are off (both outputs not active at the same time)
  // if (this->supports_cool_() && value >= 0)
  //   this->cool_output_->set_level(0.0f);
  // if (this->supports_heat_() && value <= 0)
  //   this->heat_output_->set_level(0.0f);

  // // value < 0 means cool, > 0 means heat
  // if (this->supports_cool_() && value < 0)
  //   this->cool_output_->set_level(std::min(1.0f, -value));
  // if (this->supports_heat_() && value > 0)
  //   this->heat_output_->set_level(std::min(1.0f, value));

  // // Update action variable for user feedback what's happening
  // climate::ClimateAction new_action;
  // if (this->supports_cool_() && value < 0) {
  //   new_action = climate::CLIMATE_ACTION_COOLING;
  // } else if (this->supports_heat_() && value > 0) {
  //   new_action = climate::CLIMATE_ACTION_HEATING;
  // } else if (this->mode == climate::CLIMATE_MODE_OFF) {
  //   new_action = climate::CLIMATE_ACTION_OFF;
  // } else {
  //   new_action = climate::CLIMATE_ACTION_IDLE;
  // }
  // if (new_action != this->action) {
  //   this->action = new_action;
  //   this->do_publish_ = true;
  // }

  this->pid_computed_callback_.call();
}

void PID::update_pid_() {
  float value;
  if (std::isnan(this->current_value_) || std::isnan(this->target_value_)) {
    // if any control parameters are nan, turn off all outputs
    value = 0.0;
  } else {
    // Update PID controller irrespective of current mode, to not mess up D/I terms
    // In non-auto mode, we just discard the output value
    value = this->controller_.update(this->target_value_, this->current_value_);

    // Check autotuner
    if (this->autotuner_ != nullptr && !this->autotuner_->is_finished()) {
      auto res = this->autotuner_->update(this->target_value_, this->current_value_);
      if (res.result_params.has_value()) {
        this->controller_.kp_ = res.result_params->kp;
        this->controller_.ki_ = res.result_params->ki;
        this->controller_.kd_ = res.result_params->kd;
        // keep autotuner instance so that subsequent dump_configs will print the long result message.
      } else {
        value = res.output;
      }
    }
  }

  // if (this->mode == climate::CLIMATE_MODE_OFF) {
  //   this->write_output_(0.0);
  // } else {
  this->write_output_(value);
  // }

  if (this->do_publish_)
    this->publish_state();
}

void PID::publish_state() { ESP_LOGD(TAG, "'%s' - Sending state:", this->current_value_); }

void PID::start_autotune(std::unique_ptr<pid_shared::PIDAutotuner> &&autotune) {
  this->autotuner_ = std::move(autotune);
  // float min_value = this->supports_cool_() ? -1.0f : 0.0f;
  // float max_value = this->supports_heat_() ? 1.0f : 0.0f;
  this->autotuner_->config(0.0, 1.0);
  this->autotuner_->set_autotuner_id(this->get_object_id());

  std::string TAG2 = TAG + std::string(".") + this->get_object_id();

  ESP_LOGI(TAG2.c_str(),
           "Autotune has started. This can take a long time depending on the "
           "responsiveness of your system. Your system "
           "output will be altered to deliberately oscillate above and below the setpoint multiple times. "
           "Until your sensor provides a reading, the autotuner may display \'nan\'");

  this->set_interval("autotune-progress", 10000, [this]() {
    if (this->autotuner_ != nullptr && !this->autotuner_->is_finished())
      this->autotuner_->dump_config();
  });

  // if (mode != climate::CLIMATE_MODE_HEAT_COOL) {
  //   ESP_LOGW(TAG2.c_str(), "!!! For PID autotuner you need to set AUTO (also called heat/cool) mode! %s",
  //            this->get_name().c_str());
  // }
}

void PID::reset_integral_term() { this->controller_.reset_accumulated_integral(); }

}  // namespace pid_control
}  // namespace esphome
