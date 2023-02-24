#include "pid_base.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pid_shared {

static const char *const TAG = "pid_base";

void PIDBase::dump_config() {
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

void PIDBase::write_output_(float value) {
  this->output_value_ = value;

  // first ensure outputs are off (both outputs not active at the same time)
  if (this->decrease_output_ != nullptr && value >= 0)
    this->decrease_output_->set_level(0.0f);
  if (this->increase_output_ != nullptr && value <= 0)
    this->increase_output_->set_level(0.0f);

  // value < 0 means decrease, > 0 means increase
  if (this->decrease_output_ != nullptr && value < 0) {
    float val = std::min(1.0f, -value);
    this->decrease_output_->set_level(val);
    ESP_LOGI(TAG, "Setting decrease_output to %f (%f)", val, value);
  }

  if (this->increase_output_ != nullptr && value > 0) {
    float val = std::min(1.0f, value);
    this->increase_output_->set_level(val);
    ESP_LOGI(TAG, "Setting increase_output to %f (%f)", val, value);
  }

  this->pid_computed_callback_.call();
}

void PIDBase::update_pid_() {
  float value;
  if (std::isnan(this->current_value_) || std::isnan(this->target_value_)) {
    // if any control parameters are nan, turn off all outputs
    value = 0.0;
  } else {
    // Update PID controller irrespective of current mode, to not mess up D/I terms
    // In non-auto mode, we just discard the output value

    value = this->controller_.update(this->target_value_, this->current_value_);
    ESP_LOGI(TAG, "In update_pid_(): %f %f -> %f", this->target_value_, this->current_value_, value);

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

  this->write_output_(value);
  // }

  // if (this->do_publish_)
  //   this->publish_state();
}

// void PIDBase::publish_state() { ESP_LOGD(TAG, "'%s' - Sending state:", this->current_value_); }

void PIDBase::start_autotune(std::unique_ptr<pid_shared::PIDAutotuner> &&autotune) {
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

void PIDBase::reset_integral_term() {
  ESP_LOGI(TAG, "Resetting integral to 0");
  this->controller_.reset_accumulated_integral();
}

}  // namespace pid_shared
}  // namespace esphome
