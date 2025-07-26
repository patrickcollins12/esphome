#pragma once

#include "esphome/core/component.h"
#include "pid_controller.h"
#include "pid_autotuner.h"

namespace esphome {
namespace pid_shared {

class PIDBase {
 public:
  void set_kp(float kp) { controller_.kp_ = kp; }
  void set_ki(float ki) { controller_.ki_ = ki; }
  void set_kd(float kd) { controller_.kd_ = kd; }
  void set_min_integral(float min_integral) { controller_.min_integral_ = min_integral; }
  void set_max_integral(float max_integral) { controller_.max_integral_ = max_integral; }
  void set_output_samples(int in) { controller_.output_samples_ = in; }
  void set_derivative_samples(int in) { controller_.derivative_samples_ = in; }
  void set_threshold_low(float in) { controller_.threshold_low_ = in; }
  void set_threshold_high(float in) { controller_.threshold_high_ = in; }
  void set_kp_multiplier(float in) { controller_.kp_multiplier_ = in; }
  void set_ki_multiplier(float in) { controller_.ki_multiplier_ = in; }
  void set_kd_multiplier(float in) { controller_.kd_multiplier_ = in; }
  void set_starting_integral_term(float in) { controller_.set_starting_integral_term(in); }
  void set_deadband_output_samples(int in) { controller_.deadband_output_samples_ = in; }

  void start_autotune(std::unique_ptr<pid_shared::PIDAutotuner> &&autotune);
  void reset_integral_term() { controller_.reset_accumulated_integral(); }

 protected:
  void update_pid_(float state);
  virtual void write_output_(float value) = 0;

  float target_value_;
  PIDController controller_;
  std::unique_ptr<PIDAutotuner> autotuner_;
};

}  // namespace pid_shared
}  // namespace esphome
