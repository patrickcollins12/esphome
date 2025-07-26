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

template<typename... Ts> class PIDAutotuneAction : public Action<Ts...> {
 public:
  PIDAutotuneAction(PIDBase *parent) : parent_(parent) {}

  void set_noiseband(float noiseband) { noiseband_ = noiseband; }
  void set_positive_output(float positive_output) { positive_output_ = positive_output; }
  void set_negative_output(float negative_output) { negative_output_ = negative_output; }

  void play(Ts... x) {
    auto tuner = make_unique<pid_shared::PIDAutotuner>();
    tuner->set_noiseband(this->noiseband_);
    tuner->set_output_negative(this->negative_output_);
    tuner->set_output_positive(this->positive_output_);
    this->parent_->start_autotune(std::move(tuner));
  }

 protected:
  float noiseband_;
  float positive_output_;
  float negative_output_;
  PIDBase *parent_;
};

template<typename... Ts> class PIDResetIntegralTermAction : public Action<Ts...> {
 public:
  PIDResetIntegralTermAction(PIDBase *parent) : parent_(parent) {}

  void play(Ts... x) { this->parent_->reset_integral_term(); }

 protected:
  PIDBase *parent_;
};

template<typename... Ts> class PIDSetControlParametersAction : public Action<Ts...> {
 public:
  PIDSetControlParametersAction(PIDBase *parent) : parent_(parent) {}

  void play(Ts... x) {
    auto kp = this->kp_.value(x...);
    auto ki = this->ki_.value(x...);
    auto kd = this->kd_.value(x...);

    this->parent_->set_kp(kp);
    this->parent_->set_ki(ki);
    this->parent_->set_kd(kd);
  }

 protected:
  TEMPLATABLE_VALUE(float, kp)
  TEMPLATABLE_VALUE(float, ki)
  TEMPLATABLE_VALUE(float, kd)

  PIDBase *parent_;
};

}  // namespace pid_shared
}  // namespace esphome
