#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/automation.h"
// #include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/pid_shared/pid_controller.h"
#include "esphome/components/pid_shared/pid_autotuner.h"

namespace esphome {
namespace pid_shared {

class PIDBase : public Component, public EntityBase {
 public:
  PIDBase() = default;
  virtual ~PIDBase() {}
  void setup() {} // to be defined in the derived class but needs to be here
  void dump_config();

  // void publish_state();

  void set_sensor(sensor::Sensor *sensor) { sensor_ = sensor; }
  void set_decrease_output(output::FloatOutput *decrease_output) { decrease_output_ = decrease_output; }
  void set_increase_output(output::FloatOutput *increase_output) { increase_output_ = increase_output; }
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

  float get_output_value() const { return output_value_; }
  float get_error_value() const { return controller_.error_; }
  float get_kp() { return controller_.kp_; }
  float get_ki() { return controller_.ki_; }
  float get_kd() { return controller_.kd_; }
  float get_proportional_term() const { return controller_.proportional_term_; }
  float get_integral_term() const { return controller_.integral_term_; }
  float get_derivative_term() const { return controller_.derivative_term_; }
  int get_output_samples() { return controller_.output_samples_; }
  int get_derivative_samples() { return controller_.derivative_samples_; }

  float get_threshold_low() { return controller_.threshold_low_; }
  float get_threshold_high() { return controller_.threshold_high_; }
  float get_kp_multiplier() { return controller_.kp_multiplier_; }
  float get_ki_multiplier() { return controller_.ki_multiplier_; }
  float get_kd_multiplier() { return controller_.kd_multiplier_; }
  int get_deadband_output_samples() { return controller_.deadband_output_samples_; }
  bool in_deadband() { return controller_.in_deadband(); }

  // int get_derivative_samples() const { return controller_.derivative_samples; }
  // float get_deadband() const { return controller_.deadband; }
  // float get_proportional_deadband_multiplier() const { return controller_.proportional_deadband_multiplier; }

  void add_on_pid_computed_callback(std::function<void()> &&callback) {
    pid_computed_callback_.add(std::move(callback));
  }
  void set_default_target_value(float default_target_value) { default_target_value_ = default_target_value; }
  void start_autotune(std::unique_ptr<pid_shared::PIDAutotuner> &&autotune);
  void reset_integral_term();

 protected:
  void update_pid_();

  // bool supports_cool_() const { return this->decrease_output_ != nullptr; }
  // bool supports_heat_() const { return this->increase_output_ != nullptr; }

  void write_output_(float value);

  /// The sensor used for getting the current value
  sensor::Sensor *sensor_;
  output::FloatOutput *decrease_output_{nullptr};
  output::FloatOutput *increase_output_{nullptr};
  pid_shared::PIDController controller_;

  /// Output value as reported by the PID controller, for PIDSensor
  float output_value_;
  CallbackManager<void()> pid_computed_callback_;
  float default_target_value_;
  float target_value_;
  float current_value_;
  std::unique_ptr<pid_shared::PIDAutotuner> autotuner_;
  bool do_publish_ = false;
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
