#pragma once

#include "esphome/core/component.h"
#include "esphome/components/pid_shared/pid_base.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace pid_control {

class PIDControl : public pid_shared::PIDBase, public Component {
 public:
  void setup() override;
  void loop() override;

  void set_name(const std::string &name) { name_ = name; }
  void set_sensor(sensor::Sensor *sensor) { sensor_ = sensor; }
  void set_increase_output(output::FloatOutput *increase_output) { increase_output_ = increase_output; }
  void set_decrease_output(output::FloatOutput *decrease_output) { decrease_output_ = decrease_output; }
  void set_enable_switch(switch_::Switch *enable_switch) { enable_switch_ = enable_switch; }
  void set_target_value(float target_value) { this->target_value_ = target_value; }

 protected:
  bool is_pid_enabled_();
  void write_output_(float value) override;

  std::string name_;
  sensor::Sensor *sensor_;
  output::FloatOutput *increase_output_{nullptr};
  output::FloatOutput *decrease_output_{nullptr};
  switch_::Switch *enable_switch_{nullptr};
};

template<typename... Ts> class PIDAutotuneAction : public Action<Ts...> {
 public:
  PIDAutotuneAction(PIDControl *parent) : parent_(parent) {}

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
  PIDControl *parent_;
};

template<typename... Ts> class PIDResetIntegralTermAction : public Action<Ts...> {
 public:
  PIDResetIntegralTermAction(PIDControl *parent) : parent_(parent) {}

  void play(Ts... x) { this->parent_->reset_integral_term(); }

 protected:
  PIDControl *parent_;
};

template<typename... Ts> class PIDSetControlParametersAction : public Action<Ts...> {
 public:
  PIDSetControlParametersAction(PIDControl *parent) : parent_(parent) {}

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

  PIDControl *parent_;
};

}  // namespace pid_control
}  // namespace esphome
