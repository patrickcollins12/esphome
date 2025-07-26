#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/pid_shared/pid_base.h"

namespace esphome {
namespace pid {

class PIDClimate : public climate::Climate, public Component, public pid_shared::PIDBase {
 public:
  void setup() override;
  void dump_config() override;

  void set_sensor(sensor::Sensor *sensor) { sensor_ = sensor; }
  void set_cool_output(output::FloatOutput *cool_output) { cool_output_ = cool_output; }
  void set_heat_output(output::FloatOutput *heat_output) { heat_output_ = heat_output; }
  void set_default_target_temperature(float default_target_temperature) {
    default_target_temperature_ = default_target_temperature;
  }

 protected:
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;
  void write_output_(float value) override;

  sensor::Sensor *sensor_;
  output::FloatOutput *cool_output_{nullptr};
  output::FloatOutput *heat_output_{nullptr};
  float default_target_temperature_;
};

template<typename... Ts> class PIDAutotuneAction : public Action<Ts...> {
 public:
  PIDAutotuneAction(PIDClimate *parent) : parent_(parent) {}

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
  PIDClimate *parent_;
};

template<typename... Ts> class PIDResetIntegralTermAction : public Action<Ts...> {
 public:
  PIDResetIntegralTermAction(PIDClimate *parent) : parent_(parent) {}

  void play(Ts... x) { this->parent_->reset_integral_term(); }

 protected:
  PIDClimate *parent_;
};

template<typename... Ts> class PIDSetControlParametersAction : public Action<Ts...> {
 public:
  PIDSetControlParametersAction(PIDClimate *parent) : parent_(parent) {}

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

  PIDClimate *parent_;
};

}  // namespace pid
}  // namespace esphome
