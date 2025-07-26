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

}  // namespace pid_control
}  // namespace esphome
