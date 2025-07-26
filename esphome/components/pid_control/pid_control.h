#pragma once

#include "esphome/core/component.h"
#include "esphome/components/pid_shared/pid_base.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace pid_control {

// This is the implementation of a generic, non-climate PID controller.
// It inherits from the shared PIDBase and is designed to control any sensor/output
// combination. It works by taking a sensor input and controlling two separate outputs:
// one for increasing the value (e.g., heating) and one for decreasing it (e.g., cooling).
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

  // Pointers to the sensor and outputs this PID controller will manage.
  std::string name_;
  sensor::Sensor *sensor_;
  output::FloatOutput *increase_output_{nullptr};
  output::FloatOutput *decrease_output_{nullptr};
  switch_::Switch *enable_switch_{nullptr};
};

}  // namespace pid_control
}  // namespace esphome
