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

}  // namespace pid
}  // namespace esphome
