#pragma once

#include "esphome/core/component.h"
#include "esphome/components/pid_shared/pid_base.h"

namespace esphome {
namespace pid_shared {

enum PIDSensorType {
  PID_SENSOR_TYPE_RESULT,
  PID_SENSOR_TYPE_ERROR,
  PID_SENSOR_TYPE_PROPORTIONAL,
  PID_SENSOR_TYPE_INTEGRAL,
  PID_SENSOR_TYPE_DERIVATIVE,
  PID_SENSOR_TYPE_HEAT,
  PID_SENSOR_TYPE_COOL,
  PID_SENSOR_TYPE_KP,
  PID_SENSOR_TYPE_KI,
  PID_SENSOR_TYPE_KD,
};

class PIDSensor : public sensor::Sensor, public Component {
 public:
  void setup() override;
  void set_parent(PIDBase *parent) { parent_ = parent; }
  void set_type(PIDSensorType type) { type_ = type; }

  void dump_config() override;

 protected:
  void update_from_parent_();
  PIDBase *parent_;
  PIDSensorType type_;
};

}  // namespace pid_shared
}  // namespace esphome
