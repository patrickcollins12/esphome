#pragma once

#include "esphome/components/pid_shared/pid_base.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace pid_control {

class PIDControl : public pid_shared::PIDBase {
 public:
  void setup();
  // void write_state(bool state) { enabled_ = state; }
  void set_enable_switch(switch_::Switch *enable_switch) { enable_switch_ = enable_switch; }

 protected:
  bool is_switch_enabled();

  // bool enabled_ = true;
  switch_::Switch *enable_switch_;
};

}  // namespace pid_control
}  // namespace esphome
