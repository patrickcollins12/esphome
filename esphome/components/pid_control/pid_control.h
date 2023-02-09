#pragma once

#include "esphome/components/pid_shared/pid_base.h"

namespace esphome {
namespace pid_control {

class PIDControl : public pid_shared::PIDBase {
 public:
  // PIDControl() = default;
  // ~PIDControl() {}
  void setup();
};

}  // namespace pid_control
}  // namespace esphome
