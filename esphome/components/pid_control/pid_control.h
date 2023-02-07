#pragma once

#include "esphome/components/pid_shared/pid_base.h"

namespace esphome {
namespace pid_control {

class PID : public pid_shared::PIDBase {
 public:
  PID() = default;
};

}  // namespace pid_control
}  // namespace esphome
