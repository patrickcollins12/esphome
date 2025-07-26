#include "pid_base.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pid_shared {

static const char *const TAG = "pid.base";

void PIDBase::update_pid_(float state) {
  if (this->autotuner_ != nullptr) {
    auto result = this->autotuner_->update(this->target_value_, state);
    if (result.result_params.has_value()) {
      this->set_kp(result.result_params->kp);
      this->set_ki(result.result_params->ki);
      this->set_kd(result.result_params->kd);
      this->autotuner_.reset();
    }
    this->write_output_(result.output);
  } else {
    float output = this->controller_.update(this->target_value_, state);
    this->write_output_(output);
  }
}

void PIDBase::start_autotune(std::unique_ptr<pid_shared::PIDAutotuner> &&autotune) {
  this->autotuner_ = std::move(autotune);
}

}  // namespace pid_shared
}  // namespace esphome
