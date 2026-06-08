#pragma once

#include "perceptive_legged_control/controller/PerceptiveLeggedController.h"

namespace perceptive_legged_control {

class PerceptiveLeggedCheaterController : public PerceptiveLeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace perceptive_legged_control
