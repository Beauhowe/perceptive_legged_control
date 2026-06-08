#include "perceptive_legged_control/controller/PerceptiveLeggedCheaterController.h"

#include <legged_estimation/FromTopiceEstimate.h>
#include <pluginlib/class_list_macros.hpp>

namespace perceptive_legged_control {

void PerceptiveLeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<legged::FromTopicStateEstimate>(rosNode_, leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

}  // namespace perceptive_legged_control

PLUGINLIB_EXPORT_CLASS(perceptive_legged_control::PerceptiveLeggedCheaterController, controller_interface::ControllerInterface)
