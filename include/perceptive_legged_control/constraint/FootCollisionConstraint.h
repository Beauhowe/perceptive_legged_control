#pragma once

#include <memory>

#include <legged_interface/SwitchedModelReferenceManager.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "perceptive_legged_control/interface/PlanarSignedDistanceField.h"

namespace perceptive_legged_control {

class FootCollisionConstraint final : public ocs2::StateConstraint {
 public:
  FootCollisionConstraint(const ocs2::legged_robot::SwitchedModelReferenceManager& referenceManager,
                          const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                          std::shared_ptr<PlanarSignedDistanceField> sdfPtr, size_t contactPointIndex, ocs2::scalar_t clearance);

  ~FootCollisionConstraint() override = default;
  FootCollisionConstraint* clone() const override { return new FootCollisionConstraint(*this); }

  bool isActive(ocs2::scalar_t time) const override;
  size_t getNumConstraints(ocs2::scalar_t) const override { return 1; }
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& preComp) const override;
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  FootCollisionConstraint(const FootCollisionConstraint& rhs);

  const ocs2::legged_robot::SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;
  std::shared_ptr<PlanarSignedDistanceField> sdfPtr_;
  const size_t contactPointIndex_;
  const ocs2::scalar_t clearance_;
};

}  // namespace perceptive_legged_control
