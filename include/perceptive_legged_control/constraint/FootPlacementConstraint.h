#pragma once

#include <legged_interface/SwitchedModelReferenceManager.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace perceptive_legged_control {

class FootPlacementConstraint final : public ocs2::StateConstraint {
 public:
  struct Parameter {
    ocs2::matrix_t a;
    ocs2::vector_t b;
  };

  FootPlacementConstraint(const ocs2::legged_robot::SwitchedModelReferenceManager& referenceManager,
                          const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics, size_t contactPointIndex,
                          size_t numVertices);

  ~FootPlacementConstraint() override = default;
  FootPlacementConstraint* clone() const override { return new FootPlacementConstraint(*this); }

  bool isActive(ocs2::scalar_t time) const override;
  size_t getNumConstraints(ocs2::scalar_t) const override { return numVertices_; }
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& preComp) const override;
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  FootPlacementConstraint(const FootPlacementConstraint& rhs);

  const ocs2::legged_robot::SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;
  const size_t contactPointIndex_;
  const size_t numVertices_;
};

}  // namespace perceptive_legged_control
