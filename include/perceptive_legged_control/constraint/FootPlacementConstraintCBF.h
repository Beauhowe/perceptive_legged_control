#pragma once

#include <legged_interface/SwitchedModelReferenceManager.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace perceptive_legged_control {

class FootPlacementConstraintCBF final : public ocs2::StateInputConstraint {
 public:
  FootPlacementConstraintCBF(const ocs2::legged_robot::SwitchedModelReferenceManager& referenceManager,
                             const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                             size_t contactPointIndex, size_t numVertices, ocs2::scalar_t cbfLambda);

  ~FootPlacementConstraintCBF() override = default;
  FootPlacementConstraintCBF* clone() const override { return new FootPlacementConstraintCBF(*this); }

  bool isActive(ocs2::scalar_t time) const override;
  size_t getNumConstraints(ocs2::scalar_t) const override { return numVertices_; }
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                          const ocs2::PreComputation& preComp) const override;
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::vector_t& input,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  FootPlacementConstraintCBF(const FootPlacementConstraintCBF& rhs);

  const ocs2::legged_robot::SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;
  const size_t contactPointIndex_;
  const size_t numVertices_;
  ocs2::scalar_t lambda_;
};

}  // namespace perceptive_legged_control
