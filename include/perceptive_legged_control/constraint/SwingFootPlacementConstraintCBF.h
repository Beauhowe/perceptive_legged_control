#pragma once

#include <memory>

#include <legged_interface/SwitchedModelReferenceManager.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace perceptive_legged_control {

class SwingFootPlacementConstraintCBF final : public ocs2::StateInputConstraint {
 public:
  SwingFootPlacementConstraintCBF(const ocs2::legged_robot::SwitchedModelReferenceManager& referenceManager,
                                  const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                                  size_t contactPointIndex, size_t numVertices, ocs2::scalar_t cbfLambda,
                                  std::unique_ptr<ocs2::PenaltyBase> penaltyFunction);

  ~SwingFootPlacementConstraintCBF() override = default;
  SwingFootPlacementConstraintCBF* clone() const override { return new SwingFootPlacementConstraintCBF(*this); }

  bool isActive(ocs2::scalar_t time) const override;
  size_t getNumConstraints(ocs2::scalar_t) const override { return numVertices_; }

  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                          const ocs2::PreComputation& preComp) const override;
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::vector_t& input,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  SwingFootPlacementConstraintCBF(const SwingFootPlacementConstraintCBF& rhs);

  const ocs2::legged_robot::SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;
  const size_t contactPointIndex_;
  const size_t numVertices_;
  ocs2::scalar_t lambda_;
  std::unique_ptr<ocs2::PenaltyBase> penaltyPtr_;
};

}  // namespace perceptive_legged_control
