#include "perceptive_legged_control/constraint/FootPlacementConstraint.h"

#include "perceptive_legged_control/interface/PerceptiveLeggedPrecomputation.h"
#include "perceptive_legged_control/interface/PerceptiveSwitchedModelReferenceManager.h"

namespace perceptive_legged_control {

FootPlacementConstraint::FootPlacementConstraint(const ocs2::legged_robot::SwitchedModelReferenceManager& referenceManager,
                                                 const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                                                 size_t contactPointIndex, size_t numVertices)
    : StateConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      numVertices_(numVertices) {}

FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint& rhs)
    : StateConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      numVertices_(rhs.numVertices_) {}

bool FootPlacementConstraint::isActive(ocs2::scalar_t time) const {
  return dynamic_cast<const PerceptiveSwitchedModelReferenceManager&>(*referenceManagerPtr_).getFootPlacementFlags(time)[contactPointIndex_];
}

ocs2::vector_t FootPlacementConstraint::getValue(ocs2::scalar_t, const ocs2::vector_t& state,
                                                 const ocs2::PreComputation& preComp) const {
  const auto param = ocs2::cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
  return param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b;
}

ocs2::VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(
    ocs2::scalar_t, const ocs2::vector_t& state, const ocs2::PreComputation& preComp) const {
  ocs2::VectorFunctionLinearApproximation approx = ocs2::VectorFunctionLinearApproximation::Zero(numVertices_, state.size(), 0);
  const auto param = ocs2::cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];

  const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  approx.f = param.a * positionApprox.f + param.b;
  approx.dfdx = param.a * positionApprox.dfdx;
  return approx;
}

}  // namespace perceptive_legged_control
