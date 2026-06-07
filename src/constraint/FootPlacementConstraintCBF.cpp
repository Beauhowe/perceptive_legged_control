#include "perceptive_legged_control/constraint/FootPlacementConstraintCBF.h"

#include "perceptive_legged_control/interface/PerceptiveLeggedPrecomputation.h"
#include "perceptive_legged_control/interface/PerceptiveSwitchedModelReferenceManager.h"

namespace perceptive_legged_control {

FootPlacementConstraintCBF::FootPlacementConstraintCBF(
    const ocs2::legged_robot::SwitchedModelReferenceManager& referenceManager,
    const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics, size_t contactPointIndex, size_t numVertices,
    ocs2::scalar_t cbfLambda)
    : StateInputConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      numVertices_(numVertices),
      lambda_(cbfLambda) {}

FootPlacementConstraintCBF::FootPlacementConstraintCBF(const FootPlacementConstraintCBF& rhs)
    : StateInputConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      numVertices_(rhs.numVertices_),
      lambda_(rhs.lambda_) {}

bool FootPlacementConstraintCBF::isActive(ocs2::scalar_t time) const {
  return dynamic_cast<const PerceptiveSwitchedModelReferenceManager&>(*referenceManagerPtr_).getFootPlacementFlags(time)[contactPointIndex_];
}

ocs2::vector_t FootPlacementConstraintCBF::getValue(ocs2::scalar_t, const ocs2::vector_t& state,
                                                    const ocs2::vector_t& input,
                                                    const ocs2::PreComputation& preComp) const {
  const auto param = ocs2::cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
  const auto velocity = endEffectorKinematicsPtr_->getVelocity(state, input).front();
  const ocs2::vector_t dotHs = param.a * velocity;
  const ocs2::vector_t hs = param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b;
  return dotHs + hs * lambda_;
}

ocs2::VectorFunctionLinearApproximation FootPlacementConstraintCBF::getLinearApproximation(
    ocs2::scalar_t, const ocs2::vector_t& state, const ocs2::vector_t& input, const ocs2::PreComputation& preComp) const {
  const auto param = ocs2::cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
  const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
  const auto velocity = endEffectorKinematicsPtr_->getVelocity(state, input).front();

  const ocs2::vector_t dotHs = param.a * velocity;
  const ocs2::vector_t hs = param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b;

  ocs2::VectorFunctionLinearApproximation approx = ocs2::VectorFunctionLinearApproximation::Zero(numVertices_, state.size(), input.size());
  approx.f = dotHs + hs * lambda_;
  approx.dfdx = param.a * velocityApprox.dfdx + param.a * positionApprox.dfdx * lambda_;
  approx.dfdu = param.a * velocityApprox.dfdu;
  return approx;
}

}  // namespace perceptive_legged_control
