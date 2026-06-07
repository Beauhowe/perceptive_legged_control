#include "perceptive_legged_control/constraint/SwingFootPlacementConstraintCBF.h"

#include "perceptive_legged_control/interface/PerceptiveLeggedPrecomputation.h"
#include "perceptive_legged_control/interface/PerceptiveSwitchedModelReferenceManager.h"

namespace perceptive_legged_control {

SwingFootPlacementConstraintCBF::SwingFootPlacementConstraintCBF(
    const ocs2::legged_robot::SwitchedModelReferenceManager& referenceManager,
    const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics, size_t contactPointIndex, size_t numVertices,
    ocs2::scalar_t cbfLambda, std::unique_ptr<ocs2::PenaltyBase> penaltyFunction)
    : StateInputConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      numVertices_(numVertices),
      lambda_(cbfLambda),
      penaltyPtr_(std::move(penaltyFunction)) {}

SwingFootPlacementConstraintCBF::SwingFootPlacementConstraintCBF(const SwingFootPlacementConstraintCBF& rhs)
    : StateInputConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      numVertices_(rhs.numVertices_),
      lambda_(rhs.lambda_),
      penaltyPtr_(rhs.penaltyPtr_->clone()) {}

bool SwingFootPlacementConstraintCBF::isActive(ocs2::scalar_t time) const {
  return dynamic_cast<const PerceptiveSwitchedModelReferenceManager&>(*referenceManagerPtr_)
      .getSwingFootPlacementFlags(time)[contactPointIndex_];
}

ocs2::vector_t SwingFootPlacementConstraintCBF::getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                         const ocs2::vector_t& input,
                                                         const ocs2::PreComputation& preComp) const {
  const auto swingPhase = dynamic_cast<const PerceptiveSwitchedModelReferenceManager&>(*referenceManagerPtr_)
                              .SwingPhasePerLeg(time)[contactPointIndex_];
  const auto duration = 1.0 / swingPhase.duration;
  const auto phasePenalty = penaltyPtr_->getValue(time, swingPhase.phase);
  const auto phasePenaltyDerivative = penaltyPtr_->getDerivative(time, swingPhase.phase) * duration;
  (void)phasePenaltyDerivative;

  const auto param = ocs2::cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
  const auto position = endEffectorKinematicsPtr_->getPosition(state).front();
  const auto velocity = endEffectorKinematicsPtr_->getVelocity(state, input).front();
  const ocs2::vector_t hs = param.a * position + param.b + phasePenalty * ocs2::vector_t::Ones(param.b.size());
  const ocs2::vector_t dotHs = param.a * velocity + phasePenaltyDerivative * ocs2::vector_t::Ones(param.b.size());
  (void)dotHs;

  return hs;
}

ocs2::VectorFunctionLinearApproximation SwingFootPlacementConstraintCBF::getLinearApproximation(
    ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input, const ocs2::PreComputation& preComp) const {
  const auto swingPhase = dynamic_cast<const PerceptiveSwitchedModelReferenceManager&>(*referenceManagerPtr_)
                              .SwingPhasePerLeg(time)[contactPointIndex_];
  const auto duration = 1.0 / swingPhase.duration;
  const auto phasePenalty = penaltyPtr_->getValue(time, swingPhase.phase);
  const auto phasePenaltyDerivative = penaltyPtr_->getDerivative(time, swingPhase.phase) * duration;
  (void)phasePenaltyDerivative;

  const auto param = ocs2::cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
  const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
  (void)velocityApprox;

  ocs2::VectorFunctionLinearApproximation approx = ocs2::VectorFunctionLinearApproximation::Zero(numVertices_, state.size(), input.size());
  approx.f = param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b +
             phasePenalty * ocs2::vector_t::Ones(param.b.size());
  approx.dfdx = param.a * positionApprox.dfdx;
  return approx;
}

}  // namespace perceptive_legged_control
