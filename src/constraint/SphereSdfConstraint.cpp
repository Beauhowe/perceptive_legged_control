#include "perceptive_legged_control/constraint/SphereSdfConstraint.h"

#include "perceptive_legged_control/interface/PerceptiveLeggedPrecomputation.h"

namespace perceptive_legged_control {

SphereSdfConstraint::SphereSdfConstraint(const ocs2::PinocchioSphereKinematics& sphereKinematics,
                                         std::shared_ptr<PlanarSignedDistanceField> signedDistanceFieldPtr)
    : StateConstraint(ocs2::ConstraintOrder::Linear),
      sphereKinematicsPtr_(sphereKinematics.clone()),
      signedDistanceFieldPtr_(std::move(signedDistanceFieldPtr)),
      numConstraints_(sphereKinematicsPtr_->getPinocchioSphereInterface().getNumSpheresInTotal()) {}

ocs2::vector_t SphereSdfConstraint::getValue(ocs2::scalar_t, const ocs2::vector_t& state,
                                             const ocs2::PreComputation& preComp) const {
  ocs2::vector_t value(numConstraints_);

  sphereKinematicsPtr_->setPinocchioInterface(ocs2::cast<PerceptiveLeggedPrecomputation>(preComp).getPinocchioInterface());
  const auto positions = sphereKinematicsPtr_->getPosition(state);
  const auto& radii = sphereKinematicsPtr_->getPinocchioSphereInterface().getSphereRadii();
  for (size_t i = 0; i < numConstraints_; ++i) {
    value(i) = signedDistanceFieldPtr_->value(positions[i]) - radii[i];
  }
  return value;
}

ocs2::VectorFunctionLinearApproximation SphereSdfConstraint::getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                                    const ocs2::PreComputation& preComp) const {
  ocs2::VectorFunctionLinearApproximation approx = ocs2::VectorFunctionLinearApproximation::Zero(numConstraints_, state.size(), 0);
  approx.f = getValue(time, state, preComp);

  const auto positions = sphereKinematicsPtr_->getPosition(state);
  const auto sphereApprox = sphereKinematicsPtr_->getPositionLinearApproximation(state);
  for (size_t i = 0; i < numConstraints_; ++i) {
    const ocs2::vector_t sdfGradient = signedDistanceFieldPtr_->derivative(positions[i]);
    approx.dfdx.row(i) = sdfGradient.transpose() * sphereApprox[i].dfdx;
  }
  return approx;
}

SphereSdfConstraint::SphereSdfConstraint(const SphereSdfConstraint& rhs)
    : StateConstraint(rhs),
      sphereKinematicsPtr_(rhs.sphereKinematicsPtr_->clone()),
      signedDistanceFieldPtr_(rhs.signedDistanceFieldPtr_),
      numConstraints_(rhs.numConstraints_) {}

}  // namespace perceptive_legged_control
