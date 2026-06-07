#include "perceptive_legged_control/constraint/FootCollisionConstraint.h"

#include <utility>

namespace perceptive_legged_control {

FootCollisionConstraint::FootCollisionConstraint(const ocs2::legged_robot::SwitchedModelReferenceManager& referenceManager,
                                                 const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                                                 std::shared_ptr<PlanarSignedDistanceField> sdfPtr, size_t contactPointIndex,
                                                 ocs2::scalar_t clearance)
    : StateConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      sdfPtr_(std::move(sdfPtr)),
      contactPointIndex_(contactPointIndex),
      clearance_(clearance) {}

FootCollisionConstraint::FootCollisionConstraint(const FootCollisionConstraint& rhs)
    : StateConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      sdfPtr_(rhs.sdfPtr_),
      contactPointIndex_(rhs.contactPointIndex_),
      clearance_(rhs.clearance_) {}

bool FootCollisionConstraint::isActive(ocs2::scalar_t time) const {
  const ocs2::scalar_t offset = 0.05;
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_] &&
         !referenceManagerPtr_->getContactFlags(time + 0.5 * offset)[contactPointIndex_] &&
         !referenceManagerPtr_->getContactFlags(time - offset)[contactPointIndex_];
}

ocs2::vector_t FootCollisionConstraint::getValue(ocs2::scalar_t, const ocs2::vector_t& state,
                                                 const ocs2::PreComputation&) const {
  ocs2::vector_t value(1);
  value(0) = sdfPtr_->value(endEffectorKinematicsPtr_->getPosition(state).front()) - clearance_;
  return value;
}

ocs2::VectorFunctionLinearApproximation FootCollisionConstraint::getLinearApproximation(ocs2::scalar_t time,
                                                                                        const ocs2::vector_t& state,
                                                                                        const ocs2::PreComputation& preComp) const {
  ocs2::VectorFunctionLinearApproximation approx = ocs2::VectorFunctionLinearApproximation::Zero(1, state.size(), 0);
  const auto footPosition = endEffectorKinematicsPtr_->getPosition(state).front();
  approx.f = getValue(time, state, preComp);
  approx.dfdx = sdfPtr_->derivative(footPosition).transpose() *
                endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front().dfdx;
  return approx;
}

}  // namespace perceptive_legged_control
