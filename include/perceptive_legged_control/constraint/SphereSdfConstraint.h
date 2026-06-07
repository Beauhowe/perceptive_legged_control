#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematics.h>

#include "perceptive_legged_control/interface/PlanarSignedDistanceField.h"

namespace perceptive_legged_control {

class SphereSdfConstraint final : public ocs2::StateConstraint {
 public:
  SphereSdfConstraint(const ocs2::PinocchioSphereKinematics& sphereKinematics,
                      std::shared_ptr<PlanarSignedDistanceField> signedDistanceFieldPtr);
  ~SphereSdfConstraint() override = default;

  SphereSdfConstraint* clone() const override { return new SphereSdfConstraint(*this); }

  size_t getNumConstraints(ocs2::scalar_t) const override { return numConstraints_; }

  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
                          const ocs2::PreComputation& preComp) const override;
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& preComp) const override;

 private:
  SphereSdfConstraint(const SphereSdfConstraint& rhs);

  std::unique_ptr<ocs2::PinocchioSphereKinematics> sphereKinematicsPtr_;
  std::shared_ptr<PlanarSignedDistanceField> signedDistanceFieldPtr_;
  size_t numConstraints_{};
};

}  // namespace perceptive_legged_control
