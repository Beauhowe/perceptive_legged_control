#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <legged_interface/LeggedRobotPreComputation.h>

#include "perceptive_legged_control/constraint/FootPlacementConstraint.h"
#include "perceptive_legged_control/interface/ConvexRegionSelector.h"

namespace perceptive_legged_control {

class PerceptiveLeggedPrecomputation : public ocs2::legged_robot::LeggedRobotPreComputation {
 public:
  PerceptiveLeggedPrecomputation(ocs2::PinocchioInterface pinocchioInterface, const ocs2::CentroidalModelInfo& info,
                                 const ocs2::legged_robot::SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                 ocs2::legged_robot::ModelSettings settings, const ConvexRegionSelector& convexRegionSelector);
  ~PerceptiveLeggedPrecomputation() override = default;

  PerceptiveLeggedPrecomputation* clone() const override { return new PerceptiveLeggedPrecomputation(*this); }

  void request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) override;

  const std::vector<FootPlacementConstraint::Parameter>& getFootPlacementConParameters() const { return footPlacementConParameters_; }

  PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs);

 private:
  std::pair<ocs2::matrix_t, ocs2::vector_t> getPolygonConstraint(const convex_plane_decomposition::CgalPolygon2d& polygon) const;

  const ConvexRegionSelector* convexRegionSelectorPtr_;
  std::vector<FootPlacementConstraint::Parameter> footPlacementConParameters_;
};

}  // namespace perceptive_legged_control
