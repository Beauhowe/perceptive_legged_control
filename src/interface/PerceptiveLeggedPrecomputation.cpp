#include "perceptive_legged_control/interface/PerceptiveLeggedPrecomputation.h"

#include <utility>

namespace perceptive_legged_control {

PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(
    ocs2::PinocchioInterface pinocchioInterface, const ocs2::CentroidalModelInfo& info,
    const ocs2::legged_robot::SwingTrajectoryPlanner& swingTrajectoryPlanner, ocs2::legged_robot::ModelSettings settings,
    const ConvexRegionSelector& convexRegionSelector)
    : ocs2::legged_robot::LeggedRobotPreComputation(std::move(pinocchioInterface), info, swingTrajectoryPlanner, std::move(settings)),
      convexRegionSelectorPtr_(&convexRegionSelector) {
  footPlacementConParameters_.resize(info.numThreeDofContacts);
}

PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs)
    : ocs2::legged_robot::LeggedRobotPreComputation(rhs), convexRegionSelectorPtr_(rhs.convexRegionSelectorPtr_) {
  footPlacementConParameters_.resize(rhs.footPlacementConParameters_.size());
}

void PerceptiveLeggedPrecomputation::request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x,
                                             const ocs2::vector_t& u) {
  if (!request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
    return;
  }
  ocs2::legged_robot::LeggedRobotPreComputation::request(request, t, x, u);

  if (request.contains(ocs2::Request::Constraint)) {
    for (size_t i = 0; i < footPlacementConParameters_.size(); i++) {
      FootPlacementConstraint::Parameter params;
      const auto projection = convexRegionSelectorPtr_->getProjection(i, t);
      if (projection.regionPtr == nullptr) {
        continue;
      }

      ocs2::matrix_t polytopeA;
      ocs2::vector_t polytopeB;
      std::tie(polytopeA, polytopeB) = getPolygonConstraint(convexRegionSelectorPtr_->getConvexPolygon(i, t));
      const ocs2::matrix_t p = (ocs2::matrix_t(2, 3) << 1, 0, 0, 0, 1, 0).finished();
      params.a = polytopeA * p * projection.regionPtr->transformPlaneToWorld.inverse().linear();
      params.b = polytopeB + polytopeA * projection.regionPtr->transformPlaneToWorld.inverse().translation().head(2);

      footPlacementConParameters_[i] = params;
    }
  }
}

std::pair<ocs2::matrix_t, ocs2::vector_t> PerceptiveLeggedPrecomputation::getPolygonConstraint(
    const convex_plane_decomposition::CgalPolygon2d& polygon) const {
  const size_t numVertices = polygon.size();
  ocs2::matrix_t polytopeA = ocs2::matrix_t::Zero(numVertices, 2);
  ocs2::vector_t polytopeB = ocs2::vector_t::Zero(numVertices);

  for (size_t i = 0; i < numVertices; i++) {
    size_t j = i + 1;
    if (j == numVertices) {
      j = 0;
    }
    size_t k = j + 1;
    if (k == numVertices) {
      k = 0;
    }
    const auto pointA = polygon.vertex(i);
    const auto pointB = polygon.vertex(j);
    const auto pointC = polygon.vertex(k);

    polytopeA.row(i) << pointB.y() - pointA.y(), pointA.x() - pointB.x();
    polytopeB(i) = pointA.y() * pointB.x() - pointA.x() * pointB.y();
    if (polytopeA.row(i) * (ocs2::vector_t(2) << pointC.x(), pointC.y()).finished() + polytopeB(i) < 0) {
      polytopeA.row(i) *= -1;
      polytopeB(i) *= -1;
    }
  }

  return {polytopeA, polytopeB};
}

}  // namespace perceptive_legged_control
