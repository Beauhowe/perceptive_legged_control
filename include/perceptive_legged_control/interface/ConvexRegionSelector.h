#pragma once

#include <memory>
#include <vector>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace perceptive_legged_control {

class ConvexRegionSelector {
 public:
  ConvexRegionSelector(ocs2::CentroidalModelInfo info,
                       std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                       const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics, size_t numVertices);

  void update(const ocs2::ModeSchedule& modeSchedule, ocs2::scalar_t initTime, const ocs2::vector_t& initState,
              ocs2::TargetTrajectories& targetTrajectories);

  convex_plane_decomposition::PlanarTerrainProjection getProjection(size_t leg, ocs2::scalar_t time) const;
  convex_plane_decomposition::CgalPolygon2d getConvexPolygon(size_t leg, ocs2::scalar_t time) const;
  ocs2::legged_robot::vector3_t getNominalFootholds(size_t leg, ocs2::scalar_t time) const;

  std::vector<ocs2::scalar_t> getMiddleTimes(size_t leg) const { return middleTimes_[leg]; }
  std::vector<convex_plane_decomposition::PlanarTerrainProjection> getProjections(size_t leg) const { return feetProjections_[leg]; }
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const { return planarTerrainPtr_; }
  ocs2::legged_robot::feet_array_t<ocs2::scalar_t> getInitStandFinalTimes() const { return initStandFinalTime_; }

  ocs2::legged_robot::feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

 private:
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);
  ocs2::legged_robot::vector3_t getNominalFoothold(size_t leg, ocs2::scalar_t time, const ocs2::vector_t& initState,
                                     ocs2::TargetTrajectories& targetTrajectories);

  ocs2::legged_robot::feet_array_t<std::vector<convex_plane_decomposition::PlanarTerrainProjection>> feetProjections_;
  ocs2::legged_robot::feet_array_t<std::vector<convex_plane_decomposition::CgalPolygon2d>> convexPolygons_;
  ocs2::legged_robot::feet_array_t<std::vector<ocs2::legged_robot::vector3_t>> nominalFootholds_;
  ocs2::legged_robot::feet_array_t<std::vector<ocs2::scalar_t>> middleTimes_;
  ocs2::legged_robot::feet_array_t<ocs2::scalar_t> initStandFinalTime_;
  ocs2::legged_robot::feet_array_t<std::vector<ocs2::scalar_t>> timeEvents_;

  const ocs2::CentroidalModelInfo info_;
  size_t numVertices_;
  convex_plane_decomposition::PlanarTerrain planarTerrain_;
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;
};

}  // namespace perceptive_legged_control
