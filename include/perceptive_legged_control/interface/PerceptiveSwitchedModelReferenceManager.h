#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include <grid_map_core/GridMap.hpp>
#include <legged_interface/SwitchedModelReferenceManager.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot/gait/LegLogic.h>

#include "perceptive_legged_control/interface/ConvexRegionSelector.h"

namespace perceptive_legged_control {

class PerceptiveSwitchedModelReferenceManager : public ocs2::legged_robot::SwitchedModelReferenceManager {
 public:
  PerceptiveSwitchedModelReferenceManager(ocs2::CentroidalModelInfo info,
                                          std::shared_ptr<ocs2::legged_robot::GaitSchedule> gaitSchedulePtr,
                                          std::shared_ptr<ocs2::legged_robot::SwingTrajectoryPlanner> swingTrajectoryPtr,
                                          std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                          const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                                          ocs2::scalar_t comHeight, ocs2::scalar_t swingHeightAlongLineScale = 1.0);

  const std::shared_ptr<ConvexRegionSelector>& getConvexRegionSelectorPtr() const { return convexRegionSelectorPtr_; }
  ocs2::legged_robot::contact_flag_t getFootPlacementFlags(ocs2::scalar_t time) const;
  ocs2::legged_robot::contact_flag_t getSwingFootPlacementFlags(ocs2::scalar_t time) const;
  ocs2::legged_robot::feet_array_t<ocs2::legged_robot::LegPhase> ContactPhasePerLeg(ocs2::scalar_t time) const;
  ocs2::legged_robot::feet_array_t<ocs2::legged_robot::LegPhase> SwingPhasePerLeg(ocs2::scalar_t time) const;

 protected:
  void modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t& initState,
                        ocs2::TargetTrajectories& targetTrajectories, ocs2::ModeSchedule& modeSchedule) override;

  void updateSwingTrajectoryPlanner(ocs2::scalar_t initTime, const ocs2::vector_t& initState, ocs2::ModeSchedule& modeSchedule,
                                    const grid_map::GridMap& map);

  void modifyProjections(ocs2::scalar_t initTime, const ocs2::vector_t& initState, size_t leg, size_t initIndex,
                         const std::vector<bool>& contactFlagStocks,
                         std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections);

  std::tuple<ocs2::scalar_array_t, ocs2::scalar_array_t, ocs2::scalar_array_t> getHeights(
      const std::vector<bool>& contactFlagStocks,
      const std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections, const grid_map::GridMap& map) const;

 private:
  ocs2::scalar_t mapHeight(const grid_map::GridMap& map, const grid_map::Position& position, ocs2::scalar_t fallback,
                           const std::string& preferredLayer = "smooth_planar") const;
  ocs2::scalar_t maxHeightAlongLine(const grid_map::GridMap& map, const grid_map::Position& startPoint,
                                    const grid_map::Position& endPoint, ocs2::scalar_t fallback) const;

  const ocs2::CentroidalModelInfo info_;
  ocs2::legged_robot::feet_array_t<ocs2::legged_robot::vector3_t> lastLiftoffPos_;
  std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;
  ocs2::scalar_t comHeight_;
  ocs2::scalar_t swingHeightAlongLineScale_{1.0};
  ocs2::TargetTrajectories relativeTargetTrajectories_;
  ocs2::TargetTrajectories lastAdaptedTargetTrajectories_;
  bool hasCachedTargetTrajectories_{false};
};

}  // namespace perceptive_legged_control
