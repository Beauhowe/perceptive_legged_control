#include "perceptive_legged_control/interface/PerceptiveSwitchedModelReferenceManager.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <grid_map_core/iterators/LineIterator.hpp>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>

namespace perceptive_legged_control {

using namespace ocs2;
using namespace ocs2::legged_robot;

PerceptiveSwitchedModelReferenceManager::PerceptiveSwitchedModelReferenceManager(
    CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
    std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr, const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    scalar_t locomotionComHeight)
    : SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
      info_(std::move(info)),
      convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      locomotionComHeight_(locomotionComHeight) {
  lastLiftoffPos_.fill(vector3_t::Zero());
}

void PerceptiveSwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                               TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
  if (map.exists("smooth_planar")) {
    const TargetTrajectories* relativeTargetTrajectories = &targetTrajectories;
    if (hasCachedTargetTrajectories_ && targetTrajectories == lastAdaptedTargetTrajectories_) {
      relativeTargetTrajectories = &relativeTargetTrajectories_;
    } else {
      relativeTargetTrajectories_ = targetTrajectories;
      relativeTargetTrajectories = &relativeTargetTrajectories_;
    }

    TargetTrajectories newTargetTrajectories;
    const scalar_t finalRelativeHeight =
        centroidal_model::getBasePose(relativeTargetTrajectories->getDesiredState(finalTime), info_)(2);
    const bool compensateLocomotionHeight = finalRelativeHeight >= 0.9 * locomotionComHeight_;
    const size_t nodeNum = 11;
    for (size_t i = 0; i < nodeNum; ++i) {
      const scalar_t time = initTime + static_cast<scalar_t>(i) * timeHorizon / static_cast<scalar_t>(nodeNum - 1);
      vector_t state = relativeTargetTrajectories->getDesiredState(time);
      vector_t input = relativeTargetTrajectories->getDesiredInput(time);

      vector3_t pos = centroidal_model::getBasePose(state, info_).head(3);
      const grid_map::Position position2d(pos.x(), pos.y());
      const scalar_t terrainZ = mapHeight(map, position2d, 0.0);

      const scalar_t step = 0.3;
      grid_map::Vector3 normalVector;
      normalVector(0) = (mapHeight(map, grid_map::Position(pos.x() - step, pos.y()), terrainZ) -
                         mapHeight(map, grid_map::Position(pos.x() + step, pos.y()), terrainZ)) /
                        (2.0 * step);
      normalVector(1) = (mapHeight(map, grid_map::Position(pos.x(), pos.y() - step), terrainZ) -
                         mapHeight(map, grid_map::Position(pos.x(), pos.y() + step), terrainZ)) /
                        (2.0 * step);
      normalVector(2) = 1.0;
      normalVector.normalize();

      matrix3_t R;
      const scalar_t yaw = centroidal_model::getBasePose(state, info_)(3);
      R << std::cos(yaw), -std::sin(yaw), 0.0, std::sin(yaw), std::cos(yaw), 0.0, 0.0, 0.0, 1.0;
      const vector3_t bodyNormal = R.transpose() * normalVector;
      const scalar_t pitch = std::atan2(bodyNormal.x(), bodyNormal.z());
      const scalar_t requestedRelativeHeight =
          std::isfinite(pos.z()) && pos.z() > 0.0 ? pos.z() : locomotionComHeight_;
      centroidal_model::getBasePose(state, info_)(4) = pitch;
      const scalar_t heightScale = compensateLocomotionHeight ? 1.0 / std::max(std::cos(pitch), 0.5) : 1.0;
      centroidal_model::getBasePose(state, info_)(2) = terrainZ + requestedRelativeHeight * heightScale;

      newTargetTrajectories.timeTrajectory.push_back(time);
      newTargetTrajectories.stateTrajectory.push_back(state);
      newTargetTrajectories.inputTrajectory.push_back(input);
    }
    targetTrajectories = std::move(newTargetTrajectories);
    lastAdaptedTargetTrajectories_ = targetTrajectories;
    hasCachedTargetTrajectories_ = true;
  }

  convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);
  updateSwingTrajectoryPlanner(initTime, initState, modeSchedule, map);
}

void PerceptiveSwitchedModelReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                           ModeSchedule& modeSchedule, const grid_map::GridMap& map) {
  const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
  feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence, swingHeightSequence;

  for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++) {
    const size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);
    auto projections = convexRegionSelectorPtr_->getProjections(leg);
    modifyProjections(initTime, initState, leg, initIndex, contactFlagStocks[leg], projections);

    scalar_array_t liftOffHeights, touchDownHeights, swingHeights;
    std::tie(liftOffHeights, touchDownHeights, swingHeights) = getHeights(contactFlagStocks[leg], projections, map);
    liftOffHeightSequence[leg] = liftOffHeights;
    touchDownHeightSequence[leg] = touchDownHeights;
    swingHeightSequence[leg] = swingHeights;
  }
  swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, swingHeightSequence);
}

void PerceptiveSwitchedModelReferenceManager::modifyProjections(
    scalar_t initTime, const vector_t& initState, size_t leg, size_t initIndex, const std::vector<bool>& contactFlagStocks,
    std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections) {
  if (projections.empty() || initIndex >= projections.size()) {
    return;
  }

  if (contactFlagStocks[initIndex]) {
    lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
    lastLiftoffPos_[leg].z() -= 0.02;
    for (size_t i = initIndex; i < projections.size(); ++i) {
      if (!contactFlagStocks[i]) {
        break;
      }
      projections[i].positionInWorld = lastLiftoffPos_[leg];
    }
    for (int i = static_cast<int>(initIndex); i >= 0; --i) {
      if (!contactFlagStocks[i]) {
        break;
      }
      projections[i].positionInWorld = lastLiftoffPos_[leg];
    }
  }

  if (initTime > convexRegionSelectorPtr_->getInitStandFinalTimes()[leg]) {
    for (int i = static_cast<int>(initIndex); i >= 0; --i) {
      if (contactFlagStocks[i]) {
        projections[i].positionInWorld = lastLiftoffPos_[leg];
      }
      if (i + 1 < static_cast<int>(contactFlagStocks.size()) && !contactFlagStocks[i] && !contactFlagStocks[i + 1]) {
        break;
      }
    }
  }
}

std::tuple<scalar_array_t, scalar_array_t, scalar_array_t> PerceptiveSwitchedModelReferenceManager::getHeights(
    const std::vector<bool>& contactFlagStocks,
    const std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections, const grid_map::GridMap& map) const {
  const size_t numPhases = projections.size();
  scalar_array_t liftOffHeights(numPhases, 0.0), touchDownHeights(numPhases, 0.0), swingHeights(numPhases, 0.0);
  std::vector<vector3_t> liftOffProjections(numPhases, vector3_t::Zero());
  std::vector<vector3_t> touchDownProjections(numPhases, vector3_t::Zero());

  for (size_t i = 1; i < numPhases; ++i) {
    if (!contactFlagStocks[i]) {
      liftOffHeights[i] = contactFlagStocks[i - 1] ? projections[i - 1].positionInWorld.z() : liftOffHeights[i - 1];
      liftOffProjections[i] = contactFlagStocks[i - 1] ? projections[i - 1].positionInWorld : liftOffProjections[i - 1];
    }
  }
  for (int i = static_cast<int>(numPhases) - 2; i >= 0; --i) {
    if (!contactFlagStocks[i]) {
      touchDownHeights[i] = contactFlagStocks[i + 1] ? projections[i + 1].positionInWorld.z() : touchDownHeights[i + 1];
      touchDownProjections[i] = contactFlagStocks[i + 1] ? projections[i + 1].positionInWorld : touchDownProjections[i + 1];
    }
  }

  for (size_t i = 0; i < numPhases; ++i) {
    if (!contactFlagStocks[i]) {
      const scalar_t fallback = std::max(liftOffHeights[i], touchDownHeights[i]);
      const scalar_t maxHeight = maxHeightAlongLine(map, liftOffProjections[i].head(2), touchDownProjections[i].head(2), fallback);
      swingHeights[i] = maxHeight * 1.05;
    } else {
      swingHeights[i] = projections[i].positionInWorld.z();
    }
  }

  return {liftOffHeights, touchDownHeights, swingHeights};
}

scalar_t PerceptiveSwitchedModelReferenceManager::mapHeight(const grid_map::GridMap& map, const grid_map::Position& position,
                                                            scalar_t fallback, const std::string& preferredLayer) const {
  std::string layer = preferredLayer;
  if (!map.exists(layer)) {
    layer = map.exists("elevation") ? "elevation" : std::string();
  }
  if (layer.empty() || !map.isInside(position)) {
    return fallback;
  }
  const scalar_t height = map.atPosition(layer, position, grid_map::InterpolationMethods::INTER_NEAREST);
  return std::isfinite(height) ? height : fallback;
}

scalar_t PerceptiveSwitchedModelReferenceManager::maxHeightAlongLine(const grid_map::GridMap& map, const grid_map::Position& startPoint,
                                                                     const grid_map::Position& endPoint, scalar_t fallback) const {
  const std::string layer = map.exists("elevation") ? "elevation" : (map.exists("smooth_planar") ? "smooth_planar" : std::string());
  if (layer.empty() || !map.isInside(startPoint) || !map.isInside(endPoint)) {
    return fallback;
  }

  scalar_t maxValue = std::max(mapHeight(map, startPoint, fallback, layer), mapHeight(map, endPoint, fallback, layer));
  for (grid_map::LineIterator iterator(map, startPoint, endPoint); !iterator.isPastEnd(); ++iterator) {
    const auto cellValue = map.at(layer, *iterator);
    if (std::isfinite(cellValue)) {
      maxValue = std::max(maxValue, static_cast<scalar_t>(cellValue));
    }
  }
  return maxValue;
}

ocs2::legged_robot::contact_flag_t PerceptiveSwitchedModelReferenceManager::getFootPlacementFlags(ocs2::scalar_t time) const {
  ocs2::legged_robot::contact_flag_t flag;
  const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
  for (size_t i = 0; i < flag.size(); ++i) {
    flag[i] = getContactFlags(time)[i] && time >= finalTime[i];
  }
  return flag;
}

ocs2::legged_robot::contact_flag_t PerceptiveSwitchedModelReferenceManager::getSwingFootPlacementFlags(ocs2::scalar_t time) const {
  ocs2::legged_robot::contact_flag_t flag;
  const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
  for (size_t i = 0; i < flag.size(); ++i) {
    flag[i] = !getContactFlags(time)[i] && time >= finalTime[i];
  }
  return flag;
}

ocs2::legged_robot::feet_array_t<ocs2::legged_robot::LegPhase> PerceptiveSwitchedModelReferenceManager::ContactPhasePerLeg(
    ocs2::scalar_t time) const {
  return ocs2::legged_robot::getContactPhasePerLeg(time, getModeSchedule());
}

ocs2::legged_robot::feet_array_t<ocs2::legged_robot::LegPhase> PerceptiveSwitchedModelReferenceManager::SwingPhasePerLeg(
    ocs2::scalar_t time) const {
  return ocs2::legged_robot::getSwingPhasePerLeg(time, getModeSchedule());
}

}  // namespace perceptive_legged_control
