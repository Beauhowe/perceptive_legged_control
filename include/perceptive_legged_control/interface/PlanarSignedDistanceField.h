#pragma once

#include <mutex>
#include <string>

#include <grid_map_core/GridMap.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/Types.h>

namespace perceptive_legged_control {

class PlanarSignedDistanceField {
 public:
  PlanarSignedDistanceField() = default;
  PlanarSignedDistanceField(const grid_map::GridMap& gridMap, std::string layer);

  void update(const grid_map::GridMap& gridMap, std::string layer);
  ocs2::scalar_t value(const ocs2::legged_robot::vector3_t& position) const;
  ocs2::legged_robot::vector3_t derivative(const ocs2::legged_robot::vector3_t& position) const;

 private:
  ocs2::scalar_t heightAt(const grid_map::Position& position, ocs2::scalar_t fallback) const;

  mutable std::mutex mutex_;
  grid_map::GridMap gridMap_;
  std::string layer_{"elevation"};
  bool hasMap_{false};
};

}  // namespace perceptive_legged_control
