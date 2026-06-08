#include "perceptive_legged_control/interface/PlanarSignedDistanceField.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace perceptive_legged_control {

PlanarSignedDistanceField::PlanarSignedDistanceField(const grid_map::GridMap& gridMap, std::string layer) {
  update(gridMap, std::move(layer));
}

void PlanarSignedDistanceField::update(const grid_map::GridMap& gridMap, std::string layer) {
  std::lock_guard<std::mutex> lock(mutex_);
  gridMap_ = gridMap;
  layer_ = std::move(layer);
  if (!gridMap_.exists(layer_) && gridMap_.exists("elevation")) {
    layer_ = "elevation";
  }
  if (!gridMap_.exists(layer_)) {
    hasMap_ = false;
    return;
  }

  auto& elevationData = gridMap_.get(layer_);
  if (elevationData.hasNaN()) {
    const float inpaint = elevationData.minCoeffOfFinites();
    if (!std::isfinite(inpaint)) {
      hasMap_ = false;
      return;
    }
    elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
  }

  const float maxElevation = elevationData.maxCoeffOfFinites();
  if (!std::isfinite(maxElevation)) {
    hasMap_ = false;
    return;
  }

  const double heightClearance = std::max(0.3, static_cast<double>(maxElevation) + 0.3);
  sdf_.calculateSignedDistanceField(gridMap_, layer_, heightClearance);
  hasMap_ = true;
}

ocs2::scalar_t PlanarSignedDistanceField::value(const ocs2::legged_robot::vector3_t& position) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!hasMap_) {
    return position.z();
  }
  return static_cast<ocs2::scalar_t>(sdf_.getDistanceAt(position));
}

ocs2::legged_robot::vector3_t PlanarSignedDistanceField::derivative(const ocs2::legged_robot::vector3_t& position) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!hasMap_) {
    return (ocs2::legged_robot::vector3_t() << 0.0, 0.0, 1.0).finished();
  }

  const ocs2::legged_robot::vector3_t gradient = sdf_.getDistanceGradientAt(position);
  return gradient.allFinite() ? gradient : (ocs2::legged_robot::vector3_t() << 0.0, 0.0, 1.0).finished();
}

}  // namespace perceptive_legged_control
