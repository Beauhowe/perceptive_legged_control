#include "perceptive_legged_control/interface/PlanarSignedDistanceField.h"

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
  hasMap_ = gridMap_.exists(layer_);
}

ocs2::scalar_t PlanarSignedDistanceField::value(const ocs2::legged_robot::vector3_t& position) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const grid_map::Position position2d(position.x(), position.y());
  return position.z() - heightAt(position2d, 0.0);
}

ocs2::legged_robot::vector3_t PlanarSignedDistanceField::derivative(const ocs2::legged_robot::vector3_t& position) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!hasMap_) {
    return (ocs2::legged_robot::vector3_t() << 0.0, 0.0, 1.0).finished();
  }

  const double step = std::max(0.03, gridMap_.getResolution());
  const grid_map::Position center(position.x(), position.y());
  const auto hCenter = heightAt(center, 0.0);
  const auto hxMinus = heightAt(grid_map::Position(position.x() - step, position.y()), hCenter);
  const auto hxPlus = heightAt(grid_map::Position(position.x() + step, position.y()), hCenter);
  const auto hyMinus = heightAt(grid_map::Position(position.x(), position.y() - step), hCenter);
  const auto hyPlus = heightAt(grid_map::Position(position.x(), position.y() + step), hCenter);

  ocs2::legged_robot::vector3_t gradient;
  gradient << -(hxPlus - hxMinus) / (2.0 * step), -(hyPlus - hyMinus) / (2.0 * step), 1.0;
  const auto norm = gradient.norm();
  return norm > 1e-6 ? gradient / norm : (ocs2::legged_robot::vector3_t() << 0.0, 0.0, 1.0).finished();
}

ocs2::scalar_t PlanarSignedDistanceField::heightAt(const grid_map::Position& position, ocs2::scalar_t fallback) const {
  if (!hasMap_ || !gridMap_.isInside(position)) {
    return fallback;
  }
  const auto height = gridMap_.atPosition(layer_, position, grid_map::InterpolationMethods::INTER_NEAREST);
  return std::isfinite(height) ? static_cast<ocs2::scalar_t>(height) : fallback;
}

}  // namespace perceptive_legged_control
