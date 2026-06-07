#include "perceptive_legged_control/synchronized_module/PlanarTerrainReceiver.h"

#include <cmath>
#include <utility>

#include <convex_plane_decomposition_ros/MessageConversion.h>

namespace perceptive_legged_control {

PlanarTerrainReceiver::PlanarTerrainReceiver(rclcpp::Node::SharedPtr node,
                                             std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                                             std::shared_ptr<PlanarSignedDistanceField> signedDistanceFieldPtr,
                                             const std::string& mapTopic, std::string elevationLayer)
    : node_(std::move(node)),
      elevationLayer_(std::move(elevationLayer)),
      planarTerrainPtr_(std::move(planarTerrainPtr)),
      sdfPtr_(std::move(signedDistanceFieldPtr)) {
  subscriber_ = node_->create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
      mapTopic, rclcpp::SystemDefaultsQoS(), std::bind(&PlanarTerrainReceiver::planarTerrainCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "PlanarTerrainReceiver subscribing to %s (SDF/elevation layer: %s).", mapTopic.c_str(),
              elevationLayer_.c_str());
}

void PlanarTerrainReceiver::preSolverRun(ocs2::scalar_t, ocs2::scalar_t, const ocs2::vector_t&, const ocs2::ReferenceManagerInterface&) {
  if (updated_) {
    std::lock_guard<std::mutex> lock(mutex_);
    updated_ = false;
    *planarTerrainPtr_ = planarTerrain_;
    if (sdfPtr_) {
      sdfPtr_->update(planarTerrainPtr_->gridMap, elevationLayer_);
    }
  }
}

void PlanarTerrainReceiver::planarTerrainCallback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  planarTerrain_ = convex_plane_decomposition::fromMessage(*msg);

  if (planarTerrain_.gridMap.exists(elevationLayer_)) {
    auto& elevationData = planarTerrain_.gridMap.get(elevationLayer_);
    if (elevationData.hasNaN()) {
      const float inpaint = elevationData.minCoeffOfFinites();
      RCLCPP_WARN(node_->get_logger(), "[PlanarTerrainReceiver] Map contains NaN values. Applying inpainting with min value.");
      elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
    }
  }

  updated_ = true;
}

}  // namespace perceptive_legged_control
