#include "perceptive_legged_control/PerceptiveTerrainBuffer.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>

#include <cmath>
#include <functional>
#include <utility>

namespace perceptive_legged_control {

PerceptiveTerrainBuffer::PerceptiveTerrainBuffer(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
  node_->declare_parameter<std::string>("perceptive_terrain_topic", terrainTopic_);
  node_->declare_parameter<std::string>("perceptive_height_layer", heightLayer_);
  node_->declare_parameter<double>("perceptive_height_offset", heightOffset_);
  node_->get_parameter("perceptive_terrain_topic", terrainTopic_);
  node_->get_parameter("perceptive_height_layer", heightLayer_);
  node_->get_parameter("perceptive_height_offset", heightOffset_);

  terrainSubscriber_ = node_->create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
      terrainTopic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&PerceptiveTerrainBuffer::terrainCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Perceptive terrain receiver subscribing to %s (height layer: %s).", terrainTopic_.c_str(),
              heightLayer_.c_str());
}

double PerceptiveTerrainBuffer::heightAt(double x, double y, double fallbackHeight) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!hasTerrain_) {
    return fallbackHeight;
  }

  const auto& map = latestTerrain_.gridMap;
  std::string layer = heightLayer_;
  if (!map.exists(layer)) {
    if (map.exists("elevation")) {
      layer = "elevation";
    } else {
      return fallbackHeight;
    }
  }

  const grid_map::Position position(x, y);
  if (!map.isInside(position)) {
    return fallbackHeight;
  }

  const double height = map.atPosition(layer, position, grid_map::InterpolationMethods::INTER_NEAREST);
  if (!std::isfinite(height)) {
    return fallbackHeight;
  }
  return height + heightOffset_;
}

void PerceptiveTerrainBuffer::terrainCallback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr msg) {
  auto terrain = convex_plane_decomposition::fromMessage(*msg);
  std::lock_guard<std::mutex> lock(mutex_);
  latestTerrain_ = std::move(terrain);
  hasTerrain_ = true;
}

}  // namespace perceptive_legged_control
