#pragma once

#include <mutex>
#include <string>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <grid_map_core/GridMap.hpp>
#include <rclcpp/rclcpp.hpp>

namespace perceptive_legged_control {

class PerceptiveTerrainBuffer {
 public:
  explicit PerceptiveTerrainBuffer(rclcpp::Node::SharedPtr node);

  double heightAt(double x, double y, double fallbackHeight) const;

 private:
  void terrainCallback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr terrainSubscriber_;

  mutable std::mutex mutex_;
  convex_plane_decomposition::PlanarTerrain latestTerrain_;
  bool hasTerrain_{false};
  std::string terrainTopic_{"/convex_plane_decomposition_ros/planar_terrain"};
  std::string heightLayer_{"smooth_planar"};
  double heightOffset_{0.0};
};

}  // namespace perceptive_legged_control
