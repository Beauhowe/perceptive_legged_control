#pragma once

#include <memory>
#include <vector>

#include <ocs2_mpc/SystemObservation.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "perceptive_legged_control/interface/ConvexRegionSelector.h"

namespace perceptive_legged_control {

class FootPlacementVisualization {
 public:
  FootPlacementVisualization(const ConvexRegionSelector& convexRegionSelector, size_t numFoot, rclcpp::Node::SharedPtr node,
                             ocs2::scalar_t maxUpdateFrequency = 20.0);

  void update(const ocs2::SystemObservation& observation);

 private:
  enum class Color { blue = 0, orange, yellow, purple };

  visualization_msgs::msg::Marker to3dRosMarker(const convex_plane_decomposition::CgalPolygon2d& polygon,
                                                const Eigen::Isometry3d& transformPlaneToWorld,
                                                const std_msgs::msg::Header& header, Color color, float alpha, size_t id) const;
  visualization_msgs::msg::Marker arrowAtPoint(const ocs2::legged_robot::vector3_t& direction,
                                               const ocs2::legged_robot::vector3_t& position,
                                               const std_msgs::msg::Header& header, Color color, float alpha, size_t id) const;
  visualization_msgs::msg::Marker footMarker(const ocs2::legged_robot::vector3_t& position, const std_msgs::msg::Header& header,
                                             Color color, float alpha, size_t id) const;
  std_msgs::msg::ColorRGBA colorMsg(Color color, float alpha) const;

  ocs2::scalar_t lineWidth_{0.008};
  ocs2::scalar_t footMarkerDiameter_{0.02};
  std::vector<Color> feetColorMap_{Color::blue, Color::orange, Color::yellow, Color::purple};

  const ConvexRegionSelector& convexRegionSelector_;
  size_t numFoot_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher_;
  ocs2::scalar_t lastTime_;
  ocs2::scalar_t minPublishTimeDifference_;
};

}  // namespace perceptive_legged_control
