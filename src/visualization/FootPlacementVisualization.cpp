#include "perceptive_legged_control/visualization/FootPlacementVisualization.h"

#include <limits>
#include <string>

#include <convex_plane_decomposition/SegmentedPlaneProjection.h>

namespace perceptive_legged_control {

namespace {
geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d& point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}
}  // namespace

FootPlacementVisualization::FootPlacementVisualization(const ConvexRegionSelector& convexRegionSelector, size_t numFoot,
                                                       rclcpp::Node::SharedPtr node, ocs2::scalar_t maxUpdateFrequency)
    : convexRegionSelector_(convexRegionSelector),
      numFoot_(numFoot),
      node_(std::move(node)),
      markerPublisher_(node_->create_publisher<visualization_msgs::msg::MarkerArray>("foot_placement", 1)),
      lastTime_(std::numeric_limits<ocs2::scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}

void FootPlacementVisualization::update(const ocs2::SystemObservation& observation) {
  if (observation.time - lastTime_ <= minPublishTimeDifference_) {
    return;
  }
  lastTime_ = observation.time;

  std_msgs::msg::Header header;
  header.stamp = node_->now();
  header.frame_id = "odom";

  visualization_msgs::msg::MarkerArray markerArray;
  visualization_msgs::msg::Marker deleteAll;
  deleteAll.action = visualization_msgs::msg::Marker::DELETEALL;
  markerArray.markers.push_back(deleteAll);

  size_t id = 0;
  for (size_t leg = 0; leg < numFoot_; ++leg) {
    const auto middleTimes = convexRegionSelector_.getMiddleTimes(leg);
    size_t startIndex = 0;
    for (size_t k = 0; k < middleTimes.size(); ++k) {
      const auto projection = convexRegionSelector_.getProjection(leg, middleTimes[k]);
      if (projection.regionPtr == nullptr) {
        continue;
      }
      if (middleTimes[k] < observation.time) {
        startIndex = k + 1;
        continue;
      }

      const auto color = feetColorMap_[leg % feetColorMap_.size()];
      const auto denominator = std::max<size_t>(1, middleTimes.size() - startIndex);
      const float alpha = 1.0f - static_cast<float>(k - startIndex) / static_cast<float>(denominator);

      markerArray.markers.push_back(arrowAtPoint(
          projection.regionPtr->transformPlaneToWorld.linear() * ocs2::legged_robot::vector3_t(0.0, 0.0, 0.1),
          projection.positionInWorld, header, color, alpha, id));
      markerArray.markers.push_back(to3dRosMarker(convexRegionSelector_.getConvexPolygon(leg, middleTimes[k]),
                                                  projection.regionPtr->transformPlaneToWorld, header, color, alpha, id));
      markerArray.markers.push_back(footMarker(convexRegionSelector_.getNominalFootholds(leg, middleTimes[k]), header, color, alpha, id));
      ++id;
    }
  }

  markerPublisher_->publish(markerArray);
}

visualization_msgs::msg::Marker FootPlacementVisualization::to3dRosMarker(
    const convex_plane_decomposition::CgalPolygon2d& polygon, const Eigen::Isometry3d& transformPlaneToWorld,
    const std_msgs::msg::Header& header, Color color, float alpha, size_t id) const {
  visualization_msgs::msg::Marker marker;
  marker.ns = "Convex Regions";
  marker.id = static_cast<int>(id);
  marker.header = header;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = lineWidth_;
  marker.color = colorMsg(color, alpha);
  marker.pose.orientation.w = 1.0;

  if (!polygon.is_empty()) {
    marker.points.reserve(polygon.size() + 1);
    for (const auto& point : polygon) {
      marker.points.push_back(toPointMsg(convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(point, transformPlaneToWorld)));
    }
    marker.points.push_back(toPointMsg(
        convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(polygon.vertex(0), transformPlaneToWorld)));
  }
  return marker;
}

visualization_msgs::msg::Marker FootPlacementVisualization::arrowAtPoint(const ocs2::legged_robot::vector3_t& direction,
                                                                         const ocs2::legged_robot::vector3_t& position,
                                                                         const std_msgs::msg::Header& header, Color color,
                                                                         float alpha, size_t id) const {
  visualization_msgs::msg::Marker marker;
  marker.ns = "Projections";
  marker.id = static_cast<int>(id);
  marker.header = header;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color = colorMsg(color, alpha);
  marker.pose.orientation.w = 1.0;
  marker.points.push_back(toPointMsg(position));
  marker.points.push_back(toPointMsg(position + direction));
  return marker;
}

visualization_msgs::msg::Marker FootPlacementVisualization::footMarker(const ocs2::legged_robot::vector3_t& position,
                                                                       const std_msgs::msg::Header& header, Color color, float alpha,
                                                                       size_t id) const {
  visualization_msgs::msg::Marker marker;
  marker.ns = "Nominal Footholds";
  marker.id = static_cast<int>(id);
  marker.header = header;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = toPointMsg(position);
  marker.pose.orientation.w = 1.0;
  marker.scale.x = footMarkerDiameter_;
  marker.scale.y = footMarkerDiameter_;
  marker.scale.z = footMarkerDiameter_;
  marker.color = colorMsg(color, alpha);
  return marker;
}

std_msgs::msg::ColorRGBA FootPlacementVisualization::colorMsg(Color color, float alpha) const {
  std_msgs::msg::ColorRGBA msg;
  msg.a = alpha;
  switch (color) {
    case Color::blue:
      msg.b = 1.0;
      break;
    case Color::orange:
      msg.r = 1.0;
      msg.g = 0.55;
      break;
    case Color::yellow:
      msg.r = 1.0;
      msg.g = 1.0;
      break;
    case Color::purple:
      msg.r = 0.55;
      msg.b = 1.0;
      break;
  }
  return msg;
}

}  // namespace perceptive_legged_control
