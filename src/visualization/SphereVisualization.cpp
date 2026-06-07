#include "perceptive_legged_control/visualization/SphereVisualization.h"

#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <utility>

#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <std_msgs/msg/color_rgba.hpp>

namespace perceptive_legged_control {

namespace {
geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d& point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

std_msgs::msg::ColorRGBA redColor(float alpha) {
  std_msgs::msg::ColorRGBA color;
  color.r = 1.0;
  color.a = alpha;
  return color;
}
}  // namespace

SphereVisualization::SphereVisualization(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo centroidalModelInfo,
                                         const ocs2::PinocchioSphereInterface& sphereInterface, rclcpp::Node::SharedPtr node,
                                         ocs2::scalar_t maxUpdateFrequency)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      centroidalModelInfo_(std::move(centroidalModelInfo)),
      sphereInterface_(sphereInterface),
      node_(std::move(node)),
      markerPublisher_(node_->create_publisher<visualization_msgs::msg::MarkerArray>("sphere_markers", 1)),
      lastTime_(std::numeric_limits<ocs2::scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}

void SphereVisualization::update(const ocs2::SystemObservation& observation) {
  if (observation.time - lastTime_ <= minPublishTimeDifference_) {
    return;
  }
  lastTime_ = observation.time;

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data,
                               ocs2::centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));

  const auto positions = sphereInterface_.computeSphereCentersInWorldFrame(pinocchioInterface_);
  const auto& numSpheres = sphereInterface_.getNumSpheres();
  const auto& radii = sphereInterface_.getSphereRadii();

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker deleteAll;
  deleteAll.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(deleteAll);

  size_t sphereOffset = 0;
  for (size_t i = 0; i < numSpheres.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.id = static_cast<int>(i);
    marker.ns = "Collision Spheres";
    marker.header.stamp = node_->now();
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color = redColor(0.5f);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radii[sphereOffset];
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.x;

    for (size_t j = 0; j < numSpheres[i]; ++j) {
      marker.points.push_back(toPointMsg(positions[sphereOffset + j]));
    }
    sphereOffset += numSpheres[i];
    markers.markers.push_back(marker);
  }

  markerPublisher_->publish(markers);
}

}  // namespace perceptive_legged_control
