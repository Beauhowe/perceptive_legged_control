#pragma once

#include <memory>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace perceptive_legged_control {

class SphereVisualization {
 public:
  SphereVisualization(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo centroidalModelInfo,
                      const ocs2::PinocchioSphereInterface& sphereInterface, rclcpp::Node::SharedPtr node,
                      ocs2::scalar_t maxUpdateFrequency = 20.0);

  void update(const ocs2::SystemObservation& observation);

 private:
  ocs2::PinocchioInterface pinocchioInterface_;
  ocs2::CentroidalModelInfo centroidalModelInfo_;
  const ocs2::PinocchioSphereInterface& sphereInterface_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher_;
  ocs2::scalar_t lastTime_;
  ocs2::scalar_t minPublishTimeDifference_;
};

}  // namespace perceptive_legged_control
