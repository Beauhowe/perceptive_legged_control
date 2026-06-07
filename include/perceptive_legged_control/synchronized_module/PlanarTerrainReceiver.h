#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <rclcpp/rclcpp.hpp>

#include "perceptive_legged_control/interface/PlanarSignedDistanceField.h"

namespace perceptive_legged_control {

class PlanarTerrainReceiver : public ocs2::SolverSynchronizedModule {
 public:
  PlanarTerrainReceiver(rclcpp::Node::SharedPtr node, std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                        std::shared_ptr<PlanarSignedDistanceField> signedDistanceFieldPtr, const std::string& mapTopic,
                        std::string elevationLayer);

  void preSolverRun(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t& currentState,
                    const ocs2::ReferenceManagerInterface& referenceManager) override;
  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override {}

 private:
  void planarTerrainCallback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr subscriber_;
  convex_plane_decomposition::PlanarTerrain planarTerrain_;
  std::string elevationLayer_;
  std::mutex mutex_;
  std::atomic_bool updated_{false};
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<PlanarSignedDistanceField> sdfPtr_;
};

}  // namespace perceptive_legged_control
