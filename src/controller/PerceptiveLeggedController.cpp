#include "perceptive_legged_control/controller/PerceptiveLeggedController.h"

#include <pluginlib/class_list_macros.hpp>

#include "perceptive_legged_control/interface/PerceptiveLeggedInterface.h"
#include "perceptive_legged_control/interface/PerceptiveSwitchedModelReferenceManager.h"
#include "perceptive_legged_control/synchronized_module/PlanarTerrainReceiver.h"

namespace perceptive_legged_control {

void PerceptiveLeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                                      const std::string& referenceFile, bool verbose) {
  leggedInterface_ = std::make_shared<PerceptiveLeggedInterface>(taskFile, urdfFile, referenceFile, rosNode_);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
  setupVisualization();
}

void PerceptiveLeggedController::setupMpc() {
  legged::LeggedController::setupMpc();

  auto perceptiveInterface = std::dynamic_pointer_cast<PerceptiveLeggedInterface>(leggedInterface_);
  if (!perceptiveInterface) {
    RCLCPP_ERROR(rosNode_->get_logger(), "PerceptiveLeggedController requires PerceptiveLeggedInterface.");
    return;
  }

  std::string terrainTopic = "/convex_plane_decomposition_ros/planar_terrain";
  std::string elevationLayer = "elevation";
  rosNode_->declare_parameter<std::string>("perceptive_terrain_topic", terrainTopic);
  rosNode_->declare_parameter<std::string>("perceptive_sdf_elevation_layer", elevationLayer);
  rosNode_->get_parameter("perceptive_terrain_topic", terrainTopic);
  rosNode_->get_parameter("perceptive_sdf_elevation_layer", elevationLayer);

  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(rosNode_, perceptiveInterface->getPlanarTerrainPtr(),
                                             perceptiveInterface->getSignedDistanceFieldPtr(), terrainTopic, elevationLayer);
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

void PerceptiveLeggedController::setupVisualization() {
  auto perceptiveReferenceManager =
      std::dynamic_pointer_cast<PerceptiveSwitchedModelReferenceManager>(leggedInterface_->getSwitchedModelReferenceManagerPtr());
  if (!perceptiveReferenceManager) {
    RCLCPP_WARN(rosNode_->get_logger(), "Foot placement visualization requires PerceptiveSwitchedModelReferenceManager.");
    return;
  }
  footPlacementVisualizationPtr_ = std::make_shared<FootPlacementVisualization>(
      *perceptiveReferenceManager->getConvexRegionSelectorPtr(), leggedInterface_->getCentroidalModelInfo().numThreeDofContacts,
      rosNode_);
#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
  auto perceptiveInterface = std::dynamic_pointer_cast<PerceptiveLeggedInterface>(leggedInterface_);
  if (!perceptiveInterface || !perceptiveInterface->getPinocchioSphereInterfacePtr()) {
    RCLCPP_WARN(rosNode_->get_logger(), "Sphere visualization requires PerceptiveLeggedInterface sphere interface.");
    return;
  }
  sphereVisualizationPtr_ = std::make_shared<SphereVisualization>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
      *perceptiveInterface->getPinocchioSphereInterfacePtr(), rosNode_);
#endif
}

controller_interface::return_type PerceptiveLeggedController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  const auto result = legged::LeggedController::update(time, period);
  if (result == controller_interface::return_type::OK) {
    if (footPlacementVisualizationPtr_) {
      footPlacementVisualizationPtr_->update(currentObservation_);
    }
#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
    if (sphereVisualizationPtr_) {
      sphereVisualizationPtr_->update(currentObservation_);
    }
#endif
  }
  return result;
}

}  // namespace perceptive_legged_control

PLUGINLIB_EXPORT_CLASS(perceptive_legged_control::PerceptiveLeggedController, controller_interface::ControllerInterface)
