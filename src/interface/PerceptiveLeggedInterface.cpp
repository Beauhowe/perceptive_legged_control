#include "perceptive_legged_control/interface/PerceptiveLeggedInterface.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <utility>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematics.h>
#endif

#include "perceptive_legged_control/constraint/FootPlacementConstraint.h"
#include "perceptive_legged_control/constraint/FootPlacementConstraintCBF.h"
#include "perceptive_legged_control/constraint/SwingFootPlacementConstraintCBF.h"
#include "perceptive_legged_control/constraint/FootCollisionConstraint.h"
#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
#include "perceptive_legged_control/constraint/SphereSdfConstraint.h"
#endif
#include "perceptive_legged_control/interface/PerceptiveLeggedPrecomputation.h"

#include "perceptive_legged_control/interface/ConvexRegionSelector.h"
#include "perceptive_legged_control/interface/PerceptiveSwitchedModelReferenceManager.h"

namespace perceptive_legged_control {

PerceptiveLeggedInterface::PerceptiveLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                                     const std::string& referenceFile, rclcpp::Node::SharedPtr node,
                                                     bool useHardFrictionConeConstraint)
    : legged::LeggedInterface(taskFile, urdfFile, referenceFile, useHardFrictionConeConstraint), node_(std::move(node)) {
  node_->declare_parameter<int>("perceptive_num_vertices", static_cast<int>(numVertices_));
  int numVertices = static_cast<int>(numVertices_);
  node_->get_parameter("perceptive_num_vertices", numVertices);
  numVertices_ = static_cast<size_t>(std::max(3, numVertices));
}

void PerceptiveLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                           const std::string& referenceFile, bool verbose) {
  planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();
  initializeFlatPlanarTerrain();
  signedDistanceFieldPtr_ = std::make_shared<PlanarSignedDistanceField>(planarTerrainPtr_->gridMap, "elevation");
  legged::LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  bool enableFootPlacement = true;
  bool enableFootPlacementCBF = true;
  bool enableSwingFootPlacementCBF = true;
  bool enableFootCollision = true;
  bool enableSphereSdfConstraint = false;
  ocs2::scalar_t footPlacementCbfLambda = 0.5;
  ocs2::scalar_t swingFootPlacementCbfLambda = 1.0;
  ocs2::RelaxedBarrierPenalty::Config footPlacementPenaltyConfig(1e-2, 1e-4);
  ocs2::RelaxedBarrierPenalty::Config footPlacementCbfPenaltyConfig(1e-2, 0.005);
  ocs2::RelaxedBarrierPenalty::Config swingFootPlacementPenaltyConfig(2.0, 0.1);
  ocs2::RelaxedBarrierPenalty::Config swingFootPlacementCbfPenaltyConfig(1e-2, 1e-2);
  ocs2::RelaxedBarrierPenalty::Config footCollisionPenaltyConfig(1e-2, 0.005);
  ocs2::RelaxedBarrierPenalty::Config sphereSdfPenaltyConfig(1e-2, 0.005);
  std::tie(enableFootPlacement, enableFootPlacementCBF, footPlacementCbfLambda, footPlacementCbfPenaltyConfig,
           footPlacementPenaltyConfig) = loadFootPlacementSettings(taskFile, verbose);
  std::tie(enableSwingFootPlacementCBF, swingFootPlacementCbfLambda, swingFootPlacementCbfPenaltyConfig,
           swingFootPlacementPenaltyConfig) = loadSwingFootPlacementSettings(taskFile, verbose);
  std::tie(enableFootCollision, enableSphereSdfConstraint, footCollisionPenaltyConfig, sphereSdfPenaltyConfig) =
      loadCollisionSettings(taskFile, verbose);

#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
  std::vector<std::string> collisionLinks{"LF_calf", "RF_calf", "LH_calf", "RH_calf"};
  std::vector<double> maxExcesses{0.02, 0.02, 0.02, 0.02};
  double sphereShrinkRatio = 0.6;
  node_->declare_parameter<std::vector<std::string>>("perceptive_collision_links", collisionLinks);
  node_->declare_parameter<std::vector<double>>("perceptive_collision_max_excesses", maxExcesses);
  node_->declare_parameter<double>("perceptive_sphere_shrink_ratio", sphereShrinkRatio);
  node_->declare_parameter<bool>("perceptive_enable_sphere_sdf_constraint", enableSphereSdfConstraint);
  node_->get_parameter("perceptive_collision_links", collisionLinks);
  node_->get_parameter("perceptive_collision_max_excesses", maxExcesses);
  node_->get_parameter("perceptive_sphere_shrink_ratio", sphereShrinkRatio);
  node_->get_parameter("perceptive_enable_sphere_sdf_constraint", enableSphereSdfConstraint);

#endif

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings().contactNames3DoF[i];
    std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

    if (enableFootPlacement) {
      auto footPlacementConstraint = std::make_unique<FootPlacementConstraint>(*referenceManagerPtr_, *eeKinematicsPtr, i, numVertices_);
      problemPtr_->stateSoftConstraintPtr->add(
          footName + "_footPlacement",
          std::make_unique<ocs2::StateSoftConstraint>(std::move(footPlacementConstraint),
                                                      std::make_unique<ocs2::RelaxedBarrierPenalty>(footPlacementPenaltyConfig)));
    }

    if (enableFootPlacementCBF) {
      auto footPlacementCbfConstraint = std::make_unique<FootPlacementConstraintCBF>(*referenceManagerPtr_, *eeKinematicsPtr, i,
                                                                                    numVertices_, footPlacementCbfLambda);
      problemPtr_->softConstraintPtr->add(
          footName + "_footPlacementCBF",
          std::make_unique<ocs2::StateInputSoftConstraint>(
              std::move(footPlacementCbfConstraint), std::make_unique<ocs2::RelaxedBarrierPenalty>(footPlacementCbfPenaltyConfig)));
    }

    if (enableSwingFootPlacementCBF) {
      auto swingFootPlacementCbfConstraint = std::make_unique<SwingFootPlacementConstraintCBF>(
          *referenceManagerPtr_, *eeKinematicsPtr, i, numVertices_, swingFootPlacementCbfLambda,
          std::make_unique<ocs2::RelaxedBarrierPenalty>(swingFootPlacementPenaltyConfig));
      problemPtr_->softConstraintPtr->add(
          footName + "_swingFootPlacementCBF",
          std::make_unique<ocs2::StateInputSoftConstraint>(
              std::move(swingFootPlacementCbfConstraint),
              std::make_unique<ocs2::RelaxedBarrierPenalty>(swingFootPlacementCbfPenaltyConfig)));
    }

    if (enableFootCollision) {
      auto footCollisionConstraint =
          std::make_unique<FootCollisionConstraint>(*referenceManagerPtr_, *eeKinematicsPtr, signedDistanceFieldPtr_, i, 0.03);
      problemPtr_->stateSoftConstraintPtr->add(
          footName + "_footCollision",
          std::make_unique<ocs2::StateSoftConstraint>(std::move(footCollisionConstraint),
                                                      std::make_unique<ocs2::RelaxedBarrierPenalty>(footCollisionPenaltyConfig)));
    }
  }

#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
  std::vector<ocs2::scalar_t> collisionMaxExcesses(maxExcesses.begin(), maxExcesses.end());
  pinocchioSphereInterfacePtr_ = std::make_shared<ocs2::PinocchioSphereInterface>(*pinocchioInterfacePtr_, collisionLinks,
                                                                                  collisionMaxExcesses, sphereShrinkRatio);
  ocs2::CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo_);
  auto sphereKinematicsPtr = std::make_unique<ocs2::PinocchioSphereKinematics>(*pinocchioSphereInterfacePtr_, pinocchioMapping);
  if (enableSphereSdfConstraint) {
    auto sphereSdfConstraint = std::make_unique<SphereSdfConstraint>(*sphereKinematicsPtr, signedDistanceFieldPtr_);
    problemPtr_->stateSoftConstraintPtr->add(
        "sphereSdfConstraint",
        std::make_unique<ocs2::StateSoftConstraint>(
            std::move(sphereSdfConstraint),
            std::make_unique<ocs2::RelaxedBarrierPenalty>(sphereSdfPenaltyConfig)));
  }
#endif
}

void PerceptiveLeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& /*urdfFile*/,
                                                      const std::string& referenceFile, bool verbose) {
  auto swingTrajectoryPlanner = std::make_shared<ocs2::legged_robot::SwingTrajectoryPlanner>(
      ocs2::legged_robot::loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);

  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> eeKinematicsPtr =
      getEeKinematicsPtr(modelSettings_.contactNames3DoF, "ALL_FOOT");
  auto convexRegionSelector = std::make_shared<ConvexRegionSelector>(centroidalModelInfo_, planarTerrainPtr_, *eeKinematicsPtr, numVertices_);

  ocs2::scalar_t comHeight = 0.0;
  ocs2::loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  ocs2::scalar_t swingHeightAlongLineScale = 1.0;
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    ocs2::loadData::loadPtreeValue(pt, swingHeightAlongLineScale, "swing_trajectory_config.swingHeightAlongLineScale", verbose);
  }
  referenceManagerPtr_ = std::make_shared<PerceptiveSwitchedModelReferenceManager>(
      centroidalModelInfo_, loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner), std::move(convexRegionSelector),
      *eeKinematicsPtr, comHeight, swingHeightAlongLineScale);
}

void PerceptiveLeggedInterface::setupPreComputation(const std::string&, const std::string&, const std::string&, bool) {
  problemPtr_->preComputationPtr = std::make_unique<PerceptiveLeggedPrecomputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_,
      *dynamic_cast<PerceptiveSwitchedModelReferenceManager&>(*referenceManagerPtr_).getConvexRegionSelectorPtr());
}

void PerceptiveLeggedInterface::initializeFlatPlanarTerrain() {
  const double width = 5.0;
  const double height = 5.0;
  convex_plane_decomposition::PlanarRegion plannerRegion;
  plannerRegion.transformPlaneToWorld.setIdentity();
  plannerRegion.bbox2d = convex_plane_decomposition::CgalBbox2d(-height / 2.0, -width / 2.0, height / 2.0, width / 2.0);

  convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(height / 2.0, width / 2.0));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2.0, width / 2.0));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2.0, -width / 2.0));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(height / 2.0, -width / 2.0));
  plannerRegion.boundaryWithInset.boundary = boundary;

  convex_plane_decomposition::CgalPolygonWithHoles2d inset;
  inset.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(height / 2.0 - 0.01, width / 2.0 - 0.01));
  inset.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2.0 + 0.01, width / 2.0 - 0.01));
  inset.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2.0 + 0.01, -width / 2.0 + 0.01));
  inset.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(height / 2.0 - 0.01, -width / 2.0 + 0.01));
  plannerRegion.boundaryWithInset.insets.push_back(inset);

  planarTerrainPtr_->planarRegions.clear();
  planarTerrainPtr_->planarRegions.push_back(plannerRegion);
  planarTerrainPtr_->gridMap.setFrameId("odom");
  planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(width, height), 0.03);
  planarTerrainPtr_->gridMap.add("elevation", 0.0);
  planarTerrainPtr_->gridMap.add("smooth_planar", 0.0);
}

std::tuple<bool, bool, ocs2::scalar_t, ocs2::RelaxedBarrierPenalty::Config, ocs2::RelaxedBarrierPenalty::Config>
PerceptiveLeggedInterface::loadFootPlacementSettings(const std::string& taskFile, bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "footPlacement.";

  bool enable = true;
  bool enableCbf = true;
  ocs2::scalar_t cbfLambda = 0.5;
  ocs2::RelaxedBarrierPenalty::Config cbfPenaltyConfig(1e-2, 0.005);
  ocs2::RelaxedBarrierPenalty::Config statePenaltyConfig(1e-2, 1e-4);

  ocs2::loadData::loadPtreeValue(pt, enable, prefix + "enable", verbose);
  ocs2::loadData::loadPtreeValue(pt, enableCbf, prefix + "enableCBF", verbose);
  ocs2::loadData::loadPtreeValue(pt, cbfLambda, prefix + "cbfLambda", verbose);
  ocs2::loadData::loadPtreeValue(pt, cbfPenaltyConfig.mu, prefix + "cbfPenaltyMuParam", verbose);
  ocs2::loadData::loadPtreeValue(pt, cbfPenaltyConfig.delta, prefix + "cbfPenaltyDeltaParam", verbose);
  ocs2::loadData::loadPtreeValue(pt, statePenaltyConfig.mu, prefix + "statePenaltyMuParam", verbose);
  ocs2::loadData::loadPtreeValue(pt, statePenaltyConfig.delta, prefix + "statePenaltyDeltaParam", verbose);

  return {enable, enableCbf, cbfLambda, cbfPenaltyConfig, statePenaltyConfig};
}

std::tuple<bool, ocs2::scalar_t, ocs2::RelaxedBarrierPenalty::Config, ocs2::RelaxedBarrierPenalty::Config>
PerceptiveLeggedInterface::loadSwingFootPlacementSettings(const std::string& taskFile, bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "footPlacement.";

  bool enableCbf = true;
  ocs2::scalar_t cbfLambda = 1.0;
  ocs2::RelaxedBarrierPenalty::Config cbfPenaltyConfig(1e-2, 1e-2);
  ocs2::RelaxedBarrierPenalty::Config swingPenaltyConfig(2.0, 0.1);

  ocs2::loadData::loadPtreeValue(pt, enableCbf, prefix + "enableSwingCBF", verbose);
  ocs2::loadData::loadPtreeValue(pt, cbfLambda, prefix + "swingcbfLambda", verbose);
  ocs2::loadData::loadPtreeValue(pt, cbfPenaltyConfig.mu, prefix + "swingcbfPenaltyMuParam", verbose);
  ocs2::loadData::loadPtreeValue(pt, cbfPenaltyConfig.delta, prefix + "swingcbfPenaltyDeltaParam", verbose);
  ocs2::loadData::loadPtreeValue(pt, swingPenaltyConfig.mu, prefix + "swingPenaltyMuParam", verbose);
  ocs2::loadData::loadPtreeValue(pt, swingPenaltyConfig.delta, prefix + "swingPenaltyDeltaParam", verbose);

  return {enableCbf, cbfLambda, cbfPenaltyConfig, swingPenaltyConfig};
}

std::tuple<bool, bool, ocs2::RelaxedBarrierPenalty::Config, ocs2::RelaxedBarrierPenalty::Config>
PerceptiveLeggedInterface::loadCollisionSettings(const std::string& taskFile, bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "CollisionParam.";

  bool enableFootCollision = true;
  bool enableSphereSdfConstraint = false;
  ocs2::RelaxedBarrierPenalty::Config footCollisionPenaltyConfig(1e-2, 0.005);
  ocs2::RelaxedBarrierPenalty::Config sphereSdfPenaltyConfig(1e-2, 0.005);

  ocs2::loadData::loadPtreeValue(pt, enableFootCollision, prefix + "enablefootCollision", verbose);
  ocs2::loadData::loadPtreeValue(pt, enableSphereSdfConstraint, prefix + "enableSphereSdfConstraint", verbose);
  ocs2::loadData::loadPtreeValue(pt, footCollisionPenaltyConfig.mu, prefix + "footCollisionPenaltyMu", verbose);
  ocs2::loadData::loadPtreeValue(pt, footCollisionPenaltyConfig.delta, prefix + "footCollisionPenaltyDelta", verbose);
  ocs2::loadData::loadPtreeValue(pt, sphereSdfPenaltyConfig.mu, prefix + "SphereSdfConstraintPenaltyMu", verbose);
  ocs2::loadData::loadPtreeValue(pt, sphereSdfPenaltyConfig.delta, prefix + "SphereSdfConstraintPenaltyDelta", verbose);

  return {enableFootCollision, enableSphereSdfConstraint, footCollisionPenaltyConfig, sphereSdfPenaltyConfig};
}

}  // namespace perceptive_legged_control
