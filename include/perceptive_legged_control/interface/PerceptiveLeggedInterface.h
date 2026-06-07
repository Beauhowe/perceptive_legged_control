#pragma once

#include <memory>
#include <tuple>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <legged_interface/LeggedInterface.h>
#include <ocs2_core/penalties/Penalties.h>
#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>
#endif
#include <rclcpp/rclcpp.hpp>

#include "perceptive_legged_control/interface/PlanarSignedDistanceField.h"

namespace perceptive_legged_control {

class PerceptiveLeggedInterface : public legged::LeggedInterface {
 public:
  PerceptiveLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            rclcpp::Node::SharedPtr node, bool useHardFrictionConeConstraint = false);

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const { return planarTerrainPtr_; }
  std::shared_ptr<PlanarSignedDistanceField> getSignedDistanceFieldPtr() const { return signedDistanceFieldPtr_; }
#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
  std::shared_ptr<ocs2::PinocchioSphereInterface> getPinocchioSphereInterfacePtr() const { return pinocchioSphereInterfacePtr_; }
#endif

  void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                           bool verbose) override;
  size_t getNumVertices() const { return numVertices_; }

 protected:
  void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                             bool verbose) override;

 private:
  void initializeFlatPlanarTerrain();
  std::tuple<bool, bool, ocs2::scalar_t, ocs2::RelaxedBarrierPenalty::Config, ocs2::RelaxedBarrierPenalty::Config>
  loadFootPlacementSettings(const std::string& taskFile, bool verbose) const;
  std::tuple<bool, ocs2::scalar_t, ocs2::RelaxedBarrierPenalty::Config, ocs2::RelaxedBarrierPenalty::Config>
  loadSwingFootPlacementSettings(const std::string& taskFile, bool verbose) const;
  std::tuple<bool, bool, ocs2::RelaxedBarrierPenalty::Config, ocs2::RelaxedBarrierPenalty::Config>
  loadCollisionSettings(const std::string& taskFile, bool verbose) const;

  rclcpp::Node::SharedPtr node_;
  size_t numVertices_{16};
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<PlanarSignedDistanceField> signedDistanceFieldPtr_;
#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
  std::shared_ptr<ocs2::PinocchioSphereInterface> pinocchioSphereInterfacePtr_;
#endif
};

}  // namespace perceptive_legged_control
