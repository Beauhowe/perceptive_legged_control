#pragma once

#include <memory>

#include <legged_controllers/LeggedController.h>

#include "perceptive_legged_control/visualization/FootPlacementVisualization.h"
#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
#include "perceptive_legged_control/visualization/SphereVisualization.h"
#endif

namespace perceptive_legged_control {

class PerceptiveLeggedController : public legged::LeggedController {
 public:
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose) override;
  void setupMpc() override;

 private:
  void setupVisualization();

  std::shared_ptr<FootPlacementVisualization> footPlacementVisualizationPtr_;
#ifdef PERCEPTIVE_HAS_SPHERE_APPROXIMATION
  std::shared_ptr<SphereVisualization> sphereVisualizationPtr_;
#endif
};

}  // namespace perceptive_legged_control
