cmake_minimum_required(VERSION 3.10)
project(perceptive_legged_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(convex_plane_decomposition REQUIRED)
find_package(convex_plane_decomposition_msgs REQUIRED)
find_package(convex_plane_decomposition_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(legged_controllers REQUIRED)
find_package(legged_estimation REQUIRED)
find_package(legged_interface REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_sphere_approximation QUIET)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system log log_setup)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common)

set(PACKAGE_DEPENDENCIES
  controller_interface
  convex_plane_decomposition
  convex_plane_decomposition_msgs
  convex_plane_decomposition_ros
  geometry_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  grid_map_sdf
  legged_controllers
  legged_estimation
  legged_interface
  ocs2_core
  ocs2_centroidal_model
  ocs2_mpc
  ocs2_oc
  ocs2_msgs
  ocs2_legged_robot
  ocs2_robotic_tools
  ocs2_ros_interfaces
  pluginlib
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_library(perceptive_legged_controller SHARED
  src/interface/ConvexRegionSelector.cpp
  src/synchronized_module/PlanarTerrainReceiver.cpp
  src/controller/PerceptiveLeggedController.cpp
  src/controller/PerceptiveLeggedCheaterController.cpp
  src/interface/PerceptiveLeggedInterface.cpp
  src/interface/PerceptiveLeggedPrecomputation.cpp
  src/constraint/FootPlacementConstraint.cpp
  src/constraint/FootPlacementConstraintCBF.cpp
  src/constraint/FootCollisionConstraint.cpp
  src/constraint/SwingFootPlacementConstraintCBF.cpp
  src/visualization/FootPlacementVisualization.cpp
  src/interface/PlanarSignedDistanceField.cpp
  src/interface/PerceptiveSwitchedModelReferenceManager.cpp
  src/PerceptiveTerrainBuffer.cpp
)
target_include_directories(perceptive_legged_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(perceptive_legged_controller ${PACKAGE_DEPENDENCIES})
target_link_libraries(perceptive_legged_controller ${PCL_LIBRARIES})
target_compile_options(perceptive_legged_controller PUBLIC ${OCS2_CXX_FLAGS})
if(ocs2_sphere_approximation_FOUND)
  target_sources(perceptive_legged_controller PRIVATE
    src/constraint/SphereSdfConstraint.cpp
    src/visualization/SphereVisualization.cpp
  )
  target_compile_definitions(perceptive_legged_controller PUBLIC PERCEPTIVE_HAS_SPHERE_APPROXIMATION)
  ament_target_dependencies(perceptive_legged_controller ocs2_sphere_approximation)
endif()
pluginlib_export_plugin_description_file(controller_interface perceptive_legged_control_plugins.xml)

add_executable(perceptive_target_trajectories_publisher src/PerceptiveTargetTrajectoriesPublisher.cpp)
ament_target_dependencies(perceptive_target_trajectories_publisher ${PACKAGE_DEPENDENCIES})
target_link_libraries(perceptive_target_trajectories_publisher Eigen3::Eigen)

install(TARGETS
  perceptive_legged_controller
  perceptive_target_trajectories_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES perceptive_legged_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(perceptive_legged_controller)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
