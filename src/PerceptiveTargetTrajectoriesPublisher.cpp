#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <grid_map_core/GridMap.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace ocs2;

namespace {

class PerceptiveTargetTrajectoriesPublisher final : public rclcpp::Node {
 public:
  PerceptiveTargetTrajectoriesPublisher()
      : Node("perceptive_target_trajectories_publisher"),
        tfBuffer_(get_clock()) {}

  void init() {
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_, shared_from_this(), false);
    robotName_ = declare_parameter<std::string>("robot_name", "legged_robot");
    const auto referenceFile = declare_parameter<std::string>("referenceFile", "");
    const auto taskFile = declare_parameter<std::string>("taskFile", "");
    terrainTopic_ = declare_parameter<std::string>("terrain_topic", "/convex_plane_decomposition_ros/planar_terrain");
    terrainHeightLayer_ = declare_parameter<std::string>("terrain_height_layer", "smooth_planar");
    fallbackHeightLayer_ = declare_parameter<std::string>("fallback_height_layer", "elevation");
    terrainHeightOffset_ = declare_parameter<double>("terrain_height_offset", 0.0);
    commandHorizonScale_ = declare_parameter<double>("command_horizon_scale", 2.5);
    const double targetRepublishRate = declare_parameter<double>("target_republish_rate", 20.0);

    loadData::loadCppDataType(referenceFile, "comHeight", comHeight_);
    loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState_);
    loadData::loadCppDataType(referenceFile, "targetRotationVelocity", targetRotationVelocity_);
    loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", targetDisplacementVelocity_);
    loadData::loadCppDataType(taskFile, "mpc.timeHorizon", timeToTarget_);

    targetPublisher_ = std::make_unique<TargetTrajectoriesRosPublisher>(shared_from_this(), robotName_);
    observationSub_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName_ + "_mpc_observation", 1,
        [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(observationMutex_);
          latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        });
    terrainSub_ = create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
        terrainTopic_, 1,
        [this](const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstSharedPtr msg) {
          auto terrain = std::make_unique<convex_plane_decomposition::PlanarTerrain>(convex_plane_decomposition::fromMessage(*msg));
          std::lock_guard<std::mutex> lock(terrainMutex_);
          planarTerrain_.swap(terrain);
        });
    goalSub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 1,
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { handleGoal(*msg); });
    cmdVelSub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) { handleCmdVel(*msg); });

    if (targetRepublishRate > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / targetRepublishRate);
      targetRepublishTimer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() {
        if (!hasActiveCmdVel_) {
          return;
        }
        publishCmdVelTarget(lastCmdVel_);
      });
    }

    RCLCPP_INFO(get_logger(), "Perceptive target publisher ready: terrain=%s target=%s_mpc_target horizon_scale=%.2f republish=%.1fHz",
                terrainTopic_.c_str(), robotName_.c_str(), commandHorizonScale_, targetRepublishRate);
  }

 private:
  SystemObservation latestObservation() const {
    std::lock_guard<std::mutex> lock(observationMutex_);
    return latestObservation_;
  }

  bool hasObservation(const SystemObservation& observation) const {
    return observation.time != 0.0 && observation.state.size() >= 24;
  }

  double estimateTimeToTarget(const vector_t& desiredBaseDisplacement) const {
    const double rotationTime = std::abs(desiredBaseDisplacement(3)) / targetRotationVelocity_;
    const double displacement = std::hypot(desiredBaseDisplacement(0), desiredBaseDisplacement(1));
    const double displacementTime = displacement / targetDisplacementVelocity_;
    return std::max(rotationTime, displacementTime);
  }

  double terrainHeight(double x, double y, double fallback) const {
    std::lock_guard<std::mutex> lock(terrainMutex_);
    if (!planarTerrain_) {
      return fallback;
    }

    const auto& gridMap = planarTerrain_->gridMap;
    const grid_map::Position position{x, y};
    const auto layer = gridMap.exists(terrainHeightLayer_) ? terrainHeightLayer_ : fallbackHeightLayer_;
    if (!gridMap.exists(layer) || !gridMap.isInside(position)) {
      return fallback;
    }

    const auto height = gridMap.atPosition(layer, position, grid_map::InterpolationMethods::INTER_NEAREST);
    return std::isfinite(height) ? static_cast<double>(height) + terrainHeightOffset_ : fallback;
  }

  TargetTrajectories poseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                              double targetReachingTime, const vector_t& baseVelocity = vector_t()) const {
    const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

    vector_t currentPose = observation.state.segment<6>(6);
    currentPose(2) -= terrainHeight(currentPose(0), currentPose(1), 0.0);
    currentPose(4) = 0.0;
    currentPose(5) = 0.0;

    vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
    stateTrajectory[0] << vector_t::Zero(6), currentPose, defaultJointState_;
    stateTrajectory[1] << vector_t::Zero(6), targetPose, defaultJointState_;
    if (baseVelocity.size() >= 3) {
      stateTrajectory[0].head(3) = baseVelocity.head(3);
      stateTrajectory[1].head(3) = baseVelocity.head(3);
    }

    const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));
    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  void handleGoal(const geometry_msgs::msg::PoseStamped& msg) {
    const auto observation = latestObservation();
    if (!hasObservation(observation)) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose = msg;
    try {
      tfBuffer_.transform(pose, pose, "odom", tf2::durationFromSec(0.2));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "Goal transform failed: %s", ex.what());
      return;
    }

    Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    vector_t targetPose(6);
    targetPose << pose.pose.position.x, pose.pose.position.y, 0.0, q.toRotationMatrix().eulerAngles(0, 1, 2).z(), 0.0, 0.0;
    targetPose(2) = pose.pose.position.z + comHeight_;

    const vector_t currentPose = observation.state.segment<6>(6);
    const double targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
    targetPublisher_->publishTargetTrajectories(poseToTargetTrajectories(targetPose, observation, targetReachingTime));
  }

  static bool isZeroCmdVel(const geometry_msgs::msg::Twist& msg) {
    constexpr double kEps = 1e-6;
    return std::abs(msg.linear.x) < kEps && std::abs(msg.linear.y) < kEps && std::abs(msg.linear.z) < kEps &&
           std::abs(msg.angular.z) < kEps;
  }

  void handleCmdVel(const geometry_msgs::msg::Twist& msg) {
    if (isZeroCmdVel(msg)) {
      hasActiveCmdVel_ = false;
      return;
    }
    lastCmdVel_ = msg;
    hasActiveCmdVel_ = true;
    publishCmdVelTarget(msg);
  }

  void publishCmdVelTarget(const geometry_msgs::msg::Twist& msg) {
    const auto observation = latestObservation();
    if (!hasObservation(observation)) {
      return;
    }

    const vector_t currentPose = observation.state.segment<6>(6);
    const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
    vector_t cmdVel = vector_t::Zero(4);
    cmdVel << msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z;
    vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

    const double horizon = timeToTarget_ * commandHorizonScale_;
    vector_t targetPose(6);
    targetPose << currentPose(0) + cmdVelRot(0) * horizon, currentPose(1) + cmdVelRot(1) * horizon, 0.0,
        currentPose(3) + cmdVel(3) * horizon, 0.0, 0.0;
    const scalar_t currentRelativeHeight = currentPose(2) - terrainHeight(currentPose(0), currentPose(1), 0.0);
    targetPose(2) = std::max(currentRelativeHeight, comHeight_);

    targetPublisher_->publishTargetTrajectories(
        poseToTargetTrajectories(targetPose, observation, observation.time + horizon, cmdVelRot));
  }

  std::string robotName_;
  std::string terrainTopic_;
  std::string terrainHeightLayer_;
  std::string fallbackHeightLayer_;
  scalar_t targetDisplacementVelocity_{0.5};
  scalar_t targetRotationVelocity_{1.57};
  scalar_t comHeight_{0.45};
  scalar_t timeToTarget_{1.0};
  double terrainHeightOffset_{0.0};
  double commandHorizonScale_{2.5};
  vector_t defaultJointState_{vector_t::Zero(12)};
  geometry_msgs::msg::Twist lastCmdVel_;
  bool hasActiveCmdVel_{false};

  mutable std::mutex observationMutex_;
  mutable std::mutex terrainMutex_;
  SystemObservation latestObservation_;
  std::unique_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrain_;
  std::unique_ptr<TargetTrajectoriesRosPublisher> targetPublisher_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;
  rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr terrainSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  rclcpp::TimerBase::SharedPtr targetRepublishTimer_;
  tf2_ros::Buffer tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerceptiveTargetTrajectoriesPublisher>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
