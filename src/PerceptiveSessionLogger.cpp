#include <chrono>
#include <cmath>
#include <ctime>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include <array>

#include <geometry_msgs/msg/twist.hpp>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace fs = boost::filesystem;

namespace {

constexpr size_t kNumJoints = 12;
constexpr size_t kJointStateOffset = 12;
constexpr size_t kNumFeet = 4;
constexpr double kTargetCompareEpsilon = 1e-4;
constexpr std::array<const char*, kNumJoints> kJointNames = {
    "LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE",
};
constexpr std::array<const char*, kNumFeet> kFootNames = {"LF", "LH", "RF", "RH"};
constexpr std::array<const char*, kNumFeet> kDefaultFootFrames = {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};

using SteadyClock = std::chrono::steady_clock;
using SteadyTimePoint = SteadyClock::time_point;

std::string formatTimestamp(const std::chrono::system_clock::time_point& tp) {
  const auto timeT = std::chrono::system_clock::to_time_t(tp);
  std::tm localTime{};
  localtime_r(&timeT, &localTime);
  std::ostringstream oss;
  oss << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
  return oss.str();
}

std::string formatSessionFolderName(const std::chrono::system_clock::time_point& tp) {
  const auto timeT = std::chrono::system_clock::to_time_t(tp);
  std::tm localTime{};
  localtime_r(&timeT, &localTime);
  std::ostringstream oss;
  oss << "session_" << std::put_time(&localTime, "%Y%m%d_%H%M%S");
  return oss.str();
}

bool statesEqual(const std::vector<float>& a, const std::vector<float>& b) {
  const size_t compareLen = std::min({a.size(), b.size(), kJointStateOffset + kNumJoints});
  if (compareLen < kJointStateOffset + kNumJoints) {
    return false;
  }
  for (size_t i = kJointStateOffset; i < compareLen; ++i) {
    if (std::abs(static_cast<double>(a[i]) - static_cast<double>(b[i])) > kTargetCompareEpsilon) {
      return false;
    }
  }
  return true;
}

bool targetsEqual(const ocs2_msgs::msg::MpcTargetTrajectories& a, const ocs2_msgs::msg::MpcTargetTrajectories& b) {
  if (a.time_trajectory.size() != b.time_trajectory.size() || a.state_trajectory.size() != b.state_trajectory.size()) {
    return false;
  }
  for (size_t i = 0; i < a.time_trajectory.size(); ++i) {
    if (std::abs(a.time_trajectory[i] - b.time_trajectory[i]) > kTargetCompareEpsilon) {
      return false;
    }
  }
  for (size_t i = 0; i < a.state_trajectory.size(); ++i) {
    if (!statesEqual(a.state_trajectory[i].value, b.state_trajectory[i].value)) {
      return false;
    }
  }
  return true;
}

class PerceptiveSessionLogger final : public rclcpp::Node {
 public:
  PerceptiveSessionLogger() : Node("perceptive_session_logger") {}

  void init() {
    const auto sessionStart = std::chrono::system_clock::now();
    const std::string logDir = declare_parameter<std::string>("log_dir", "/workspace/logs");
    robotName_ = declare_parameter<std::string>("robot_name", "legged_robot");
    const bool logRosout = declare_parameter<bool>("log_rosout", true);
    const bool logCmdVel = declare_parameter<bool>("log_cmd_vel", true);
    const bool logModeSchedule = declare_parameter<bool>("log_mode_schedule", true);
    const bool logMpcTarget = declare_parameter<bool>("log_mpc_target", true);
    const double observationRateHz = declare_parameter<double>("observation_rate_hz", 50.0);
    mpcTargetLogRateHz_ = declare_parameter<double>("mpc_target_log_rate_hz", 5.0);
    cmdVelLogRateHz_ = declare_parameter<double>("cmd_vel_log_rate_hz", 20.0);
    rosoutLogRateHz_ = declare_parameter<double>("rosout_log_rate_hz", 0.0);
    worldFrame_ = declare_parameter<std::string>("world_frame", "odom");
    footFrames_ = declare_parameter<std::vector<std::string>>("foot_frames",
                                                              std::vector<std::string>(kDefaultFootFrames.begin(),
                                                                                       kDefaultFootFrames.end()));

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    sessionDir_ = fs::path(logDir) / formatSessionFolderName(sessionStart);
    fs::create_directories(sessionDir_);

    eventsStream_.open(sessionDir_ / "events.log", std::ios::out | std::ios::trunc);
    observationStream_.open(sessionDir_ / "observation.csv", std::ios::out | std::ios::trunc);
    writeObservationHeader(observationStream_);

    writeEvent("session_start", "log_dir=" + sessionDir_.string());

    const auto observationPeriod = observationRateHz > 0.0
                                       ? std::chrono::duration<double>(1.0 / observationRateHz)
                                       : std::chrono::duration<double>(0.0);
    if (observationRateHz > 0.0) {
      observationTimer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(observationPeriod),
                                            [this]() { flushObservationSample(); });
    }

    observationSub_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName_ + "_mpc_observation", rclcpp::SensorDataQoS(),
        [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latestObservation_ = msg;
        });

    if (logMpcTarget) {
      targetSub_ = create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
          robotName_ + "_mpc_target", 10,
          [this](const ocs2_msgs::msg::MpcTargetTrajectories::ConstSharedPtr msg) { logMpcTargetMessage(msg); });
      targetEventTimer_ = create_wall_timer(std::chrono::seconds(1), [this]() { logTargetHealth(); });
    }

    if (logCmdVel) {
      cmdVelSub_ = create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            latestCmdVel_ = *msg;
            hasLatestCmdVel_ = true;
            if (cmdVelLogRateHz_ <= 0.0) {
              writeCmdVelSampleUnlocked(formatTimestamp(std::chrono::system_clock::now()), latestCmdVel_);
            }
          });
      if (cmdVelLogRateHz_ > 0.0) {
        const auto period = std::chrono::duration<double>(1.0 / cmdVelLogRateHz_);
        cmdVelTimer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                         [this]() { flushCmdVelSample(); });
      }
    }

    if (logModeSchedule) {
      modeScheduleSub_ = create_subscription<ocs2_msgs::msg::ModeSchedule>(
          robotName_ + "_mpc_mode_schedule", 10,
          [this](const ocs2_msgs::msg::ModeSchedule::ConstSharedPtr msg) {
            if (!modeScheduleStream_.is_open()) {
              modeScheduleStream_.open(sessionDir_ / "mode_schedule.log", std::ios::out | std::ios::trunc);
            }
            const auto now = formatTimestamp(std::chrono::system_clock::now());
            std::lock_guard<std::mutex> lock(mutex_);
            modeScheduleStream_ << now << " event_times=[";
            for (size_t i = 0; i < msg->event_times.size(); ++i) {
              if (i > 0) {
                modeScheduleStream_ << ',';
              }
              modeScheduleStream_ << msg->event_times[i];
            }
            modeScheduleStream_ << "] modes=[";
            for (size_t i = 0; i < msg->mode_sequence.size(); ++i) {
              if (i > 0) {
                modeScheduleStream_ << ',';
              }
              modeScheduleStream_ << static_cast<int>(msg->mode_sequence[i]);
            }
            modeScheduleStream_ << "]\n";
          });
    }

    if (logRosout) {
      rosoutSub_ = create_subscription<rcl_interfaces::msg::Log>(
          "/rosout", rclcpp::QoS(100), [this](const rcl_interfaces::msg::Log::ConstSharedPtr msg) { logRosoutMessage(msg); });
    }

    RCLCPP_INFO(get_logger(), "Perceptive session logger writing to %s", sessionDir_.c_str());
    writeEvent("logger_ready", "robot_name=" + robotName_);
  }

 private:
  void writeEvent(const std::string& event, const std::string& detail) {
    if (!eventsStream_.is_open()) {
      return;
    }
    const auto now = formatTimestamp(std::chrono::system_clock::now());
    eventsStream_ << now << " [" << event << "] " << detail << '\n';
    eventsStream_.flush();
  }

  void flushObservationSample() {
    ocs2_msgs::msg::MpcObservation::ConstSharedPtr observation;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      observation = latestObservation_;
    }
    if (!observation || observation->state.value.size() < 12) {
      return;
    }

    const auto& state = observation->state.value;
    const auto now = formatTimestamp(std::chrono::system_clock::now());
    const std::string footFields = formatFootPositions();
    std::lock_guard<std::mutex> lock(mutex_);
    observationStream_ << now << ',' << observation->time << ',' << static_cast<int>(observation->mode) << ','
                         << state[6] << ',' << state[7] << ',' << state[8] << ',' << state[9] << ',' << state[10] << ','
                         << state[11] << ',' << state[0] << ',' << state[1] << ',' << state[2];
    appendJointAngles(observationStream_, state);
    observationStream_ << footFields << '\n';
  }

  void flushCmdVelSample() {
    geometry_msgs::msg::Twist cmdVel;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!hasLatestCmdVel_) {
        return;
      }
      cmdVel = latestCmdVel_;
    }
    const auto now = formatTimestamp(std::chrono::system_clock::now());
    std::lock_guard<std::mutex> lock(mutex_);
    writeCmdVelSampleUnlocked(now, cmdVel);
  }

  void writeCmdVelSampleUnlocked(const std::string& wallTime, const geometry_msgs::msg::Twist& cmdVel) {
    if (!cmdVelStream_.is_open()) {
      cmdVelStream_.open(sessionDir_ / "cmd_vel.log", std::ios::out | std::ios::trunc);
      cmdVelStream_ << "wall_time,linear_x,linear_y,linear_z,angular_z\n";
    }
    cmdVelStream_ << wallTime << ',' << cmdVel.linear.x << ',' << cmdVel.linear.y << ',' << cmdVel.linear.z << ','
                  << cmdVel.angular.z << '\n';
  }

  void logRosoutMessage(const rcl_interfaces::msg::Log::ConstSharedPtr& msg) {
    if (!msg) {
      return;
    }

    const bool isHighPriority = msg->level >= rcl_interfaces::msg::Log::WARN;
    if (!isHighPriority) {
      if (rosoutLogRateHz_ <= 0.0) {
        return;
      }
      const auto now = SteadyClock::now();
      std::lock_guard<std::mutex> lock(mutex_);
      if (lastRosoutInfoLogTime_ != SteadyTimePoint{} &&
          std::chrono::duration<double>(now - lastRosoutInfoLogTime_).count() < 1.0 / rosoutLogRateHz_) {
        return;
      }
      lastRosoutInfoLogTime_ = now;
    }

    if (!rosoutStream_.is_open()) {
      rosoutStream_.open(sessionDir_ / "rosout.log", std::ios::out | std::ios::trunc);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    rosoutStream_ << msg->stamp.sec << '.' << std::setw(9) << std::setfill('0') << msg->stamp.nanosec << ' '
                  << static_cast<int>(msg->level) << ' ' << msg->name << ": " << msg->msg << '\n';
  }

  static void writeObservationHeader(std::ostream& stream) {
    stream << "wall_time,mpc_time,mode,base_x,base_y,base_z,yaw,pitch,roll,vcom_x,vcom_y,vcom_z";
    for (const char* name : kJointNames) {
      stream << ',' << name;
    }
    for (const char* name : kFootNames) {
      stream << ',' << name << "_foot_x," << name << "_foot_y," << name << "_foot_z";
    }
    stream << '\n';
  }

  static void appendEmptyCsvFields(std::ostream& stream, size_t count) {
    for (size_t i = 0; i < count; ++i) {
      stream << ',';
    }
  }

  template <typename Scalar>
  static void appendJointAngles(std::ostream& stream, const std::vector<Scalar>& state) {
    if (state.size() < kJointStateOffset + kNumJoints) {
      appendEmptyCsvFields(stream, kNumJoints);
      return;
    }
    for (size_t i = 0; i < kNumJoints; ++i) {
      stream << ',' << static_cast<double>(state[kJointStateOffset + i]);
    }
  }

  std::string formatFootPositions() const {
    std::ostringstream stream;
    for (const auto& footFrame : footFrames_) {
      try {
        const auto transform = tfBuffer_->lookupTransform(worldFrame_, footFrame, tf2::TimePointZero);
        stream << ',' << transform.transform.translation.x << ',' << transform.transform.translation.y << ','
               << transform.transform.translation.z;
      } catch (const tf2::TransformException&) {
        stream << ",,,";
      }
    }
    return stream.str();
  }

  static void appendBasePose(std::ostream& stream, const ocs2_msgs::msg::MpcState& state) {
    const auto& values = state.value;
    if (values.size() < 12) {
      stream << ",,,,,,";
      return;
    }
    for (size_t i = 6; i < 12; ++i) {
      if (i > 6) {
        stream << ',';
      }
      stream << values[i];
    }
  }

  bool shouldLogTargetUnlocked(const ocs2_msgs::msg::MpcTargetTrajectories& msg) {
    const auto now = SteadyClock::now();
    if (!hasLastLoggedTarget_) {
      lastTargetLogTime_ = now;
      return true;
    }
    if (targetsEqual(msg, lastLoggedTarget_)) {
      return false;
    }
    if (mpcTargetLogRateHz_ <= 0.0) {
      lastTargetLogTime_ = now;
      return true;
    }
    if (lastTargetLogTime_ == SteadyTimePoint{} ||
        std::chrono::duration<double>(now - lastTargetLogTime_).count() >= 1.0 / mpcTargetLogRateHz_) {
      lastTargetLogTime_ = now;
      return true;
    }
    return false;
  }

  void logMpcTargetMessage(const ocs2_msgs::msg::MpcTargetTrajectories::ConstSharedPtr& msg) {
    if (!msg) {
      return;
    }

    const double timeEnd = msg->time_trajectory.empty() ? 0.0 : msg->time_trajectory.back();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latestTargetFinalTime_ = timeEnd;
      if (timeEnd > lastExpiredTargetFinalTime_ + kTargetCompareEpsilon) {
        lastExpiredTargetFinalTime_ = -1.0;
      }
    }

    const auto now = formatTimestamp(std::chrono::system_clock::now());
    const double timeStart = msg->time_trajectory.empty() ? 0.0 : msg->time_trajectory.front();

    std::lock_guard<std::mutex> lock(mutex_);
    if (!shouldLogTargetUnlocked(*msg)) {
      return;
    }
    lastLoggedTarget_ = *msg;
    hasLastLoggedTarget_ = true;

    if (!targetStream_.is_open()) {
      targetStream_.open(sessionDir_ / "mpc_target.log", std::ios::out | std::ios::trunc);
      writeMpcTargetHeader(targetStream_);
    }

    targetStream_ << now << ',' << timeStart << ',' << timeEnd << ',';
    if (!msg->state_trajectory.empty()) {
      appendBasePose(targetStream_, msg->state_trajectory.front());
      appendJointAngles(targetStream_, msg->state_trajectory.front().value);
      appendBasePose(targetStream_, msg->state_trajectory.back());
      appendJointAngles(targetStream_, msg->state_trajectory.back().value);
    } else {
      appendEmptyCsvFields(targetStream_, 6 + kNumJoints + 6 + kNumJoints);
    }
    targetStream_ << '\n';
    targetStream_.flush();

    logMpcTargetTrajectoryNodes(now, *msg);
  }

  static void writeMpcTargetHeader(std::ostream& stream) {
    stream << "wall_time,time_start,time_end,"
           << "start_x,start_y,start_z,start_yaw,start_pitch,start_roll";
    for (const char* name : kJointNames) {
      stream << ",start_" << name;
    }
    stream << ",end_x,end_y,end_z,end_yaw,end_pitch,end_roll";
    for (const char* name : kJointNames) {
      stream << ",end_" << name;
    }
    stream << '\n';
  }

  void logMpcTargetTrajectoryNodes(const std::string& wallTime, const ocs2_msgs::msg::MpcTargetTrajectories& msg) {
    if (msg.state_trajectory.empty()) {
      return;
    }

    const double timeStart = msg.time_trajectory.empty() ? 0.0 : msg.time_trajectory.front();
    const double timeEnd = msg.time_trajectory.empty() ? 0.0 : msg.time_trajectory.back();

    if (!targetTrajectoryStream_.is_open()) {
      targetTrajectoryStream_.open(sessionDir_ / "mpc_target_trajectory.csv", std::ios::out | std::ios::trunc);
      targetTrajectoryStream_ << "wall_time,time_start,time_end,node_index,node_time,"
                              << "base_x,base_y,base_z,yaw,pitch,roll";
      for (const char* name : kJointNames) {
        targetTrajectoryStream_ << ',' << name;
      }
      targetTrajectoryStream_ << '\n';
    }

    for (size_t i = 0; i < msg.state_trajectory.size(); ++i) {
      const double nodeTime = i < msg.time_trajectory.size() ? msg.time_trajectory[i] : 0.0;
      targetTrajectoryStream_ << wallTime << ',' << timeStart << ',' << timeEnd << ',' << i << ',' << nodeTime << ',';
      appendBasePose(targetTrajectoryStream_, msg.state_trajectory[i]);
      appendJointAngles(targetTrajectoryStream_, msg.state_trajectory[i].value);
      targetTrajectoryStream_ << '\n';
    }
    targetTrajectoryStream_.flush();
  }

  void logTargetHealth() {
    ocs2_msgs::msg::MpcObservation::ConstSharedPtr observation;
    double targetFinalTime = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      observation = latestObservation_;
      targetFinalTime = latestTargetFinalTime_;
    }
    if (!observation) {
      return;
    }

    const double mpcTime = observation->time;
    if (targetFinalTime > 0.0 && mpcTime > targetFinalTime + 0.05) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (std::abs(targetFinalTime - lastExpiredTargetFinalTime_) <= kTargetCompareEpsilon) {
        return;
      }
      lastExpiredTargetFinalTime_ = targetFinalTime;
      std::ostringstream detail;
      detail << "mpc_time=" << mpcTime << " target_final_time=" << targetFinalTime;
      writeEvent("target_plan_expired", detail.str());
    }
  }

  std::string robotName_;
  std::string worldFrame_;
  std::vector<std::string> footFrames_;
  double mpcTargetLogRateHz_{5.0};
  double cmdVelLogRateHz_{20.0};
  double rosoutLogRateHz_{0.0};
  fs::path sessionDir_;
  std::mutex mutex_;
  ocs2_msgs::msg::MpcObservation::ConstSharedPtr latestObservation_;
  double latestTargetFinalTime_{0.0};
  double lastExpiredTargetFinalTime_{-1.0};
  bool hasLastLoggedTarget_{false};
  bool hasLatestCmdVel_{false};
  ocs2_msgs::msg::MpcTargetTrajectories lastLoggedTarget_;
  geometry_msgs::msg::Twist latestCmdVel_;
  SteadyTimePoint lastTargetLogTime_{};
  SteadyTimePoint lastRosoutInfoLogTime_{};

  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  std::ofstream eventsStream_;
  std::ofstream observationStream_;
  std::ofstream cmdVelStream_;
  std::ofstream modeScheduleStream_;
  std::ofstream rosoutStream_;
  std::ofstream targetStream_;
  std::ofstream targetTrajectoryStream_;

  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr targetSub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  rclcpp::Subscription<ocs2_msgs::msg::ModeSchedule>::SharedPtr modeScheduleSub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosoutSub_;
  rclcpp::TimerBase::SharedPtr observationTimer_;
  rclcpp::TimerBase::SharedPtr targetEventTimer_;
  rclcpp::TimerBase::SharedPtr cmdVelTimer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PerceptiveSessionLogger>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
