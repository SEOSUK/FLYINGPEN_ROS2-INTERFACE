// Velocity/noise diagnostic logger. Firmware behavior is not modified here.
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using LogMsg = crazyflie_interfaces::msg::LogDataGeneric;
using PosesMsg = motion_capture_tracking_interfaces::msg::NamedPoseArray;
using Clock = std::chrono::steady_clock;
using std::placeholders::_1;

namespace {
double nanv() { return std::numeric_limits<double>::quiet_NaN(); }
std::string expand_user(const std::string & p) {
  if (!p.empty() && p[0] == '~') if (const char * h = std::getenv("HOME")) return std::string(h) + p.substr(1);
  return p;
}
std::string timestamp() {
  std::time_t raw = std::time(nullptr); std::tm tm{}; localtime_r(&raw, &tm); char s[32];
  std::strftime(s, sizeof(s), "%Y%m%d_%H%M%S", &tm); return s;
}
std::string sanitize(const std::string & v) {
  std::string r; for (char c : v) r.push_back(std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_' ? c : '_'); return r;
}
}  // namespace

class VelocityDebugLogger final : public rclcpp::Node {
public:
  VelocityDebugLogger() : Node("data_logging_debug"), start_(Clock::now()), last_row_(start_) {
    csv_dir_ = expand_user(declare_parameter<std::string>("csv_dir", "~/hitl_ws/src/flying_pen/bag/logging"));
    cf_ns_ = declare_parameter<std::string>("cf_ns", "/cf2");
    loop_hz_ = declare_parameter<double>("loop_hz", 50.0);
    robot_name_ = cf_ns_; while (!robot_name_.empty() && robot_name_.front() == '/') robot_name_.erase(robot_name_.begin());
    std::filesystem::create_directories(csv_dir_);
    csv_path_ = (std::filesystem::path(csv_dir_) / (timestamp() + "_velocity_debug.csv")).string();
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_) throw std::runtime_error("Cannot open " + csv_path_);
    write_header();
    poses_sub_ = create_subscription<PosesMsg>("/poses", rclcpp::SensorDataQoS(), std::bind(&VelocityDebugLogger::poses, this, _1));
    received_["mocap_receive_debug"] = false;
    mocap_debug_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/mocap_receive_debug", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&VelocityDebugLogger::mocap_debug, this, _1));
    bind("cf_kalman_timing", 6, &VelocityDebugLogger::kalman_);
    bind("cf_loop_timing", 3, &VelocityDebugLogger::loop_);
    bind("cf_extpos_timing", 5, &VelocityDebugLogger::ext_timing_);
    bind("cf_extpos_xyz", 3, &VelocityDebugLogger::ext_xyz_);
    bind("cf_state_posvel", 6, &VelocityDebugLogger::state_);
    bind("cf_pos_velocity", 6, &VelocityDebugLogger::pos_velocity_);
    bind("cf_pos_velocity_events", 5, &VelocityDebugLogger::pos_events_);
    bind("cf_contact_velocity", 6, &VelocityDebugLogger::contact_velocity_);
    bind("cf_contact_offset_velocity", 3, &VelocityDebugLogger::contact_offset_velocity_);
    bind("cf_velocity_command", 3, &VelocityDebugLogger::command_);
    bind("cf_gyro_body", 3, &VelocityDebugLogger::gyro_);
    tag_sub_ = create_subscription<std_msgs::msg::String>("/flying_pen/debug_log_filename_tag", 10, std::bind(&VelocityDebugLogger::tag, this, _1));
    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/data_logging_msg_debug", 10);
    startup_timer_ = create_wall_timer(std::chrono::seconds(5), std::bind(&VelocityDebugLogger::check_topics, this));
    RCLCPP_INFO(get_logger(), "Velocity debug CSV: %s", csv_path_.c_str());
  }
  ~VelocityDebugLogger() override { csv_.flush(); }
  double loop_hz() const { return loop_hz_; }
  void write_row() {
    const auto now = Clock::now();
    const double t = std::chrono::duration<double>(now - start_).count();
    const double dt_ms = std::chrono::duration<double, std::milli>(now - last_row_).count(); last_row_ = now;
    std::vector<double> row; row.reserve(58); row.push_back(t); row.push_back(dt_ms);
    append(row, mocap_xyz_); row.push_back(mocap_dt_ms_); row.push_back(mocap_count_);
    row.push_back(mocap_source_time_); row.push_back(mocap_source_age_ms_);
    append(row, ext_xyz_); append(row, ext_timing_); append(row, kalman_); append(row, loop_);
    append(row, state_); append(row, pos_velocity_); append(row, pos_events_);
    append(row, contact_velocity_); append(row, contact_offset_velocity_);
    append(row, command_); append(row, gyro_);
    csv_ << std::fixed << std::setprecision(9);
    for (size_t i = 0; i < row.size(); ++i) {
      csv_ << (i ? "," : "") << row[i];
    }
    csv_ << '\n';
    if (t - last_flush_sec_ >= 1.0) { csv_.flush(); last_flush_sec_ = t; }
    std_msgs::msg::Float64MultiArray msg; msg.data = row; publisher_->publish(msg);
  }
private:
  template<size_t N> static void append(std::vector<double> & row, const std::array<double, N> & a) { row.insert(row.end(), a.begin(), a.end()); }
  template<size_t N> void bind(const std::string & topic, size_t expected, std::array<double, N> VelocityDebugLogger::* field) {
    received_[topic] = false;
    subscriptions_.push_back(create_subscription<LogMsg>(cf_ns_ + "/" + topic, 10,
      [this, topic, expected, field](const LogMsg::SharedPtr msg) {
        if (msg->values.size() != expected) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s expected %zu fields, received %zu", topic.c_str(), expected, msg->values.size()); return;
        }
        auto & dst = this->*field; for (size_t i = 0; i < N; ++i) dst[i] = msg->values[i]; received_[topic] = true;
      }));
  }
  void poses(const PosesMsg::SharedPtr msg) {
    for (const auto & p : msg->poses) if (p.name == robot_name_) {
      mocap_xyz_ = {p.pose.position.x, p.pose.position.y, p.pose.position.z}; mocap_received_ = true; break;
    }
  }
  void mocap_debug(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != 4) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "mocap_receive_debug expected 4 fields, received %zu", msg->data.size());
      return;
    }
    mocap_dt_ms_ = msg->data[0];
    mocap_count_ = msg->data[1];
    mocap_source_time_ = msg->data[2];
    mocap_source_age_ms_ = msg->data[3];
    received_["mocap_receive_debug"] = true;
  }
  void check_topics() {
    if (!mocap_received_) RCLCPP_ERROR(get_logger(), "No /poses entry for '%s'; mocap columns remain NaN", robot_name_.c_str());
    for (const auto & x : received_) if (!x.second) RCLCPP_ERROR(get_logger(), "No data on %s/%s; verify YAML and firmware TOC", cf_ns_.c_str(), x.first.c_str());
    startup_timer_->cancel();
  }
  void tag(const std_msgs::msg::String::SharedPtr msg) {
    const std::string clean = sanitize(msg->data); if (clean.empty()) return;
    csv_.flush(); csv_.close(); const auto old = std::filesystem::path(csv_path_);
    const auto next = old.parent_path() / (timestamp() + "_velocity_debug_" + clean + ".csv");
    std::error_code ec; std::filesystem::rename(old, next, ec);
    if (ec) { RCLCPP_ERROR(get_logger(), "CSV rename failed: %s", ec.message().c_str()); csv_.open(old, std::ios::app); }
    else { csv_path_ = next.string(); csv_.open(next, std::ios::app); RCLCPP_INFO(get_logger(), "Velocity debug CSV renamed: %s", csv_path_.c_str()); }
  }
  void write_header() {
    csv_ << "t_sec,loggerDtMs,mocapRawX,mocapRawY,mocapRawZ,mocapRxDtMs,mocapRxCount,mocapSourceTime,mocapSourceAgeMs,"
      "extPosX,extPosY,extPosZ,extPosRxDtMs,extPosRxCount,extPosGapCount,extPosGapLastMs,extPosRxDtMaxMs,"
      "kalmanRtPred,kalmanRtUpdate,kalmanRtFinal,kalmanResetCount,kalmanSupervisorResetCount,stateUpdateCount,loopDtUs,loopDtMaxUs,loopOverrunCount,"
      "stateX,stateY,stateZ,stateVx,stateVy,stateVz,posRawVx,posRawVy,posRawVz,posVx,posVy,posVz,"
      "posDeltaX,posDeltaY,posDeltaZ,velocity_rejection_count,velocity_buffer_reset_count,"
      "vcRawX,vcRawY,vcRawZ,vcX,vcY,vcZ,"
      "rotOffsetVelX,rotOffsetVelY,rotOffsetVelZ,"
      "velDesX,velDesY,velDesZ,gyroX,gyroY,gyroZ\n"; csv_.flush();
  }
  std::string csv_dir_, csv_path_, cf_ns_, robot_name_; std::ofstream csv_;
  double loop_hz_{50.0}, last_flush_sec_{0.0}; Clock::time_point start_, last_row_;
  double mocap_count_{0.0}; bool mocap_received_{false};
  double mocap_dt_ms_{nanv()}, mocap_source_time_{nanv()}, mocap_source_age_ms_{nanv()};
  std::array<double,3> mocap_xyz_{nanv(),nanv(),nanv()}, ext_xyz_{nanv(),nanv(),nanv()}, loop_{nanv(),nanv(),nanv()}, contact_offset_velocity_{nanv(),nanv(),nanv()}, command_{nanv(),nanv(),nanv()}, gyro_{nanv(),nanv(),nanv()};
  std::array<double,5> ext_timing_{nanv(),nanv(),nanv(),nanv(),nanv()}, pos_events_{nanv(),nanv(),nanv(),nanv(),nanv()};
  std::array<double,6> kalman_{nanv(),nanv(),nanv(),nanv(),nanv(),nanv()};
  std::array<double,6> state_{nanv(),nanv(),nanv(),nanv(),nanv(),nanv()}, pos_velocity_{nanv(),nanv(),nanv(),nanv(),nanv(),nanv()}, contact_velocity_{nanv(),nanv(),nanv(),nanv(),nanv(),nanv()};
  std::map<std::string,bool> received_; std::vector<rclcpp::Subscription<LogMsg>::SharedPtr> subscriptions_;
  rclcpp::Subscription<PosesMsg>::SharedPtr poses_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mocap_debug_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tag_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); auto node = std::make_shared<VelocityDebugLogger>(); rclcpp::Rate rate(node->loop_hz());
  while (rclcpp::ok()) { rclcpp::spin_some(node); node->write_row(); rate.sleep(); } rclcpp::shutdown(); return 0;
}
