// Canonical FlyingPen flight-data logger.
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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

class DataLoggingNode final : public rclcpp::Node {
public:
  DataLoggingNode() : Node("data_logging"), start_(Clock::now()), last_row_(start_) {
    csv_dir_ = expand_user(declare_parameter<std::string>("csv_dir", "~/hitl_ws/src/flying_pen/bag/logging"));
    cf_ns_ = declare_parameter<std::string>("cf_ns", "/cf2");
    loop_hz_ = declare_parameter<double>("loop_hz", 50.0);
    startup_check_delay_sec_ = declare_parameter<double>("startup_check_delay_sec", 15.0);
    publish_topic_ = declare_parameter<std::string>("publish_topic", "/data_logging_msg");
    robot_name_ = cf_ns_; while (!robot_name_.empty() && robot_name_.front() == '/') robot_name_.erase(robot_name_.begin());
    std::filesystem::create_directories(csv_dir_);
    csv_path_ = (std::filesystem::path(csv_dir_) / (timestamp() + "_force_control.csv")).string();
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_) throw std::runtime_error("Cannot open " + csv_path_);
    write_header();
    poses_sub_ = create_subscription<PosesMsg>("/poses", rclcpp::SensorDataQoS(), std::bind(&DataLoggingNode::poses, this, _1));
    cf_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      cf_ns_ + "/pose", 10, std::bind(&DataLoggingNode::cf_pose, this, _1));
    bind("cf_ee_tracking", 6, &DataLoggingNode::ee_tracking_);
    bind("cf_force_raw", 4, &DataLoggingNode::force_raw_);
    bind("cf_mob_torque_bar", 6, &DataLoggingNode::torque_bar_);
    bind("cf_contact_force", 3, &DataLoggingNode::contact_force_);
    bind("cf_normal_eta", 6, &DataLoggingNode::normal_velocity_);
    bind("cf_mob_input", 6, &DataLoggingNode::mob_input_);
    bind("cf_mob_actuation", 6, &DataLoggingNode::mob_actuation_);
    bind("cf_imu_raw", 3, &DataLoggingNode::imu_acc_raw_);
    tag_sub_ = create_subscription<std_msgs::msg::String>("/flying_pen/debug_log_filename_tag", 10, std::bind(&DataLoggingNode::tag, this, _1));
    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(publish_topic_, 10);
    fw_cmd_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("/fw_cmd", 10);
    startup_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(1.0, startup_check_delay_sec_)),
      std::bind(&DataLoggingNode::check_topics, this));
    RCLCPP_INFO(get_logger(), "Flight-data CSV: %s", csv_path_.c_str());
  }
  ~DataLoggingNode() override { csv_.flush(); }
  double loop_hz() const { return loop_hz_; }
  void write_row() {
    const auto now = Clock::now();
    const double t = std::chrono::duration<double>(now - start_).count();
    const double dt_ms = std::chrono::duration<double, std::milli>(now - last_row_).count(); last_row_ = now;
    std::vector<double> row; row.reserve(51); row.push_back(t); row.push_back(dt_ms);
    append(row, mocap_xyz_); append(row, wall_normal_); append(row, ee_tracking_);
    append(row, force_raw_); append(row, torque_bar_); append(row, contact_force_);
    append(row, normal_velocity_); append(row, mob_input_); append(row, mob_actuation_);
    append(row, imu_acc_raw_); append(row, attitude_rpy_);
    csv_ << std::fixed << std::setprecision(9);
    for (size_t i = 0; i < row.size(); ++i) {
      csv_ << (i ? "," : "") << row[i];
    }
    csv_ << '\n';
    if (t - last_flush_sec_ >= 1.0) { csv_.flush(); last_flush_sec_ = t; }
    std_msgs::msg::Float64MultiArray msg; msg.data = row;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto mask_missing = [this, &msg, nan](const std::string & topic, size_t first, size_t count) {
      if (!received_.at(topic)) for (size_t i=first; i<first+count; ++i) msg.data[i]=nan;
    };
    mask_missing("cf_ee_tracking", 8, 6);
    mask_missing("cf_force_raw", 14, 4);
    mask_missing("cf_mob_torque_bar", 18, 6);
    mask_missing("cf_contact_force", 24, 3);
    mask_missing("cf_normal_eta", 27, 6);
    mask_missing("cf_mob_input", 33, 6);
    mask_missing("cf_mob_actuation", 39, 6);
    mask_missing("cf_imu_raw", 45, 3);
    if (!cf_pose_received_) for (size_t i=48; i<51; ++i) msg.data[i]=nan;
    publisher_->publish(msg);
    if (received_.at("cf_ee_tracking")) {
      geometry_msgs::msg::PointStamped fw_cmd;
      fw_cmd.header.stamp = get_clock()->now(); fw_cmd.header.frame_id = "world";
      fw_cmd.point.x = ee_tracking_[0]; fw_cmd.point.y = ee_tracking_[1]; fw_cmd.point.z = ee_tracking_[2];
      fw_cmd_publisher_->publish(fw_cmd);
    }
  }
private:
  template<size_t N> static void append(std::vector<double> & row, const std::array<double, N> & a) { row.insert(row.end(), a.begin(), a.end()); }
  template<size_t N> void bind(const std::string & topic, size_t expected, std::array<double, N> DataLoggingNode::* field) {
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
    for (const auto & p : msg->poses) {
      if (p.name == robot_name_) {
        mocap_xyz_ = {p.pose.position.x, p.pose.position.y, p.pose.position.z};
        mocap_received_ = true;
      } else if (p.name == "tilted_wall") {
        // RViz defines the wall's outward normal as local +X.
        const auto & q = p.pose.orientation;
        const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        if (norm > 1.0e-12) {
          const double x=q.x/norm, y=q.y/norm, z=q.z/norm, w=q.w/norm;
          wall_normal_ = {1.0-2.0*(y*y+z*z), 2.0*(x*y+w*z), 2.0*(x*z-w*y)};
        }
      }
    }
  }
  void cf_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto & q = msg->pose.orientation;
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (norm <= 1.0e-12) return;
    const double x=q.x/norm, y=q.y/norm, z=q.z/norm, w=q.w/norm;
    const double sinr_cosp = 2.0 * (w*x + y*z);
    const double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
    const double sinp = std::clamp(2.0 * (w*y - z*x), -1.0, 1.0);
    const double siny_cosp = 2.0 * (w*z + x*y);
    const double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
    constexpr double rad_to_deg = 57.2957795130823208768;
    attitude_rpy_ = {
      std::atan2(sinr_cosp, cosr_cosp) * rad_to_deg,
      std::asin(sinp) * rad_to_deg,
      std::atan2(siny_cosp, cosy_cosp) * rad_to_deg};
    cf_pose_received_ = true;
  }
  void check_topics() {
    if (!mocap_received_) RCLCPP_ERROR(get_logger(), "No /poses entry for '%s'; mocap columns remain zero", robot_name_.c_str());
    if (!cf_pose_received_) RCLCPP_ERROR(get_logger(), "No data on %s/pose; attitude columns remain zero", cf_ns_.c_str());
    for (const auto & x : received_) if (!x.second) RCLCPP_ERROR(get_logger(), "No data on %s/%s; verify YAML and firmware TOC", cf_ns_.c_str(), x.first.c_str());
    startup_timer_->cancel();
  }
  void tag(const std_msgs::msg::String::SharedPtr msg) {
    const std::string clean = sanitize(msg->data); if (clean.empty()) return;
    csv_.flush(); csv_.close(); const auto old = std::filesystem::path(csv_path_);
    const auto next = old.parent_path() / (timestamp() + "_force_control_" + clean + ".csv");
    std::error_code ec; std::filesystem::rename(old, next, ec);
    if (ec) { RCLCPP_ERROR(get_logger(), "CSV rename failed: %s", ec.message().c_str()); csv_.open(old, std::ios::app); }
    else { csv_path_ = next.string(); csv_.open(next, std::ios::app); RCLCPP_INFO(get_logger(), "Flight-data CSV renamed: %s", csv_path_.c_str()); }
  }
  void write_header() {
    csv_ << "t_sec,loggerDtMs,mocapRawX,mocapRawY,mocapRawZ,"
      "wallNormalX,wallNormalY,wallNormalZ,"
      "fwCmdX,fwCmdY,fwCmdZ,fwEePosX,fwEePosY,fwEePosZ,"
      "forceCmd,mobForceX,mobForceY,mobForceZ,"
      "mobTorqueX,mobTorqueY,mobTorqueZ,mobForceBarX,mobForceBarY,mobForceBarZ,"
      "mobForceHatCX,mobForceHatCY,mobForceHatCZ,"
      "forceNormalEstX,forceNormalEstY,forceNormalEstZ,fwEeVelX,fwEeVelY,fwEeVelZ,"
      "mobInputForceX,mobInputForceY,mobInputForceZ,mobInputTorqueX,mobInputTorqueY,mobInputTorqueZ,"
      "motorThrust1,motorThrust2,motorThrust3,motorThrust4,batteryVoltage,etaT,"
      "imuAccRawX,imuAccRawY,imuAccRawZ,attitudeRoll,attitudePitch,attitudeYaw\n"; csv_.flush();
  }
  std::string csv_dir_, csv_path_, cf_ns_, robot_name_, publish_topic_; std::ofstream csv_;
  double loop_hz_{50.0}, startup_check_delay_sec_{15.0}, last_flush_sec_{0.0}; Clock::time_point start_, last_row_;
  bool mocap_received_{false}, cf_pose_received_{false};
  std::array<double,3> mocap_xyz_{0.0,0.0,0.0}, wall_normal_{0.0,0.0,0.0}, contact_force_{0.0,0.0,0.0};
  std::array<double,4> force_raw_{0.0,0.0,0.0,0.0};
  std::array<double,6> ee_tracking_{0.0,0.0,0.0,0.0,0.0,0.0}, torque_bar_{0.0,0.0,0.0,0.0,0.0,0.0};
  std::array<double,6> normal_velocity_{0.0,0.0,0.0,0.0,0.0,0.0};
  std::array<double,6> mob_input_{0.0,0.0,0.0,0.0,0.0,0.0}, mob_actuation_{0.0,0.0,0.0,0.0,0.0,0.0};
  std::array<double,3> imu_acc_raw_{0.0,0.0,0.0}, attitude_rpy_{0.0,0.0,0.0};
  std::map<std::string,bool> received_; std::vector<rclcpp::Subscription<LogMsg>::SharedPtr> subscriptions_;
  rclcpp::Subscription<PosesMsg>::SharedPtr poses_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tag_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr fw_cmd_publisher_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); auto node = std::make_shared<DataLoggingNode>(); rclcpp::Rate rate(node->loop_hz());
  while (rclcpp::ok()) { rclcpp::spin_some(node); node->write_row(); rate.sleep(); } rclcpp::shutdown(); return 0;
}
