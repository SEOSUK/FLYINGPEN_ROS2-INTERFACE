#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <crazyflie_interfaces/msg/log_data_generic.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <crazyflie_interfaces/msg/position_control.hpp>
#include <crazyflie_interfaces/msg/status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <ncurses.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
  explicit CommandPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("command_publisher", options)
  {
    cf_position_pub_ =
      this->create_publisher<crazyflie_interfaces::msg::Position>(
        "cf2/cmd_position",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    position_control_pub_ =
      this->create_publisher<crazyflie_interfaces::msg::PositionControl>(
        "cf2/cmd_position_control",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    key_pub_ = this->create_publisher<std_msgs::msg::String>(
      "keyboard_input", 10);

    calibration_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "calibration_status", 10,
      std::bind(&CommandPublisher::calibrationStatusCallback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<crazyflie_interfaces::msg::Status>(
      "cf2/status", 10,
      std::bind(&CommandPublisher::statusCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "cf2/pose", 10,
      std::bind(&CommandPublisher::poseCallback, this, std::placeholders::_1));
    fw_cmd_sub_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "cf2/flying_pen_normal_state", 10,
      std::bind(&CommandPublisher::fwCmdPositionCallback, this, std::placeholders::_1));
    mob_force_sub_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "cf2/cf_contact_force", 10,
      std::bind(&CommandPublisher::mobForceCallback, this, std::placeholders::_1));
    fp_normal_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "cf2/flying_pen/fp_normal", 10,
      std::bind(&CommandPublisher::fpNormalCallback, this, std::placeholders::_1));

    position_tick_ = declareVector3Parameter("position_tick", {0.1, 0.1, 0.2});
    velocity_tick_ = declareVector3Parameter("velocity_tick", {0.1, 0.1, 0.1});
    yaw_tick_deg_ = this->declare_parameter<double>("yaw_tick_deg", 2.0);
    force_delta_ = this->declare_parameter<double>("force_tick", 0.01);
    auto_approach_speed_ = this->declare_parameter<double>("auto_approach_speed", 0.12);
    auto_target_z_ = this->declare_parameter<double>("auto_target_z", 0.86);
    auto_target_y_ = this->declare_parameter<double>("auto_target_y", 0.0);
    auto_target_x_ = this->declare_parameter<double>("auto_target_x", 1.60);
    auto_position_tolerance_ = this->declare_parameter<double>("auto_position_tolerance", 0.015);
    auto_t2_speed_ = this->declare_parameter<double>("auto_t2_speed", 0.09);
    auto_contact_settle_sec_ = this->declare_parameter<double>("auto_contact_settle_sec", 10.0);
    auto_motion_sec_ = this->declare_parameter<double>("auto_motion_sec", 6.0);
    auto_second_down_extra_sec_ =
      this->declare_parameter<double>("auto_second_down_extra_sec", 1.5);
    auto_wait_sec_ = this->declare_parameter<double>("auto_wait_sec", 6.0);
    auto_fourth_wait_sec_ = this->declare_parameter<double>("auto_fourth_wait_sec", 10.0);
    auto_final_wait_sec_ = this->declare_parameter<double>("auto_final_wait_sec", 8.0);
    h_auto_approach_speed_ = this->declare_parameter<double>("h_auto_approach_speed", 0.12);
    h_auto_target_x_ = this->declare_parameter<double>("h_auto_target_x", 1.60);
    h_auto_target_y_ = this->declare_parameter<double>("h_auto_target_y", 0.02);
    h_auto_target_z_ = this->declare_parameter<double>("h_auto_target_z", 0.87);
    h_position_tolerance_ = this->declare_parameter<double>("h_position_tolerance", 0.01);
    h_force_command_ = this->declare_parameter<double>("h_force_command", 0.024);
    h_contact_settle_sec_ = this->declare_parameter<double>("h_contact_settle_sec", 3.0);
    h_d_motion_sec_ = this->declare_parameter<double>("h_d_motion_sec", 2.0);
    h_motion_speed_ = this->declare_parameter<double>("h_motion_speed", 0.06);
    h_final_wait_sec_ = this->declare_parameter<double>("h_final_wait_sec", 2.0);
    box_start_x_ = this->declare_parameter<double>("box_start_x", 0.475);
    box_start_y_ = this->declare_parameter<double>("box_start_y", 0.141);
    box_start_z_ = this->declare_parameter<double>("box_start_z", 0.80);
    box_home_x_ = this->declare_parameter<double>("box_home_x", 0.0);
    box_home_y_ = this->declare_parameter<double>("box_home_y", 0.0);
    box_home_z_ = this->declare_parameter<double>("box_home_z", 0.0);
    box_t1_angle_deg_ = this->declare_parameter<double>("box_t1_angle_deg", 20.0);
    box_height_m_ = this->declare_parameter<double>("box_height_m", 0.12);
    box_t1_speed_ = this->declare_parameter<double>("box_t1_speed", 0.06);
    box_t2_max_speed_ = this->declare_parameter<double>("box_t2_max_speed", 0.06);
    box_t2_kp_ = this->declare_parameter<double>("box_t2_kp", 0.8);
    box_z_tolerance_ = this->declare_parameter<double>("box_z_tolerance", 0.015);
    box_angle_tolerance_deg_ =
      this->declare_parameter<double>("box_angle_tolerance_deg", 2.0);
    box_t1_direction_sign_ = this->declare_parameter<double>("box_t1_direction_sign", 1.0);
    box_force_command_ = this->declare_parameter<double>("box_force_command", 0.024);
    box_contact_settle_sec_ =
      this->declare_parameter<double>("box_contact_settle_sec", 4.0);
    box_approach_speed_ = this->declare_parameter<double>("box_approach_speed", 0.12);
    box_return_speed_ = this->declare_parameter<double>("box_return_speed", 0.12);
    box_position_tolerance_ =
      this->declare_parameter<double>("box_position_tolerance", 0.025);
    box_stage_timeout_sec_ =
      this->declare_parameter<double>("box_stage_timeout_sec", 35.0);
    box_feedback_timeout_sec_ =
      this->declare_parameter<double>("box_feedback_timeout_sec", 0.75);
    box_max_t1_travel_m_ = this->declare_parameter<double>("box_max_t1_travel_m", 0.35);
    box_min_t2_vertical_projection_ =
      this->declare_parameter<double>("box_min_t2_vertical_projection", 0.35);
    command_frame_ = this->declare_parameter<std::string>("command_frame", loadSharedCommandFrameDefault());
    end_effector_offset_ = declareVector3Parameter("end_effector_offset", loadSharedEndEffectorOffsetDefault());

    cmd_xyz_yaw_.fill(0.0);
    force_des_ = 0.0;
    current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_POSITION;
    current_command_reference_ = parseCommandReference(command_frame_);
    mob_force_.fill(std::numeric_limits<double>::quiet_NaN());
    latest_battery_voltage_ = std::numeric_limits<double>::quiet_NaN();
    displayed_battery_voltage_ = std::numeric_limits<double>::quiet_NaN();
    status_msg_ = "ready";
    last_battery_display_update_ = std::chrono::steady_clock::now();
    has_latest_pose_ = false;
    has_latest_fw_cmd_ = false;
    has_latest_fp_normal_ = false;
    command_initialized_ = true;

    last_inputs_.clear();
    for (size_t i = 0; i < HISTORY_LEN; ++i) {
      last_inputs_.push_back("-");
    }

    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    if (has_colors()) {
      start_color();
      use_default_colors();
      init_pair(1, COLOR_GREEN, -1);
    }

    drawLayout();
    publishPositionControl();

    RCLCPP_INFO(
      this->get_logger(),
      "command_publisher started. position_tick=(%.3f, %.3f, %.3f) velocity_tick=(%.3f, %.3f, %.3f)",
      position_tick_[0], position_tick_[1], position_tick_[2],
      velocity_tick_[0], velocity_tick_[1], velocity_tick_[2]);

    timer_ = this->create_wall_timer(
      50ms, std::bind(&CommandPublisher::timerCallback, this));
  }

  ~CommandPublisher() override
  {
    endwin();
  }

private:
  static constexpr int ROW_USAGE_HEADER = 0;
  static constexpr int ROW_USAGE_1 = 2;
  static constexpr int ROW_USAGE_2 = 3;
  static constexpr int ROW_USAGE_3 = 4;

  static constexpr int ROW_STATUS_HEADER = 6;
  static constexpr int ROW_STATUS_MODE = 8;
  static constexpr int ROW_STATUS_FORCE = 10;
  static constexpr int ROW_STATUS_MOB = 11;
  static constexpr int ROW_STATUS_BATTERY = 12;
  static constexpr int ROW_STATUS_MSG = 14;

  static constexpr int ROW_CMD_HEADER = 16;
  static constexpr int ROW_CMD_LINE1 = 18;
  static constexpr int ROW_CMD_LINE2 = 19;
  static constexpr int ROW_CMD_HIST_HDR = 21;
  static constexpr int ROW_CMD_HIST_0 = 22;
  static constexpr size_t HISTORY_LEN = 5;

  std::array<double, 3> declareVector3Parameter(
    const std::string & name,
    const std::vector<double> & defaultValue)
  {
    const auto values = this->declare_parameter<std::vector<double>>(name, defaultValue);
    std::array<double, 3> result{{0.0, 0.0, 0.0}};

    if (values.size() != 3) {
      RCLCPP_WARN(
        this->get_logger(),
        "Parameter '%s' must contain exactly 3 values. Using zeros.",
        name.c_str());
      return result;
    }

    for (size_t i = 0; i < 3; ++i) {
      result[i] = values[i];
    }

    return result;
  }

  std::string loadSharedCommandFrameDefault() const
  {
    const auto shared_yaml =
      ament_index_cpp::get_package_share_directory("crazyflie") + "/config/su_params.yaml";
    std::ifstream file(shared_yaml);
    if (!file.is_open()) {
      return "end_effector";
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.find("command_reference:") == std::string::npos) {
        continue;
      }

      const auto colon = line.find(':');
      if (colon == std::string::npos || colon + 1 >= line.size()) {
        break;
      }

      std::string value = line.substr(colon + 1);
      value.erase(0, value.find_first_not_of(" \t"));
      const auto last = value.find_last_not_of(" \t\r\n");
      if (last != std::string::npos) {
        value.erase(last + 1);
      } else {
        value.clear();
      }

      return value == "drone" ? "drone" : "end_effector";
    }

    return "end_effector";
  }

  std::vector<double> loadSharedEndEffectorOffsetDefault() const
  {
    const auto shared_yaml =
      ament_index_cpp::get_package_share_directory("crazyflie") + "/config/su_params.yaml";
    std::ifstream file(shared_yaml);
    if (!file.is_open()) {
      return {0.1, 0.0, 0.04};
    }

    bool insideSuWrench = false;
    std::array<double, 3> offset{{0.1, 0.0, 0.04}};
    std::array<bool, 3> found{{false, false, false}};
    std::string line;
    while (std::getline(file, line)) {
      if (!insideSuWrench) {
        if (line.find("su_wrench:") != std::string::npos) {
          insideSuWrench = true;
        }
        continue;
      }

      if (line.rfind("      su_", 0) == 0 && line.find("su_wrench:") == std::string::npos) {
        break;
      }

      auto parseValue = [&](const std::string & key, double & dst, bool & ok) {
        if (line.find(key) == std::string::npos) {
          return;
        }
        const auto colon = line.find(':');
        if (colon == std::string::npos || colon + 1 >= line.size()) {
          return;
        }
        try {
          dst = std::stod(line.substr(colon + 1));
          ok = true;
        } catch (...) {
        }
      };

      parseValue("rOffX:", offset[0], found[0]);
      parseValue("rOffY:", offset[1], found[1]);
      parseValue("rOffZ:", offset[2], found[2]);
      if (found[0] && found[1] && found[2]) {
        return {offset[0], offset[1], offset[2]};
      }
    }

    return {offset[0], offset[1], offset[2]};
  }

  void timerCallback()
  {
    int ch = 0;
    while ((ch = getch()) != ERR) {
      handleKey(static_cast<char>(ch));
    }

    updateAutoTrajectory();
    updateHTrajectory();
    updateBoxTrajectory();
    updateDisplayedBatteryVoltage();
    publishPositionCmd();
    drawStatusBlock();
    drawCommandBlock();
  }

  void handleKey(char c)
  {
    const bool any_auto_active =
      auto_trajectory_active_ || h_trajectory_active_ || box_trajectory_active_;
    if (any_auto_active && c == 'p') {
      cancelAutoTrajectory("emergency DISARM");
      cancelHTrajectory("emergency DISARM");
      cancelBoxTrajectory("emergency DISARM", false);
      cmd_xyz_yaw_[0] = cmd_xyz_yaw_[1] = cmd_xyz_yaw_[2] = 0.0;
      publishKeyboardTrigger('p');
      status_msg_ = "AUTO emergency DISARM requested";
      return;
    }
    if ((auto_trajectory_active_ && c != 'g') ||
        (h_trajectory_active_ && c != 'h') ||
        (box_trajectory_active_ && c != 'b')) {
      // An interrupt has exactly the combined safety effect of pressing x, then u:
      // zero the active command and return to world-frame position mode.
      resetActiveCommand();
      setPositionMode(crazyflie_interfaces::msg::PositionControl::MODE_POSITION);
      status_msg_ = std::string("AUTO interrupted by '") + c + "'; applied x + u";
      pushInputHistory(std::string(1, c) + " : AUTO CANCELED -> x + u");
      return;
    }
    if (c == 'w')       { onPositiveX(); }
    else if (c == 's')  { onNegativeX(); }
    else if (c == 'a')  { onPositiveY(); }
    else if (c == 'd')  { onNegativeY(); }
    else if (c == 'e')  { onPositiveZ(); }
    else if (c == 'q')  { onNegativeZ(); }
    else if (c == 'z')  { cmd_xyz_yaw_[3] += yaw_tick_deg_; pushInputHistory("z : yaw += tick"); }
    else if (c == 'c')  { cmd_xyz_yaw_[3] -= yaw_tick_deg_; pushInputHistory("c : yaw -= tick"); }
    else if (c == 'x')  { resetActiveCommand(); }
    else if (c == 'i')  { setPositionMode(crazyflie_interfaces::msg::PositionControl::MODE_VELOCITY); }
    else if (c == 'u')  { setPositionMode(crazyflie_interfaces::msg::PositionControl::MODE_POSITION); }
    else if (c == 'f')  { publishKeyboardTrigger('f'); }
    else if (c == 'g')  { startAutoTrajectory(); }
    else if (c == 'h')  { startHTrajectory(); }
    else if (c == 'b')  { startBoxTrajectory(); }
    else if (c == 'j')  { force_des_ += force_delta_; publishPositionControl(); updateForceStatus(); pushInputHistory("j : force += tick"); }
    else if (c == 'k')  { force_des_ -= force_delta_; publishPositionControl(); updateForceStatus(); pushInputHistory("k : force -= tick"); }
    else if (c == 'l')  { force_des_ = 0.0; publishPositionControl(); updateForceStatus(); pushInputHistory("l : force reset"); }
    else if (c == 'o' || c == 'p') {
      publishKeyboardTrigger(c);
    } else if (c == 't') {
      pushInputHistory("t : quit");
      status_msg_ = "exit key pressed";
      rclcpp::shutdown();
    }
  }

  bool isContactFrameVelocityMode() const
  {
    return current_mode_ == crazyflie_interfaces::msg::PositionControl::MODE_VELOCITY;
  }

  bool usesVelocityCommands() const
  {
    return true;
  }

  uint8_t parseCommandReference(const std::string & frame) const
  {
    return frame == "drone" ?
      crazyflie_interfaces::msg::PositionControl::REFERENCE_DRONE :
      crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR;
  }

  bool isEndEffectorReference() const
  {
    return current_command_reference_ == crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR;
  }

  void onPositiveX()
  {
    cancelAutoTrajectory("manual velocity command");
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[0] += velocity_tick_[0];
      pushInputHistory("w : vx += tick");
    } else {
      cmd_xyz_yaw_[0] += position_tick_[0];
      pushInputHistory("w : x += tick");
    }
  }

  void onNegativeX()
  {
    cancelAutoTrajectory("manual velocity command");
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[0] -= velocity_tick_[0];
      pushInputHistory("s : vx -= tick");
    } else {
      cmd_xyz_yaw_[0] -= position_tick_[0];
      pushInputHistory("s : x -= tick");
    }
  }

  void onPositiveY()
  {
    cancelAutoTrajectory("manual velocity command");
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[1] += velocity_tick_[1];
      pushInputHistory("a : vy += tick");
    } else {
      cmd_xyz_yaw_[1] += position_tick_[1];
      pushInputHistory("a : y += tick");
    }
  }

  void onNegativeY()
  {
    cancelAutoTrajectory("manual velocity command");
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[1] -= velocity_tick_[1];
      pushInputHistory("d : vy -= tick");
    } else {
      cmd_xyz_yaw_[1] -= position_tick_[1];
      pushInputHistory("d : y -= tick");
    }
  }

  void onPositiveZ()
  {
    cancelAutoTrajectory("manual velocity command");
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[2] += velocity_tick_[2];
      pushInputHistory("e : vz += tick");
    } else {
      cmd_xyz_yaw_[2] += position_tick_[2];
      pushInputHistory("e : z += tick");
    }
  }

  void onNegativeZ()
  {
    cancelAutoTrajectory("manual velocity command");
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[2] -= velocity_tick_[2];
      pushInputHistory("q : vz -= tick");
    } else {
      cmd_xyz_yaw_[2] -= position_tick_[2];
      pushInputHistory("q : z -= tick");
    }
  }

  void setPositionMode(uint8_t mode)
  {
    cancelAutoTrajectory("mode changed");
    if (mode == current_mode_) {
      status_msg_ = isContactFrameVelocityMode() ?
        "already in CONTACT-FRAME velocity mode" :
        "already in WORLD-FRAME velocity mode";
      return;
    }
    current_mode_ = mode;
    velocity_mode_command_ready_time_ = std::chrono::steady_clock::time_point{};
    publishPositionControl();
    if (isContactFrameVelocityMode()) {
      status_msg_ = "contact-frame velocity mode enabled";
      pushInputHistory("i : contact-frame velocity ON");
    } else {
      status_msg_ = "world-frame velocity mode enabled";
      pushInputHistory("u : contact-frame velocity OFF");
    }
  }

  void setCommandReference(uint8_t commandReference)
  {
    if (commandReference == current_command_reference_) {
      status_msg_ = isEndEffectorReference() ? "already in END_EFFECTOR reference" : "already in DRONE reference";
      return;
    }

    if (has_latest_pose_) {
      const auto latest_reference_pose =
        commandReference == crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR ?
          latestPoseInReferenceFrame(crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR) :
          latestPoseInReferenceFrame(crazyflie_interfaces::msg::PositionControl::REFERENCE_DRONE);
      cmd_xyz_yaw_[0] = latest_reference_pose[0];
      cmd_xyz_yaw_[1] = latest_reference_pose[1];
      cmd_xyz_yaw_[2] = latest_reference_pose[2];
      cmd_xyz_yaw_[3] = latest_pose_xyz_yaw_[3];
    }

    current_command_reference_ = commandReference;
    command_frame_ = isEndEffectorReference() ? "end_effector" : "drone";
    publishPositionControl();
    if (isEndEffectorReference()) {
      status_msg_ = "command reference set to END_EFFECTOR";
      pushInputHistory("g : command ref -> end_effector");
    } else {
      status_msg_ = "command reference set to DRONE";
      pushInputHistory("f : command ref -> drone");
    }
  }

  void resetActiveCommand()
  {
    cancelAutoTrajectory("x pressed");
    cancelHTrajectory("x pressed");
    cancelBoxTrajectory("x pressed", false);
    if (usesVelocityCommands()) {
      cmd_xyz_yaw_[0] = 0.0;
      cmd_xyz_yaw_[1] = 0.0;
      cmd_xyz_yaw_[2] = 0.0;
      status_msg_ = "velocity command reset to zero";
      pushInputHistory("x : zero velocity cmd");
    } else {
      if (has_latest_pose_) {
        const auto latest_reference_pose = latestPoseInActiveReferenceFrame();
        cmd_xyz_yaw_[0] = latest_reference_pose[0];
        cmd_xyz_yaw_[1] = latest_reference_pose[1];
        cmd_xyz_yaw_[2] = latest_reference_pose[2];
        cmd_xyz_yaw_[3] = latest_pose_xyz_yaw_[3];
        status_msg_ = "position command aligned to latest pose";
        pushInputHistory("x : hold current pose");
      } else {
        cmd_xyz_yaw_[0] = 0.0;
        cmd_xyz_yaw_[1] = 0.0;
        cmd_xyz_yaw_[2] = 0.0;
        status_msg_ = "position command reset to zero";
        pushInputHistory("x : reset position cmd");
      }
    }
  }

  enum class AutoStage
  {
    ApproachZ,
    ApproachY,
    ApproachX,
    ContactSettle,
    Up1,
    Wait1,
    Up2,
    Wait2,
    Down1,
    Wait3,
    Down2,
    Wait4,
    ReturnX,
    ReturnZ,
  };

  void setAutoStage(const AutoStage stage, const char * status)
  {
    auto_stage_ = stage;
    auto_stage_start_time_ = std::chrono::steady_clock::now();
    cmd_xyz_yaw_[0] = 0.0;
    cmd_xyz_yaw_[1] = 0.0;
    cmd_xyz_yaw_[2] = 0.0;
    status_msg_ = status;
  }

  void startAutoTrajectory()
  {
    if (auto_trajectory_active_) {
      status_msg_ = "automatic trajectory already active";
      return;
    }
    if (!has_latest_fw_cmd_) {
      status_msg_ = "AUTO rejected: waiting for firmware command log";
      pushInputHistory("g : AUTO rejected (no firmware command)");
      return;
    }
    const bool parameters_valid =
      std::isfinite(auto_approach_speed_) && auto_approach_speed_ > 0.0 &&
      std::isfinite(auto_position_tolerance_) && auto_position_tolerance_ > 0.0 &&
      std::isfinite(auto_t2_speed_) && auto_t2_speed_ > 0.0 &&
      std::isfinite(auto_contact_settle_sec_) && auto_contact_settle_sec_ >= 0.0 &&
      std::isfinite(auto_motion_sec_) && auto_motion_sec_ > 0.0 &&
      std::isfinite(auto_second_down_extra_sec_) && auto_second_down_extra_sec_ >= 0.0 &&
      std::isfinite(auto_wait_sec_) && auto_wait_sec_ >= 0.0 &&
      std::isfinite(auto_fourth_wait_sec_) && auto_fourth_wait_sec_ >= 0.0 &&
      std::isfinite(auto_final_wait_sec_) && auto_final_wait_sec_ >= 0.0;
    if (!parameters_valid) {
      status_msg_ = "AUTO rejected: invalid command.yaml parameters";
      pushInputHistory("g : AUTO rejected (invalid params)");
      return;
    }

    // Firmware MODE_POSITION integrates these world-frame velocity-like commands.
    current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_POSITION;
    velocity_mode_command_ready_time_ = std::chrono::steady_clock::time_point{};
    publishPositionControl();
    auto_trajectory_active_ = true;
    setAutoStage(AutoStage::ApproachZ, "AUTO: approach command Z -> 0.86 m");
    pushInputHistory("g : start automatic surface trajectory");
  }

  void cancelAutoTrajectory(const char * reason)
  {
    if (!auto_trajectory_active_) {
      return;
    }
    auto_trajectory_active_ = false;
    cmd_xyz_yaw_[0] = 0.0;
    cmd_xyz_yaw_[1] = 0.0;
    cmd_xyz_yaw_[2] = 0.0;
    status_msg_ = std::string("AUTO canceled: ") + reason;
  }

  bool commandAxisTowardTarget(const std::size_t axis, const double target)
  {
    const auto command = latestFwCommandInActiveReferenceFrame();
    const double error = target - command[axis];
    cmd_xyz_yaw_[0] = 0.0;
    cmd_xyz_yaw_[1] = 0.0;
    cmd_xyz_yaw_[2] = 0.0;
    if (std::abs(error) <= auto_position_tolerance_) {
      return true;
    }
    cmd_xyz_yaw_[axis] = std::copysign(std::min(auto_approach_speed_, std::abs(error)), error);
    return false;
  }

  void updateAutoTrajectory()
  {
    if (!auto_trajectory_active_) {
      return;
    }
    if (!has_latest_fw_cmd_) {
      cancelAutoTrajectory("firmware command log lost");
      return;
    }

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - auto_stage_start_time_).count();
    switch (auto_stage_) {
      case AutoStage::ApproachZ:
        if (commandAxisTowardTarget(2, auto_target_z_)) {
          setAutoStage(AutoStage::ApproachY, "AUTO: approach command Y -> 0.00 m");
        }
        break;
      case AutoStage::ApproachY:
        if (commandAxisTowardTarget(1, auto_target_y_)) {
          setAutoStage(AutoStage::ApproachX, "AUTO: approach command X -> 1.60 m");
        }
        break;
      case AutoStage::ApproachX:
        if (commandAxisTowardTarget(0, auto_target_x_)) {
          current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_VELOCITY;
          force_des_ += force_delta_;  // Same effect as one 'j' key press.
          publishPositionControl();
          setAutoStage(AutoStage::ContactSettle, "AUTO: i + j complete; wait 10 s");
        }
        break;
      case AutoStage::ContactSettle:
        if (elapsed >= auto_contact_settle_sec_) {
          setAutoStage(AutoStage::Up1, "AUTO: +t2 pass 1/4");
        }
        break;
      case AutoStage::Up1:
        cmd_xyz_yaw_[2] = auto_t2_speed_;
        if (elapsed >= auto_motion_sec_) setAutoStage(AutoStage::Wait1, "AUTO: wait 1/4");
        break;
      case AutoStage::Wait1:
        if (elapsed >= auto_wait_sec_) setAutoStage(AutoStage::Up2, "AUTO: +t2 pass 2/4");
        break;
      case AutoStage::Up2:
        cmd_xyz_yaw_[2] = auto_t2_speed_;
        if (elapsed >= auto_motion_sec_) setAutoStage(AutoStage::Wait2, "AUTO: wait before -t2 pass");
        break;
      case AutoStage::Wait2:
        if (elapsed >= auto_wait_sec_) setAutoStage(AutoStage::Down1, "AUTO: -t2 pass 3/4");
        break;
      case AutoStage::Down1:
        cmd_xyz_yaw_[2] = -auto_t2_speed_;
        if (elapsed >= auto_motion_sec_) setAutoStage(AutoStage::Wait3, "AUTO: wait 3/4");
        break;
      case AutoStage::Wait3:
        if (elapsed >= auto_fourth_wait_sec_) {
          setAutoStage(AutoStage::Down2, "AUTO: -t2 pass 4/4");
        }
        break;
      case AutoStage::Down2:
        cmd_xyz_yaw_[2] = -auto_t2_speed_;
        if (elapsed >= auto_motion_sec_ + auto_second_down_extra_sec_) {
          setAutoStage(AutoStage::Wait4, "AUTO: final contact wait");
        }
        break;
      case AutoStage::Wait4:
        if (elapsed >= auto_final_wait_sec_) {
          // Same mode transition as 'u', without canceling the active sequence.
          current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_POSITION;
          velocity_mode_command_ready_time_ = std::chrono::steady_clock::time_point{};
          publishPositionControl();
          setAutoStage(AutoStage::ReturnX, "AUTO: u complete; return command X -> 0");
        }
        break;
      case AutoStage::ReturnX:
        if (commandAxisTowardTarget(0, 0.0)) {
          setAutoStage(AutoStage::ReturnZ, "AUTO: return command Z -> 0");
        }
        break;
      case AutoStage::ReturnZ:
        if (commandAxisTowardTarget(2, 0.0)) {
          auto_trajectory_active_ = false;
          cmd_xyz_yaw_[0] = cmd_xyz_yaw_[1] = cmd_xyz_yaw_[2] = 0.0;
          publishKeyboardTrigger('p');
          status_msg_ = "AUTO complete; returned X/Z to zero and DISARMED";
          pushInputHistory("g : AUTO complete -> u, X=0, Z=0, p");
        }
        break;
    }
  }

  enum class HStage
  {
    ApproachZ,
    ApproachY,
    ApproachX,
    ContactSettle,
    MoveD,
    MoveA,
    FinalWait,
    ReturnX,
    ReturnY,
    ReturnZ,
  };

  void setHStage(const HStage stage, const char * status)
  {
    h_stage_ = stage;
    h_stage_start_time_ = std::chrono::steady_clock::now();
    cmd_xyz_yaw_[0] = 0.0;
    cmd_xyz_yaw_[1] = 0.0;
    cmd_xyz_yaw_[2] = 0.0;
    status_msg_ = status;
  }

  void startHTrajectory()
  {
    if (h_trajectory_active_) {
      status_msg_ = "H AUTO already active";
      return;
    }
    if (auto_trajectory_active_) {
      status_msg_ = "H AUTO rejected: g trajectory is active";
      return;
    }
    if (!has_latest_fw_cmd_ || !has_latest_pose_) {
      status_msg_ = "H AUTO rejected: waiting for command log and pose";
      pushInputHistory("h : AUTO rejected (no command/pose)");
      return;
    }

    const bool parameters_valid =
      std::isfinite(h_auto_approach_speed_) && h_auto_approach_speed_ > 0.0 &&
      std::isfinite(h_auto_target_x_) &&
      std::isfinite(h_auto_target_y_) &&
      std::isfinite(h_auto_target_z_) &&
      std::isfinite(h_position_tolerance_) && h_position_tolerance_ > 0.0 &&
      std::isfinite(h_force_command_) && h_force_command_ >= 0.0 &&
      std::isfinite(h_contact_settle_sec_) && h_contact_settle_sec_ >= 0.0 &&
      std::isfinite(h_d_motion_sec_) && h_d_motion_sec_ > 0.0 &&
      std::isfinite(h_motion_speed_) && h_motion_speed_ > 0.0 &&
      std::isfinite(h_final_wait_sec_) && h_final_wait_sec_ >= 0.0;
    if (!parameters_valid) {
      status_msg_ = "H AUTO rejected: invalid command.yaml parameters";
      pushInputHistory("h : AUTO rejected (invalid params)");
      return;
    }

    current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_POSITION;
    velocity_mode_command_ready_time_ = std::chrono::steady_clock::time_point{};
    publishPositionControl();
    h_reference_crossings_ = 0;
    h_inside_reference_tolerance_ = true;
    h_trajectory_active_ = true;
    setHStage(HStage::ApproachZ, "H AUTO: approach target Z");
    pushInputHistory("h : start automatic return-crossing trajectory");
  }

  void cancelHTrajectory(const char * reason)
  {
    if (!h_trajectory_active_) {
      return;
    }
    h_trajectory_active_ = false;
    cmd_xyz_yaw_[0] = 0.0;
    cmd_xyz_yaw_[1] = 0.0;
    cmd_xyz_yaw_[2] = 0.0;
    status_msg_ = std::string("H AUTO canceled: ") + reason;
  }

  bool hCommandAxisTowardTarget(const std::size_t axis, const double target)
  {
    const auto command = latestFwCommandInActiveReferenceFrame();
    const double error = target - command[axis];
    cmd_xyz_yaw_[0] = 0.0;
    cmd_xyz_yaw_[1] = 0.0;
    cmd_xyz_yaw_[2] = 0.0;
    if (std::abs(error) <= h_position_tolerance_) {
      return true;
    }
    cmd_xyz_yaw_[axis] =
      std::copysign(std::min(h_auto_approach_speed_, std::abs(error)), error);
    return false;
  }

  bool hPoseIsNearReference() const
  {
    const auto pose = latestPoseInActiveReferenceFrame();
    return std::abs(pose[0] - h_reference_xy_[0]) <= h_position_tolerance_ &&
           std::abs(pose[1] - h_reference_xy_[1]) <= h_position_tolerance_;
  }

  void updateHTrajectory()
  {
    if (!h_trajectory_active_) {
      return;
    }
    if (!has_latest_fw_cmd_ || !has_latest_pose_) {
      cancelHTrajectory("command log or pose lost");
      return;
    }

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - h_stage_start_time_).count();
    switch (h_stage_) {
      case HStage::ApproachZ:
        if (hCommandAxisTowardTarget(2, h_auto_target_z_)) {
          setHStage(HStage::ApproachY, "H AUTO: approach target Y");
        }
        break;
      case HStage::ApproachY:
        if (hCommandAxisTowardTarget(1, h_auto_target_y_)) {
          setHStage(HStage::ApproachX, "H AUTO: approach target X");
        }
        break;
      case HStage::ApproachX:
        if (hCommandAxisTowardTarget(0, h_auto_target_x_)) {
          const auto pose = latestPoseInActiveReferenceFrame();
          h_reference_xy_[0] = pose[0];
          h_reference_xy_[1] = pose[1];
          h_inside_reference_tolerance_ = true;
          h_reference_crossings_ = 0;
          current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_VELOCITY;
          force_des_ = h_force_command_;
          publishPositionControl();
          setHStage(HStage::ContactSettle, "H AUTO: i + force complete; settling");
        }
        break;
      case HStage::ContactSettle:
        if (elapsed >= h_contact_settle_sec_) {
          setHStage(HStage::MoveD, "H AUTO: d direction motion");
        }
        break;
      case HStage::MoveD:
        cmd_xyz_yaw_[1] = -h_motion_speed_;
        if (elapsed >= h_d_motion_sec_) {
          h_inside_reference_tolerance_ = hPoseIsNearReference();
          setHStage(HStage::MoveA, "H AUTO: a direction; waiting for first crossing");
        }
        break;
      case HStage::MoveA:
      {
        const bool inside = hPoseIsNearReference();
        if (inside && !h_inside_reference_tolerance_) {
          ++h_reference_crossings_;
          if (h_reference_crossings_ >= 3) {
            setHStage(HStage::FinalWait, "H AUTO: third crossing; final wait");
            break;
          }
          if (h_reference_crossings_ == 2) {
            status_msg_ = "H AUTO: second crossing; a speed doubled";
            pushInputHistory("h : second XY crossing -> speed x2");
          } else {
            status_msg_ = "H AUTO: first crossing; base speed maintained";
            pushInputHistory("h : first XY crossing -> base speed");
          }
        }
        h_inside_reference_tolerance_ = inside;
        cmd_xyz_yaw_[1] = h_motion_speed_ * (h_reference_crossings_ >= 2 ? 2.0 : 1.0);
        break;
      }
      case HStage::FinalWait:
        if (elapsed >= h_final_wait_sec_) {
          current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_POSITION;
          velocity_mode_command_ready_time_ = std::chrono::steady_clock::time_point{};
          publishPositionControl();
          setHStage(HStage::ReturnX, "H AUTO: u complete; return X to zero");
        }
        break;
      case HStage::ReturnX:
        if (hCommandAxisTowardTarget(0, 0.0)) {
          setHStage(HStage::ReturnY, "H AUTO: return Y to zero");
        }
        break;
      case HStage::ReturnY:
        if (hCommandAxisTowardTarget(1, 0.0)) {
          setHStage(HStage::ReturnZ, "H AUTO: return Z to zero");
        }
        break;
      case HStage::ReturnZ:
        if (hCommandAxisTowardTarget(2, 0.0)) {
          h_trajectory_active_ = false;
          cmd_xyz_yaw_[0] = cmd_xyz_yaw_[1] = cmd_xyz_yaw_[2] = 0.0;
          status_msg_ = "H AUTO complete; returned XYZ to zero";
          pushInputHistory("h : AUTO complete -> u, XYZ=0");
        }
        break;
    }
  }

  enum class BoxStage
  {
    ApproachZ,
    ApproachY,
    ApproachX,
    ContactSettle,
    T1Out,
    T2Up,
    T1Back,
    T2Down,
    ReturnX,
    ReturnY,
    ReturnZ,
  };

  void setBoxStage(const BoxStage stage, const char * status)
  {
    box_stage_ = stage;
    box_stage_start_time_ = std::chrono::steady_clock::now();
    cmd_xyz_yaw_[0] = 0.0;
    cmd_xyz_yaw_[1] = 0.0;
    cmd_xyz_yaw_[2] = 0.0;
    status_msg_ = status;
  }

  bool boxParametersValid() const
  {
    const std::array<double, 22> finite_values{{
      box_start_x_, box_start_y_, box_start_z_, box_home_x_, box_home_y_, box_home_z_,
      box_t1_angle_deg_, box_height_m_, box_t1_speed_, box_t2_max_speed_, box_t2_kp_,
      box_z_tolerance_, box_angle_tolerance_deg_, box_t1_direction_sign_,
      box_force_command_, box_contact_settle_sec_, box_approach_speed_, box_return_speed_,
      box_position_tolerance_, box_stage_timeout_sec_, box_feedback_timeout_sec_,
      box_max_t1_travel_m_}};
    if (!std::all_of(
        finite_values.begin(), finite_values.end(),
        [](double value) {return std::isfinite(value);}) ||
      !std::isfinite(box_min_t2_vertical_projection_))
    {
      return false;
    }
    return std::abs(box_t1_angle_deg_) > 0.0 &&
           box_angle_tolerance_deg_ < std::abs(box_t1_angle_deg_) &&
           box_height_m_ > box_z_tolerance_ &&
           box_t1_speed_ > 0.0 && box_t2_max_speed_ > 0.0 && box_t2_kp_ > 0.0 &&
           box_z_tolerance_ > 0.0 && box_angle_tolerance_deg_ > 0.0 &&
           std::abs(box_t1_direction_sign_) > 0.0 && box_force_command_ >= 0.0 &&
           box_contact_settle_sec_ >= 0.0 &&
           box_contact_settle_sec_ <= box_stage_timeout_sec_ &&
           box_approach_speed_ > 0.0 &&
           box_return_speed_ > 0.0 && box_position_tolerance_ > 0.0 &&
           box_stage_timeout_sec_ > 0.0 && box_feedback_timeout_sec_ > 0.0 &&
           box_max_t1_travel_m_ > 0.0 &&
           box_min_t2_vertical_projection_ > 0.0 &&
           box_min_t2_vertical_projection_ <= 1.0;
  }

  void startBoxTrajectory()
  {
    if (box_trajectory_active_) {
      status_msg_ = "BOX AUTO already active";
      return;
    }
    if (auto_trajectory_active_ || h_trajectory_active_) {
      status_msg_ = "BOX AUTO rejected: another automatic trajectory is active";
      return;
    }
    if (!has_latest_fw_cmd_) {
      status_msg_ = "BOX AUTO rejected: waiting for firmware command feedback";
      pushInputHistory("b : AUTO rejected (no command feedback)");
      return;
    }
    if (!boxParametersValid()) {
      status_msg_ = "BOX AUTO rejected: invalid command.yaml parameters";
      pushInputHistory("b : AUTO rejected (invalid params)");
      return;
    }

    current_command_reference_ =
      crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR;
    command_frame_ = "end_effector";
    current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_POSITION;
    velocity_mode_command_ready_time_ = std::chrono::steady_clock::time_point{};
    publishPositionControl();
    box_trajectory_active_ = true;
    setBoxStage(BoxStage::ApproachZ, "BOX AUTO: approach EE Z");
    pushInputHistory("b : start Experiment 3 square drawing");
  }

  void cancelBoxTrajectory(const char * reason, bool leave_contact_mode)
  {
    if (!box_trajectory_active_) {
      return;
    }
    box_trajectory_active_ = false;
    cmd_xyz_yaw_[0] = cmd_xyz_yaw_[1] = cmd_xyz_yaw_[2] = 0.0;
    if (leave_contact_mode) {
      current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_POSITION;
      velocity_mode_command_ready_time_ = std::chrono::steady_clock::time_point{};
      publishPositionControl();
    }
    status_msg_ = std::string("BOX AUTO canceled: ") + reason;
    pushInputHistory(status_msg_);
    RCLCPP_ERROR(this->get_logger(), "%s", status_msg_.c_str());
  }

  bool boxCommandAxisTowardTarget(
    const std::size_t axis, const double target, const double speed)
  {
    const auto command = latestFwCommandInActiveReferenceFrame();
    const double error = target - command[axis];
    cmd_xyz_yaw_[0] = cmd_xyz_yaw_[1] = cmd_xyz_yaw_[2] = 0.0;
    if (std::abs(error) <= box_position_tolerance_) {
      return true;
    }
    cmd_xyz_yaw_[axis] = std::copysign(std::min(speed, std::abs(error)), error);
    return false;
  }

  bool boxFeedbackFresh(
    const std::chrono::steady_clock::time_point & stamp,
    const std::chrono::steady_clock::time_point now) const
  {
    return stamp.time_since_epoch().count() > 0 &&
           std::chrono::duration<double>(now - stamp).count() <= box_feedback_timeout_sec_;
  }

  bool boxContactFrameValid() const
  {
    const auto finite3 = [](const std::array<double, 3> & vector) {
        return std::all_of(
          vector.begin(), vector.end(), [](double value) {return std::isfinite(value);});
      };
    const auto norm3 = [](const std::array<double, 3> & vector) {
        return std::sqrt(
          vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
      };
    const auto dot3 = [](const std::array<double, 3> & a, const std::array<double, 3> & b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
      };
    if (!std::isfinite(box_ee_z_) || !finite3(box_normal_) ||
      !finite3(box_t1_) || !finite3(box_t2_))
    {
      return false;
    }
    const double normal_norm = norm3(box_normal_);
    const double t1_norm = norm3(box_t1_);
    const double t2_norm = norm3(box_t2_);
    if (normal_norm < 0.5 || t1_norm < 0.5 || t2_norm < 0.5 ||
      normal_norm > 1.5 || t1_norm > 1.5 || t2_norm > 1.5 ||
      std::hypot(box_normal_[0], box_normal_[1]) < 1.0e-3)
    {
      return false;
    }
    return std::abs(dot3(box_normal_, box_t1_) / (normal_norm * t1_norm)) < 0.35 &&
           std::abs(dot3(box_normal_, box_t2_) / (normal_norm * t2_norm)) < 0.35 &&
           std::abs(dot3(box_t1_, box_t2_) / (t1_norm * t2_norm)) < 0.35;
  }

  bool boxAngleReached(const double target, const double direction) const
  {
    const double tolerance = box_angle_tolerance_deg_ * M_PI / 180.0;
    return direction > 0.0 ? box_normal_azimuth_unwrapped_ >= target - tolerance :
           box_normal_azimuth_unwrapped_ <= target + tolerance;
  }

  void updateBoxHeightStage(const double target, const BoxStage next, const char * status)
  {
    const double error = target - box_ee_z_;
    if (std::abs(error) <= box_z_tolerance_) {
      setBoxStage(next, status);
      return;
    }
    if (std::abs(box_t2_[2]) < box_min_t2_vertical_projection_) {
      cancelBoxTrajectory("t2 vertical projection below safety threshold", true);
      return;
    }
    const double unsaturated = box_t2_kp_ * error / box_t2_[2];
    cmd_xyz_yaw_[0] = 0.0;
    cmd_xyz_yaw_[1] = 0.0;
    cmd_xyz_yaw_[2] = std::max(
      -box_t2_max_speed_, std::min(box_t2_max_speed_, unsaturated));
    if (!std::isfinite(cmd_xyz_yaw_[2])) {
      cancelBoxTrajectory("non-finite t2 command", true);
    }
  }

  void updateBoxTrajectory()
  {
    if (!box_trajectory_active_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - box_stage_start_time_).count();
    if (!boxFeedbackFresh(last_fw_cmd_time_, now)) {
      cancelBoxTrajectory("firmware command feedback timeout", true);
      return;
    }
    if (elapsed > box_stage_timeout_sec_) {
      cancelBoxTrajectory("stage timeout", true);
      return;
    }

    const bool drawing_stage =
      box_stage_ == BoxStage::T1Out || box_stage_ == BoxStage::T2Up ||
      box_stage_ == BoxStage::T1Back || box_stage_ == BoxStage::T2Down;
    if (drawing_stage &&
      (!has_latest_fp_normal_ || !boxFeedbackFresh(last_fp_normal_time_, now)))
    {
      cancelBoxTrajectory("normal-estimator feedback timeout", true);
      return;
    }
    if (drawing_stage && !boxContactFrameValid()) {
      cancelBoxTrajectory("invalid estimated contact frame", true);
      return;
    }

    const double angle_delta = box_t1_angle_deg_ * M_PI / 180.0;
    const double t1_command_sign =
      std::copysign(1.0, box_t1_angle_deg_) * std::copysign(1.0, box_t1_direction_sign_);
    switch (box_stage_) {
      case BoxStage::ApproachZ:
        if (boxCommandAxisTowardTarget(2, box_start_z_, box_approach_speed_)) {
          setBoxStage(BoxStage::ApproachY, "BOX AUTO: approach EE Y");
        }
        break;
      case BoxStage::ApproachY:
        if (boxCommandAxisTowardTarget(1, box_start_y_, box_approach_speed_)) {
          setBoxStage(BoxStage::ApproachX, "BOX AUTO: approach EE X");
        }
        break;
      case BoxStage::ApproachX:
        if (boxCommandAxisTowardTarget(0, box_start_x_, box_approach_speed_)) {
          current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_VELOCITY;
          force_des_ = box_force_command_;
          publishPositionControl();
          setBoxStage(BoxStage::ContactSettle, "BOX AUTO: contact settling");
        }
        break;
      case BoxStage::ContactSettle:
        if (elapsed >= box_contact_settle_sec_) {
          if (!has_latest_fp_normal_ || !boxFeedbackFresh(last_fp_normal_time_, now)) {
            cancelBoxTrajectory("no fresh normal feedback after contact settling", true);
            break;
          }
          if (!boxContactFrameValid()) {
            cancelBoxTrajectory("invalid contact frame after contact settling", true);
            break;
          }
          box_z0_ = box_ee_z_;
          box_theta0_ = box_normal_azimuth_raw_;
          box_normal_azimuth_unwrapped_ = box_theta0_;
          box_last_normal_azimuth_raw_ = box_normal_azimuth_raw_;
          setBoxStage(BoxStage::T1Out, "BOX AUTO: segment 1, t1 angular feedback");
        }
        break;
      case BoxStage::T1Out:
        if (boxAngleReached(box_theta0_ + angle_delta, angle_delta)) {
          setBoxStage(BoxStage::T2Up, "BOX AUTO: segment 2, t2 height P control");
        } else if (box_t1_speed_ * elapsed > box_max_t1_travel_m_) {
          cancelBoxTrajectory("segment 1 maximum t1 travel exceeded", true);
        } else {
          cmd_xyz_yaw_[0] = 0.0;
          cmd_xyz_yaw_[1] = t1_command_sign * box_t1_speed_;
          cmd_xyz_yaw_[2] = 0.0;
        }
        break;
      case BoxStage::T2Up:
        updateBoxHeightStage(
          box_z0_ + box_height_m_, BoxStage::T1Back,
          "BOX AUTO: segment 3, reverse t1 angular feedback");
        break;
      case BoxStage::T1Back:
        if (boxAngleReached(box_theta0_, -angle_delta)) {
          setBoxStage(BoxStage::T2Down, "BOX AUTO: segment 4, t2 height P control");
        } else if (box_t1_speed_ * elapsed > box_max_t1_travel_m_) {
          cancelBoxTrajectory("segment 3 maximum t1 travel exceeded", true);
        } else {
          cmd_xyz_yaw_[0] = 0.0;
          cmd_xyz_yaw_[1] = -t1_command_sign * box_t1_speed_;
          cmd_xyz_yaw_[2] = 0.0;
        }
        break;
      case BoxStage::T2Down:
        updateBoxHeightStage(box_z0_, BoxStage::ReturnX, "BOX AUTO: contact off; return X");
        if (box_stage_ == BoxStage::ReturnX && box_trajectory_active_) {
          current_mode_ = crazyflie_interfaces::msg::PositionControl::MODE_POSITION;
          velocity_mode_command_ready_time_ = std::chrono::steady_clock::time_point{};
          publishPositionControl();
        }
        break;
      case BoxStage::ReturnX:
        if (boxCommandAxisTowardTarget(0, box_home_x_, box_return_speed_)) {
          setBoxStage(BoxStage::ReturnY, "BOX AUTO: return Y");
        }
        break;
      case BoxStage::ReturnY:
        if (boxCommandAxisTowardTarget(1, box_home_y_, box_return_speed_)) {
          setBoxStage(BoxStage::ReturnZ, "BOX AUTO: return Z");
        }
        break;
      case BoxStage::ReturnZ:
        if (boxCommandAxisTowardTarget(2, box_home_z_, box_return_speed_)) {
          box_trajectory_active_ = false;
          cmd_xyz_yaw_[0] = cmd_xyz_yaw_[1] = cmd_xyz_yaw_[2] = 0.0;
          publishKeyboardTrigger('p');
          status_msg_ = "BOX AUTO complete; returned home and DISARMED";
          pushInputHistory("b : AUTO complete -> home, p");
        }
        break;
    }
  }

  void publishPositionCmd()
  {
    if (!command_initialized_) {
      return;
    }

    if (usesVelocityCommands() &&
        velocity_mode_command_ready_time_.time_since_epoch().count() > 0 &&
        std::chrono::steady_clock::now() < velocity_mode_command_ready_time_) {
      return;
    }

    crazyflie_interfaces::msg::Position msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";
    msg.x = static_cast<float>(cmd_xyz_yaw_[0]);
    msg.y = static_cast<float>(cmd_xyz_yaw_[1]);
    msg.z = static_cast<float>(cmd_xyz_yaw_[2]);
    msg.yaw = static_cast<float>(cmd_xyz_yaw_[3]);
    cf_position_pub_->publish(msg);
  }

  std::array<double, 3> rotateOffsetByYawDeg(double yaw_deg) const
  {
    // Crazyflie yaw is positive in the opposite direction to ROS/world XY.
    const double yaw_rad = yaw_deg * M_PI / 180.0;
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);

    return {{
      c * end_effector_offset_[0] - s * end_effector_offset_[1],
      s * end_effector_offset_[0] + c * end_effector_offset_[1],
      end_effector_offset_[2]
    }};
  }

  std::array<double, 3> latestPoseInReferenceFrame(uint8_t commandReference) const
  {
    std::array<double, 3> pose{{
      latest_pose_xyz_yaw_[0],
      latest_pose_xyz_yaw_[1],
      latest_pose_xyz_yaw_[2]
    }};

    if (commandReference != crazyflie_interfaces::msg::PositionControl::REFERENCE_END_EFFECTOR) {
      return pose;
    }

    const auto rotated_offset = rotateOffsetByYawDeg(latest_pose_xyz_yaw_[3]);
    pose[0] += rotated_offset[0];
    pose[1] += rotated_offset[1];
    pose[2] += rotated_offset[2];
    return pose;
  }

  std::array<double, 3> latestPoseInActiveReferenceFrame() const
  {
    return latestPoseInReferenceFrame(current_command_reference_);
  }

  std::array<double, 3> latestFwCommandInActiveReferenceFrame() const
  {
    // flying_pen_normal_state directly logs suPosRef.eeCmdX/Y/Z, so these
    // values are already expressed at the end effector. Do not add rOff twice.
    return {{
      latest_fw_cmd_xyz_[0],
      latest_fw_cmd_xyz_[1],
      latest_fw_cmd_xyz_[2]
    }};
  }

  void publishPositionControl()
  {
    if (!position_control_pub_) {
      return;
    }

    crazyflie_interfaces::msg::PositionControl msg;
    msg.header.stamp = this->get_clock()->now();
    msg.position_mode = current_mode_;
    msg.command_reference = current_command_reference_;
    msg.trajectory_mode = crazyflie_interfaces::msg::PositionControl::TRAJECTORY_NONE;
    msg.force_desired = static_cast<float>(force_des_);
    position_control_pub_->publish(msg);
  }

  void updateForceStatus()
  {
    char buf[128];
    snprintf(buf, sizeof(buf), "set force_desired = %.3f (via PositionControl)", force_des_);
    status_msg_ = buf;
  }

  void publishKeyboardTrigger(char key)
  {
    if (!key_pub_) {
      status_msg_ = "key_pub not ready";
      return;
    }

    std_msgs::msg::String msg;
    msg.data = std::string(1, key);
    key_pub_->publish(msg);

    if (key == 'o') {
      status_msg_ = "published 'o' to keyboard_input (ARM)";
      pushInputHistory("o : ARM (keyboard_input)");
    } else if (key == 'p') {
      status_msg_ = "published 'p' to keyboard_input (DISARM)";
      pushInputHistory("p : DISARM (keyboard_input)");
    } else if (key == 'f') {
      status_msg_ = "published 'f' to keyboard_input (firmware hover calibration)";
      pushInputHistory("f : firmware hover calibration");
    } else {
      status_msg_ = "published keyboard trigger";
      pushInputHistory("keyboard trigger published");
    }
  }

  void statusCallback(const crazyflie_interfaces::msg::Status::SharedPtr msg)
  {
    if (msg) {
      latest_battery_voltage_ = msg->battery_voltage;
    }
  }

  void calibrationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg || msg->data.empty()) {
      return;
    }
    status_msg_ = msg->data;
    pushInputHistory(msg->data);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const auto & q = msg->pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw_deg = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;

    latest_pose_xyz_yaw_[0] = msg->pose.position.x;
    latest_pose_xyz_yaw_[1] = msg->pose.position.y;
    latest_pose_xyz_yaw_[2] = msg->pose.position.z;
    latest_pose_xyz_yaw_[3] = yaw_deg;
    has_latest_pose_ = true;

  }

  void fwCmdPositionCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    // flying_pen_normal_state layout:
    // [eePosX, eePosY, eePosZ, eeCmdX, eeCmdY, eeCmdZ]
    if (!msg || msg->values.size() < 6) {
      return;
    }

    latest_fw_cmd_xyz_[0] = msg->values[3];
    latest_fw_cmd_xyz_[1] = msg->values[4];
    latest_fw_cmd_xyz_[2] = msg->values[5];
    has_latest_fw_cmd_ = true;
    last_fw_cmd_time_ = std::chrono::steady_clock::now();
  }

  void fpNormalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // fp_normal is the normal-estimator CSV row without its first eight columns:
    // [eePos(0:2), eeCmd(3:5), vc(6:8), fcHat(9:11), fnHat(12),
    //  nHat(13:15), nDot(16:18), kappa(19), t1(20:22), t2(23:25), ...].
    if (!msg || msg->data.size() < 26) {
      return;
    }
    box_ee_z_ = msg->data[2];
    for (std::size_t i = 0; i < 3; ++i) {
      box_normal_[i] = msg->data[13 + i];
      box_t1_[i] = msg->data[20 + i];
      box_t2_[i] = msg->data[23 + i];
    }
    const double azimuth = std::atan2(box_normal_[1], box_normal_[0]);
    if (std::isfinite(azimuth)) {
      if (has_latest_fp_normal_ && std::isfinite(box_last_normal_azimuth_raw_)) {
        box_normal_azimuth_unwrapped_ +=
          std::remainder(azimuth - box_last_normal_azimuth_raw_, 2.0 * M_PI);
      } else {
        box_normal_azimuth_unwrapped_ = azimuth;
      }
      box_normal_azimuth_raw_ = azimuth;
      box_last_normal_azimuth_raw_ = azimuth;
    }
    has_latest_fp_normal_ = true;
    last_fp_normal_time_ = std::chrono::steady_clock::now();
  }

  void mobForceCallback(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
  {
    if (!msg || msg->values.size() < 3) {
      return;
    }

    mob_force_[0] = msg->values[0];
    mob_force_[1] = msg->values[1];
    mob_force_[2] = msg->values[2];
  }

  void updateDisplayedBatteryVoltage()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_battery_display_update_ < 1s) {
      return;
    }

    last_battery_display_update_ = now;
    displayed_battery_voltage_ = latest_battery_voltage_;
  }

  void pushInputHistory(const std::string & entry)
  {
    last_inputs_.push_front(entry);
    while (last_inputs_.size() > HISTORY_LEN) {
      last_inputs_.pop_back();
    }
    while (last_inputs_.size() < HISTORY_LEN) {
      last_inputs_.push_back("-");
    }
  }

  void drawSepLine(int row, const char * title)
  {
    move(row, 0);
    clrtoeol();
    printw("========================%s========================", title);
  }

  void drawLayout()
  {
    clear();

    drawSepLine(ROW_USAGE_HEADER, "usage");
    mvprintw(ROW_USAGE_1, 0, "velocity: w/s(x), a/d(y), e/q(z), z/c(yaw), x(zero vel), i(contact on)");
    mvprintw(ROW_USAGE_2, 0, "velocity: w/s/a/d/e/q(v), u(contact off)");
    mvprintw(ROW_USAGE_3, 0, "g/h/b: automatic trajectories, any other key: cancel, o/p: arm/disarm, t: quit");

    drawSepLine(ROW_STATUS_HEADER, "status");
    mvprintw(ROW_STATUS_MODE, 0, "mode: ");
    mvprintw(ROW_STATUS_FORCE, 0, "force command: ");
    mvprintw(ROW_STATUS_MOB, 0, "MOB force: ");
    mvprintw(ROW_STATUS_BATTERY, 0, "battery voltage: ");
    mvprintw(ROW_STATUS_MSG, 0, "status: ");

    drawSepLine(ROW_CMD_HEADER, "Position Command, Now");
    mvprintw(ROW_CMD_LINE1, 0, "cmd x/y/z = 0.000 / 0.000 / 0.000");
    mvprintw(ROW_CMD_LINE2, 0, "yaw = 0.0 deg");
    mvprintw(ROW_CMD_HIST_HDR, 0, "last inputs (recent 5):");
    for (size_t i = 0; i < HISTORY_LEN; ++i) {
      mvprintw(ROW_CMD_HIST_0 + static_cast<int>(i), 0, "  %zu) -", i + 1);
    }

    refresh();
  }

  void drawStatusBlock()
  {
    move(ROW_STATUS_MODE, 0);
    clrtoeol();
    printw(
      "mode: %s, frame: %s, pos_tick=(%.2f %.2f %.2f), vel_tick=(%.2f %.2f %.2f)",
      isContactFrameVelocityMode() ? "VEL_CONTACT" : "VEL_WORLD",
      command_frame_.c_str(),
      position_tick_[0], position_tick_[1], position_tick_[2],
      velocity_tick_[0], velocity_tick_[1], velocity_tick_[2]);

    move(ROW_STATUS_FORCE, 0);
    clrtoeol();
    printw("force command: %.3f", force_des_);

    move(ROW_STATUS_MOB, 0);
    clrtoeol();
    if (std::isfinite(mob_force_[0]) && std::isfinite(mob_force_[1]) && std::isfinite(mob_force_[2])) {
      const double mob_norm = std::sqrt(
        mob_force_[0] * mob_force_[0] +
        mob_force_[1] * mob_force_[1] +
        mob_force_[2] * mob_force_[2]);
      printw(
        "MOB force: x=%.3f, y=%.3f, z=%.3f [N], norm=%.3f",
        mob_force_[0], mob_force_[1], mob_force_[2], mob_norm);
    } else {
      printw("MOB force: waiting for cf2/cf_Fext_MOB");
    }

    move(ROW_STATUS_BATTERY, 0);
    clrtoeol();
    if (std::isfinite(displayed_battery_voltage_)) {
      printw("battery voltage: %.2f V (1 Hz)", displayed_battery_voltage_);
    } else {
      printw("battery voltage: waiting for cf2/status");
    }

    move(ROW_STATUS_MSG, 0);
    clrtoeol();
    printw("status: ");
    const bool calibration_done = status_msg_ == "[calibration done]";
    if (calibration_done && has_colors()) {
      attron(COLOR_PAIR(1) | A_BOLD);
    }
    printw("%s", status_msg_.c_str());
    if (calibration_done && has_colors()) {
      attroff(COLOR_PAIR(1) | A_BOLD);
    }

    refresh();
  }

  void drawCommandBlock()
  {
    move(ROW_CMD_LINE1, 0);
    clrtoeol();
    printw(
      "velocity cmd xyz = %.3f , %.3f , %.3f (reference=%s)",
      cmd_xyz_yaw_[0], cmd_xyz_yaw_[1], cmd_xyz_yaw_[2], command_frame_.c_str());

    move(ROW_CMD_LINE2, 0);
    clrtoeol();
    if (has_latest_pose_) {
      printw(
        "yaw = %.1f deg, latest pose = %.3f , %.3f , %.3f , yaw %.1f",
        cmd_xyz_yaw_[3],
        latest_pose_xyz_yaw_[0], latest_pose_xyz_yaw_[1], latest_pose_xyz_yaw_[2], latest_pose_xyz_yaw_[3]);
    } else {
      printw("yaw = %.1f deg, latest pose = waiting for cf2/pose", cmd_xyz_yaw_[3]);
    }

    mvprintw(ROW_CMD_HIST_HDR, 0, "last inputs (recent 5):");
    for (size_t i = 0; i < HISTORY_LEN; ++i) {
      move(ROW_CMD_HIST_0 + static_cast<int>(i), 0);
      clrtoeol();
      printw("  %zu) %s", i + 1, last_inputs_[i].c_str());
    }

    refresh();
  }

  rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cf_position_pub_;
  rclcpp::Publisher<crazyflie_interfaces::msg::PositionControl>::SharedPtr position_control_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr calibration_status_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr fw_cmd_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr mob_force_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fp_normal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 4> cmd_xyz_yaw_;
  std::array<double, 4> latest_pose_xyz_yaw_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> latest_fw_cmd_xyz_{{0.0, 0.0, 0.0}};
  std::array<double, 3> position_tick_;
  std::array<double, 3> velocity_tick_;
  std::array<double, 3> end_effector_offset_;
  std::array<double, 3> mob_force_;
  double yaw_tick_deg_;
  double force_des_;
  double force_delta_;
  double auto_approach_speed_;
  double auto_target_z_;
  double auto_target_y_;
  double auto_target_x_;
  double auto_position_tolerance_;
  double auto_t2_speed_;
  double auto_contact_settle_sec_;
  double auto_motion_sec_;
  double auto_second_down_extra_sec_;
  double auto_wait_sec_;
  double auto_fourth_wait_sec_;
  double auto_final_wait_sec_;
  double h_auto_approach_speed_;
  double h_auto_target_x_;
  double h_auto_target_y_;
  double h_auto_target_z_;
  double h_position_tolerance_;
  double h_force_command_;
  double h_contact_settle_sec_;
  double h_d_motion_sec_;
  double h_motion_speed_;
  double h_final_wait_sec_;
  double box_start_x_;
  double box_start_y_;
  double box_start_z_;
  double box_home_x_;
  double box_home_y_;
  double box_home_z_;
  double box_t1_angle_deg_;
  double box_height_m_;
  double box_t1_speed_;
  double box_t2_max_speed_;
  double box_t2_kp_;
  double box_z_tolerance_;
  double box_angle_tolerance_deg_;
  double box_t1_direction_sign_;
  double box_force_command_;
  double box_contact_settle_sec_;
  double box_approach_speed_;
  double box_return_speed_;
  double box_position_tolerance_;
  double box_stage_timeout_sec_;
  double box_feedback_timeout_sec_;
  double box_max_t1_travel_m_;
  double box_min_t2_vertical_projection_;
  double box_ee_z_{std::numeric_limits<double>::quiet_NaN()};
  double box_normal_azimuth_raw_{std::numeric_limits<double>::quiet_NaN()};
  double box_last_normal_azimuth_raw_{std::numeric_limits<double>::quiet_NaN()};
  double box_normal_azimuth_unwrapped_{0.0};
  double box_theta0_{0.0};
  double box_z0_{0.0};
  double latest_battery_voltage_;
  double displayed_battery_voltage_;
  uint8_t current_mode_;
  uint8_t current_command_reference_;
  bool has_latest_pose_;
  bool has_latest_fw_cmd_;
  bool has_latest_fp_normal_;
  bool command_initialized_{false};
  bool auto_trajectory_active_{false};
  AutoStage auto_stage_{AutoStage::ApproachZ};
  bool h_trajectory_active_{false};
  HStage h_stage_{HStage::ApproachZ};
  bool box_trajectory_active_{false};
  BoxStage box_stage_{BoxStage::ApproachZ};
  std::array<double, 3> box_normal_{{0.0, 0.0, 0.0}};
  std::array<double, 3> box_t1_{{0.0, 0.0, 0.0}};
  std::array<double, 3> box_t2_{{0.0, 0.0, 0.0}};
  std::array<double, 2> h_reference_xy_{{0.0, 0.0}};
  int h_reference_crossings_{0};
  bool h_inside_reference_tolerance_{true};
  std::string command_frame_;
  std::string status_msg_;
  std::chrono::steady_clock::time_point last_battery_display_update_;
  std::chrono::steady_clock::time_point velocity_mode_command_ready_time_{};
  std::chrono::steady_clock::time_point auto_stage_start_time_{};
  std::chrono::steady_clock::time_point h_stage_start_time_{};
  std::chrono::steady_clock::time_point box_stage_start_time_{};
  std::chrono::steady_clock::time_point last_fw_cmd_time_{};
  std::chrono::steady_clock::time_point last_fp_normal_time_{};
  std::deque<std::string> last_inputs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  bool has_params_file = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--params-file") {
      has_params_file = true;
      break;
    }
  }

  if (!has_params_file) {
    const auto config_path =
      ament_index_cpp::get_package_share_directory("flying_pen") + "/config/command.yaml";
    options.arguments({"--ros-args", "--params-file", config_path});
  }

  auto node = std::make_shared<CommandPublisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
