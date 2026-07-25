#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "odrive_s1_hw_interface/socketcan_bus.hpp"

namespace odrive_s1_hw_interface
{

/// Per-wheel bookkeeping: which CAN node this joint talks to, the state/command
/// values exposed to ros2_control, and the raw feedback most recently parsed off
/// the bus for that node.
struct WheelContext
{
  std::string joint_name;
  uint8_t node_id = 0;

  // Exposed to ros2_control (rad, rad/s) - written by read(), read by controllers.
  double position = 0.0;
  double velocity = 0.0;
  // Written by controllers via the command interface, read by write().
  double velocity_command = 0.0;

  // Raw ODrive feedback (turns, turns/s) and health, guarded by the owning
  // class's state_mutex_.
  float raw_pos_turns = 0.0f;
  float raw_vel_turns_s = 0.0f;
  uint8_t axis_state = 0;
  uint32_t active_errors = 0;
  rclcpp::Time last_feedback_time;
  bool feedback_seen = false;
};

class ODriveS1SystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ODrive CAN Simple protocol command IDs (arbitration_id = (node_id << 5) | cmd_id).
  // Same IDs as sample_can.py, kept as named constants here.
  static constexpr uint32_t CMD_HEARTBEAT = 0x001;
  static constexpr uint32_t CMD_SET_AXIS_STATE = 0x007;
  static constexpr uint32_t CMD_GET_ENCODER_ESTIMATES = 0x009;
  static constexpr uint32_t CMD_SET_CONTROLLER_MODE = 0x00B;
  static constexpr uint32_t CMD_SET_INPUT_VEL = 0x00D;
  static constexpr uint32_t CMD_CLEAR_ERRORS = 0x018;

  static constexpr uint32_t AXIS_STATE_IDLE = 1;
  static constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
  static constexpr uint32_t CONTROL_MODE_VELOCITY_CONTROL = 2;
  static constexpr uint32_t INPUT_MODE_PASSTHROUGH = 1;

  void send_axis_state(uint8_t node_id, uint32_t state);
  void send_controller_mode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode);
  void send_input_vel(uint8_t node_id, float vel_turns_s, float torque_ff = 0.0f);
  void send_clear_errors(uint8_t node_id);

  /// Parses one incoming frame and, if it belongs to one of our wheels, updates
  /// that wheel's cached feedback under state_mutex_.
  void process_frame(const can_frame & frame, const rclcpp::Time & now);

  /// Polls the bus (blocking, with an overall timeout) until the given node's
  /// last-seen heartbeat reports desired_state. Only used during on_activate(),
  /// never from the real-time read()/write() cycle.
  bool wait_for_axis_state(
    uint8_t node_id, uint32_t desired_state, std::chrono::milliseconds timeout);

  rclcpp::Logger get_logger() const {return rclcpp::get_logger("ODriveS1SystemHardware");}

  std::vector<WheelContext> wheels_;
  SocketCanBus can_bus_;

  std::string can_interface_ = "can0";
  int can_timeout_ms_ = 100;
  double max_velocity_turns_per_sec_ = 5.0;

  std::mutex state_mutex_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  // Written as a literal rather than via M_PI: M_PI is a glibc/POSIX extension
  // that isn't guaranteed visible under a strict -std=c++17 (vs. gnu++17) build.
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kTurnsToRad = 2.0 * kPi;
  static constexpr double kRadToTurns = 1.0 / kTurnsToRad;
};

}  // namespace odrive_s1_hw_interface
