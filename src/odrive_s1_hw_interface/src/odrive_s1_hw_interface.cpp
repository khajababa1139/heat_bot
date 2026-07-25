#include "odrive_s1_hw_interface/odrive_s1_hw_interface.hpp"

#include <algorithm>
#include <cstring>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace odrive_s1_hw_interface
{

hardware_interface::CallbackReturn ODriveS1SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- hardware-level parameters (from the <hardware> block in the xacro) ----
  if (info.hardware_parameters.count("can_interface")) {
    can_interface_ = info.hardware_parameters.at("can_interface");
  }
  if (info.hardware_parameters.count("can_timeout_ms")) {
    can_timeout_ms_ = std::stoi(info.hardware_parameters.at("can_timeout_ms"));
  }
  if (info.hardware_parameters.count("max_velocity_turns_per_sec")) {
    max_velocity_turns_per_sec_ =
      std::stod(info.hardware_parameters.at("max_velocity_turns_per_sec"));
  }

  wheels_.resize(info.joints.size());

  for (size_t i = 0; i < info.joints.size(); ++i) {
    const auto & joint = info.joints[i];
    auto & wheel = wheels_[i];
    wheel.joint_name = joint.name;

    if (joint.parameters.count("node_id") == 0) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' is missing the required 'node_id' parameter",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    wheel.node_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("node_id")));

    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must expose exactly one 'velocity' command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must expose 'position' and 'velocity' state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    get_logger(), "Configured %zu wheel joint(s) on CAN interface '%s' (timeout %d ms, cap %.2f turns/s)",
    wheels_.size(), can_interface_.c_str(), can_timeout_ms_, max_velocity_turns_per_sec_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveS1SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & wheel : wheels_) {
    state_interfaces.emplace_back(
      wheel.joint_name, hardware_interface::HW_IF_POSITION, &wheel.position);
    state_interfaces.emplace_back(
      wheel.joint_name, hardware_interface::HW_IF_VELOCITY, &wheel.velocity);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ODriveS1SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & wheel : wheels_) {
    command_interfaces.emplace_back(
      wheel.joint_name, hardware_interface::HW_IF_VELOCITY, &wheel.velocity_command);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ODriveS1SystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!can_bus_.open(can_interface_)) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to open SocketCAN interface '%s'. Bring it up first, e.g.: "
      "sudo ip link set %s up type can bitrate 250000",
      can_interface_.c_str(), can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & wheel : wheels_) {
    wheel.feedback_seen = false;
    wheel.position = 0.0;
    wheel.velocity = 0.0;
    wheel.velocity_command = 0.0;
  }

  RCLCPP_INFO(get_logger(), "SocketCAN interface '%s' opened", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveS1SystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  can_bus_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveS1SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Clear stale faults first, then request closed-loop velocity control on every axis.
  for (auto & wheel : wheels_) {
    send_clear_errors(wheel.node_id);
  }

  for (auto & wheel : wheels_) {
    send_controller_mode(wheel.node_id, CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
    send_axis_state(wheel.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  // Give every axis a chance to report CLOSED_LOOP_CONTROL back over its heartbeat
  // before declaring activation successful - this is the same closed-loop check
  // sample_can.py's `closed` command does interactively, just automated here.
  bool all_ok = true;
  for (auto & wheel : wheels_) {
    if (!wait_for_axis_state(
        wheel.node_id, AXIS_STATE_CLOSED_LOOP_CONTROL, std::chrono::milliseconds(1000)))
    {
      RCLCPP_ERROR(
        get_logger(), "Node %u ('%s') did not confirm CLOSED_LOOP_CONTROL within 1 s",
        wheel.node_id, wheel.joint_name.c_str());
      all_ok = false;
    }
  }

  if (!all_ok) {
    // Best-effort: walk back any axis we did manage to arm before failing activation.
    for (auto & wheel : wheels_) {
      send_axis_state(wheel.node_id, AXIS_STATE_IDLE);
    }
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "All %zu ODrive axis(es) confirmed CLOSED_LOOP_CONTROL", wheels_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveS1SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Zero velocity before dropping to IDLE, so nothing lurches on the way down.
  for (auto & wheel : wheels_) {
    send_input_vel(wheel.node_id, 0.0f);
  }
  for (auto & wheel : wheels_) {
    send_axis_state(wheel.node_id, AXIS_STATE_IDLE);
  }
  RCLCPP_INFO(get_logger(), "All ODrive axes returned to IDLE");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ODriveS1SystemHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Drain whatever is currently sitting in the socket's receive buffer without
  // blocking the control loop. The cap keeps this bounded even if the bus is
  // unusually busy; in normal operation there are only a handful of frames
  // (4 heartbeats + 4 encoder-estimate messages) per cycle.
  constexpr int kMaxFramesPerCycle = 64;
  for (int i = 0; i < kMaxFramesPerCycle; ++i) {
    auto frame = can_bus_.receive(0);
    if (!frame) {
      break;
    }
    process_frame(*frame, time);
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  for (auto & wheel : wheels_) {
    if (!wheel.feedback_seen) {
      continue;  // no feedback yet at all - keep the zeroed initial state
    }

    const double age = (time - wheel.last_feedback_time).seconds();
    const double stale_after = (can_timeout_ms_ / 1000.0) * 5.0;
    if (age > stale_after) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), clock_, 1000, "No CAN feedback from node %u ('%s') for %.2f s",
        wheel.node_id, wheel.joint_name.c_str(), age);
      continue;  // hold last known good value rather than snapping to zero
    }

    wheel.position = static_cast<double>(wheel.raw_pos_turns) * kTurnsToRad;
    wheel.velocity = static_cast<double>(wheel.raw_vel_turns_s) * kTurnsToRad;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ODriveS1SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & wheel : wheels_) {
    double turns_s = wheel.velocity_command * kRadToTurns;
    turns_s = std::clamp(turns_s, -max_velocity_turns_per_sec_, max_velocity_turns_per_sec_);
    send_input_vel(wheel.node_id, static_cast<float>(turns_s));
  }
  return hardware_interface::return_type::OK;
}

void ODriveS1SystemHardware::process_frame(const can_frame & frame, const rclcpp::Time & now)
{
  const uint8_t node_id = static_cast<uint8_t>(frame.can_id >> 5);
  const uint32_t cmd_id = frame.can_id & 0x1F;

  auto it = std::find_if(
    wheels_.begin(), wheels_.end(),
    [node_id](const WheelContext & w) {return w.node_id == node_id;});
  if (it == wheels_.end()) {
    return;  // frame from a node we don't own - ignore
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  if (cmd_id == CMD_HEARTBEAT && frame.can_dlc >= 5) {
    uint32_t active_errors = 0;
    std::memcpy(&active_errors, frame.data, sizeof(active_errors));
    it->active_errors = active_errors;
    it->axis_state = frame.data[4];
    it->last_feedback_time = now;
    it->feedback_seen = true;
  } else if (cmd_id == CMD_GET_ENCODER_ESTIMATES && frame.can_dlc >= 8) {
    float pos = 0.0f;
    float vel = 0.0f;
    std::memcpy(&pos, frame.data, sizeof(pos));
    std::memcpy(&vel, frame.data + 4, sizeof(vel));
    it->raw_pos_turns = pos;
    it->raw_vel_turns_s = vel;
    it->last_feedback_time = now;
    it->feedback_seen = true;
  }
}

bool ODriveS1SystemHardware::wait_for_axis_state(
  uint8_t node_id, uint32_t desired_state, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    auto frame = can_bus_.receive(20);
    if (frame) {
      process_frame(*frame, rclcpp::Clock(RCL_STEADY_TIME).now());
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = std::find_if(
      wheels_.begin(), wheels_.end(),
      [node_id](const WheelContext & w) {return w.node_id == node_id;});
    if (it != wheels_.end() && it->feedback_seen && it->axis_state == desired_state) {
      return true;
    }
  }
  return false;
}

void ODriveS1SystemHardware::send_axis_state(uint8_t node_id, uint32_t state)
{
  can_frame frame{};
  frame.can_id = (static_cast<uint32_t>(node_id) << 5) | CMD_SET_AXIS_STATE;
  frame.can_dlc = 4;
  std::memcpy(frame.data, &state, sizeof(state));
  can_bus_.send(frame);
}

void ODriveS1SystemHardware::send_controller_mode(
  uint8_t node_id, uint32_t control_mode, uint32_t input_mode)
{
  can_frame frame{};
  frame.can_id = (static_cast<uint32_t>(node_id) << 5) | CMD_SET_CONTROLLER_MODE;
  frame.can_dlc = 8;
  std::memcpy(frame.data, &control_mode, sizeof(control_mode));
  std::memcpy(frame.data + 4, &input_mode, sizeof(input_mode));
  can_bus_.send(frame);
}

void ODriveS1SystemHardware::send_input_vel(uint8_t node_id, float vel_turns_s, float torque_ff)
{
  can_frame frame{};
  frame.can_id = (static_cast<uint32_t>(node_id) << 5) | CMD_SET_INPUT_VEL;
  frame.can_dlc = 8;
  std::memcpy(frame.data, &vel_turns_s, sizeof(vel_turns_s));
  std::memcpy(frame.data + 4, &torque_ff, sizeof(torque_ff));
  can_bus_.send(frame);
}

void ODriveS1SystemHardware::send_clear_errors(uint8_t node_id)
{
  can_frame frame{};
  frame.can_id = (static_cast<uint32_t>(node_id) << 5) | CMD_CLEAR_ERRORS;
  frame.can_dlc = 1;
  frame.data[0] = 0x00;
  can_bus_.send(frame);
}

}  // namespace odrive_s1_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  odrive_s1_hw_interface::ODriveS1SystemHardware, hardware_interface::SystemInterface)
