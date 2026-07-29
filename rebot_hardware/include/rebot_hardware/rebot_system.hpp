// Copyright 2026 reBot ros2_control contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REBOT_HARDWARE__REBOT_SYSTEM_HPP_
#define REBOT_HARDWARE__REBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

#include "rebot_hardware/damiao_motor_driver.hpp"
#include "rebot_hardware/serial_can_bridge.hpp"

namespace rebot_hardware
{

/// Name of the custom MIT-gain command interfaces (next to the standard
/// position/velocity/effort ones).
constexpr char HW_IF_KP[] = "kp";
constexpr char HW_IF_KD[] = "kd";

/// ros2_control SystemInterface for the Seeed reBot Arm B601-DM
/// (Damiao DM-J4310 / DM-J4340P motors behind a USB-CAN serial bridge).
///
/// Command interfaces per joint (the full Damiao MIT tuple). A NaN command
/// means "not set" and falls back to a safe default in write():
///   position -> hold current measured position
///   velocity -> 0
///   kp / kd  -> URDF <param name="kp"/"kd"> defaults
///   effort   -> 0
/// Controllers that claim these interfaces must write NaN back on
/// deactivation so that controller switching is torque-jump free.
///
/// All hardware parameters come from the URDF <ros2_control> tag:
///  hardware <param>: serial_port, baud_rate, comm_error_threshold,
///                    activate_timeout_ms, enable_retries
///  per-joint <param>: motor_id, feedback_id, motor_model, kp, kd,
///                     mechanical_reduction, position_offset,
///                     min_position, max_position
class ReBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  ~ReBotSystemHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Static per-joint configuration parsed from the URDF.
  struct JointConfig
  {
    uint32_t motor_id{0};
    uint32_t feedback_id{0};
    damiao::MotorLimits limits{12.5, 30.0, 10.0};
    double kp{0.0};
    double kd{0.0};
    /// motor_position = joint_position * reduction + offset
    double reduction{1.0};
    double offset{0.0};
    /// Software position limits in joint space, clamped in write().
    double min_position{0.0};
    double max_position{0.0};
  };

  /// Best-effort disable of every motor (activation, error and shutdown
  /// paths). Never throws.
  void disable_all_motors();

  /// Update joint states from one decoded feedback frame. Returns the joint
  /// index or -1 when the frame does not belong to a configured motor.
  int apply_feedback(const damiao::CanFrame & frame);

  std::vector<JointConfig> joints_;

  // Joint-space commands and states (SI units). NaN command = "not set".
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_kp_;
  std::vector<double> hw_commands_kd_;
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;

  // Consecutive write cycles without feedback, per motor.
  std::vector<unsigned int> missed_replies_;
  unsigned int comm_error_threshold_{10};

  // Throttling counter for hot-path error logs.
  unsigned int error_log_counter_{0};

  std::string serial_port_;
  int baud_rate_{SerialCanBridge::kDefaultBaudRate};
  int activate_timeout_ms_{100};
  int enable_retries_{5};

  std::unique_ptr<SerialCanBridge> bridge_;
  rclcpp::Logger logger_{rclcpp::get_logger("ReBotSystemHardware")};
};

}  // namespace rebot_hardware

#endif  // REBOT_HARDWARE__REBOT_SYSTEM_HPP_
