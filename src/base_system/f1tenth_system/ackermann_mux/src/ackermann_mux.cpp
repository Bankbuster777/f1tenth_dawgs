// Copyright 2020 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 * @author Jeremie Deray
 * @author Hongrui Zheng
 */

#include <ackermann_mux/ackermann_mux.hpp>
#include <ackermann_mux/topic_handle.hpp>
#include <ackermann_mux/ackermann_mux_diagnostics.hpp>
#include <ackermann_mux/ackermann_mux_diagnostics_status.hpp>
#include <ackermann_mux/utils.hpp>
#include <ackermann_mux/params_helpers.hpp>

#include <list>
#include <memory>
#include <string>

/**
 * @brief hasIncreasedAbsVelocity Check if the absolute velocity has increased
 * in any of the components: linear (abs(x)) or angular (abs(yaw))
 * @param old_drive Old velocity
 * @param new_drive New velocity
 * @return true is any of the absolute velocity components has increased
 */
bool hasIncreasedAbsVelocity(
  const ackermann_msgs::msg::AckermannDriveStamped & old_drive,
  const ackermann_msgs::msg::AckermannDriveStamped & new_drive)
{
  const auto old_linear_x = std::abs(old_drive.drive.speed);
  const auto new_linear_x = std::abs(new_drive.drive.speed);

  // const auto old_angular_z = std::abs(old_drive.drive.angular.z);
  // const auto new_angular_z = std::abs(new_drive.drive.angular.z);

  // return (old_linear_x < new_linear_x) || (old_angular_z < new_angular_z);
  return (old_linear_x < new_linear_x);
}

namespace ackermann_mux
{
// see e.g. https://stackoverflow.com/a/40691657
constexpr std::chrono::duration<int64_t> AckermannMux::DIAGNOSTICS_PERIOD;

AckermannMux::AckermannMux()
: Node("ackermann_mux", "",
    rclcpp::NodeOptions().allow_undeclared_parameters(
      true).automatically_declare_parameters_from_overrides(true))
{
}

void AckermannMux::init()
{
  /// Get topics and locks:
  velocity_hs_ = std::make_shared<velocity_topic_container>();
  lock_hs_ = std::make_shared<lock_topic_container>();
  getTopicHandles("topics", *velocity_hs_);
  getTopicHandles("locks", *lock_hs_);

  /// Publisher for output topic with real-time QoS:
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();  // Best effort for minimal latency

  cmd_pub_ =
    this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann_cmd",
    qos);

  /// Brake handling parameters:
  // Declare parameters only if not already declared (to avoid conflicts with YAML files)
  if (!this->has_parameter("enable_brake")) {
    this->declare_parameter("enable_brake", false);
  }
  if (!this->has_parameter("deceleration_threshold")) {
    this->declare_parameter("deceleration_threshold", 0.5);
  }
  if (!this->has_parameter("brake_gain")) {
    this->declare_parameter("brake_gain", 1000.0);
  }
  if (!this->has_parameter("max_brake_current")) {
    this->declare_parameter("max_brake_current", 20000.0);
  }

  enable_brake_ = this->get_parameter("enable_brake").as_bool();
  deceleration_threshold_ = this->get_parameter("deceleration_threshold").as_double();
  brake_gain_ = this->get_parameter("brake_gain").as_double();
  max_brake_current_ = this->get_parameter("max_brake_current").as_double();

  /// Brake publisher (only create if brake is enabled):
  if (enable_brake_) {
    brake_pub_ =
      this->create_publisher<std_msgs::msg::Float64>(
      "commands/motor/brake",
      qos);

    RCLCPP_INFO(
      get_logger(),
      "Brake ENABLED - parameters: threshold=%.2f m/s, gain=%.0f A/(m/s), max=%.0f A",
      deceleration_threshold_, brake_gain_, max_brake_current_);
  } else {
    RCLCPP_INFO(get_logger(), "Brake DISABLED - no brake commands will be published");
  }

  /// Initialize last command with zero speed:
  last_cmd_.drive.speed = 0.0;

  /// Diagnostics:
  diagnostics_ = std::make_shared<diagnostics_type>(this);
  status_ = std::make_shared<status_type>();
  status_->velocity_hs = velocity_hs_;
  status_->lock_hs = lock_hs_;

  diagnostics_timer_ = this->create_wall_timer(
    DIAGNOSTICS_PERIOD, [this]() -> void {
      updateDiagnostics();
    });
}

void AckermannMux::updateDiagnostics()
{
  status_->priority = getLockPriority();
  diagnostics_->updateStatus(status_);
}

void AckermannMux::publishAckermann(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr & msg)
{
  if (enable_brake_ && brake_pub_) {
    // Brake enabled: Use speed/brake switching logic

    // Calculate speed difference from last command
    double current_speed = msg->drive.speed;
    double last_speed = last_cmd_.drive.speed;
    double speed_diff = current_speed - last_speed;

    // Determine if decelerating
    bool is_decelerating = (speed_diff < -deceleration_threshold_);

    if (is_decelerating) {
      // Decelerating: publish ONLY brake, set speed to 0
      double brake_current = std::abs(speed_diff) * brake_gain_;
      brake_current = std::min(brake_current, max_brake_current_);

      // Publish brake command
      auto brake_msg = std_msgs::msg::Float64();
      brake_msg.data = brake_current;
      brake_pub_->publish(brake_msg);

      // Publish ackermann command with speed=0 (steering is maintained)
      auto modified_msg = *msg;
      modified_msg.drive.speed = 0.0;
      cmd_pub_->publish(modified_msg);

      RCLCPP_DEBUG(
        get_logger(),
        "Deceleration: brake=%.0f A, speed=0.0 (original: %.2f m/s -> %.2f m/s, diff: %.2f)",
        brake_current, last_speed, current_speed, speed_diff);

    } else {
      // Accelerating or maintaining: publish ONLY speed, release brake explicitly

      // Release brake (set to 0)
      auto brake_msg = std_msgs::msg::Float64();
      brake_msg.data = 0.0;
      brake_pub_->publish(brake_msg);

      // Publish normal ackermann command
      cmd_pub_->publish(*msg);

      RCLCPP_DEBUG(
        get_logger(),
        "Acceleration/Maintaining: speed=%.2f m/s (diff: %.2f), brake released",
        current_speed, speed_diff);
    }

  } else {
    // Brake disabled: publish ONLY speed commands, never publish brake
    cmd_pub_->publish(*msg);

    RCLCPP_DEBUG(
      get_logger(),
      "Brake disabled: speed=%.2f m/s (no brake control)",
      msg->drive.speed);
  }

  // Store current command for next comparison
  last_cmd_ = *msg;
}

template<typename T>
void AckermannMux::getTopicHandles(const std::string & param_name, std::list<T> & topic_hs)
{
  RCLCPP_DEBUG(get_logger(), "getTopicHandles: %s", param_name.c_str());

  rcl_interfaces::msg::ListParametersResult list = list_parameters({param_name}, 10);

  try {
    for (auto prefix : list.prefixes) {
      RCLCPP_DEBUG(get_logger(), "Prefix: %s", prefix.c_str());

      std::string topic;
      double timeout = 0;
      int priority = 0;

      auto nh = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

      fetch_param(nh, prefix + ".topic", topic);
      fetch_param(nh, prefix + ".timeout", timeout);
      fetch_param(nh, prefix + ".priority", priority);

      RCLCPP_DEBUG(get_logger(), "Retrieved topic: %s", topic.c_str());
      RCLCPP_DEBUG(get_logger(), "Listed prefix: %.2f", timeout);
      RCLCPP_DEBUG(get_logger(), "Listed prefix: %d", priority);

      topic_hs.emplace_back(prefix, topic, std::chrono::duration<double>(timeout), priority, this);
    }
  } catch (const ParamsHelperException & e) {
    RCLCPP_FATAL(get_logger(), "Error parsing params '%s':\n\t%s", param_name.c_str(), e.what());
    throw e;
  }
}

int AckermannMux::getLockPriority()
{
  LockTopicHandle::priority_type priority = 0;

  /// max_element on the priority of lock topic handles satisfying
  /// that is locked:
  for (const auto & lock_h : *lock_hs_) {
    if (lock_h.isLocked()) {
      auto tmp = lock_h.getPriority();
      if (priority < tmp) {
        priority = tmp;
      }
    }
  }

  RCLCPP_DEBUG(get_logger(), "Priority = %d.", static_cast<int>(priority));

  return priority;
}

bool AckermannMux::hasPriority(const VelocityTopicHandle & ackermann)
{
  const auto lock_priority = getLockPriority();

  LockTopicHandle::priority_type priority = 0;
  std::string velocity_name = "NULL";

  /// max_element on the priority of velocity topic handles satisfying
  /// that is NOT masked by the lock priority:
  for (const auto & velocity_h : *velocity_hs_) {
    if (!velocity_h.isMasked(lock_priority)) {
      const auto velocity_priority = velocity_h.getPriority();
      if (priority < velocity_priority) {
        priority = velocity_priority;
        velocity_name = velocity_h.getName();
      }
    }
  }

  return ackermann.getName() == velocity_name;
}

}  // namespace ackermann_mux
