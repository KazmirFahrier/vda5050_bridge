// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Conversions between fault_core types and fault_msgs messages.

#ifndef FAULT_MANAGER__CONVERSIONS_HPP_
#define FAULT_MANAGER__CONVERSIONS_HPP_

#include <chrono>
#include <cstdint>

#include "builtin_interfaces/msg/time.hpp"
#include "fault_core/types.hpp"
#include "fault_msgs/msg/fault.hpp"
#include "fault_msgs/msg/fault_event.hpp"

namespace fault_manager
{

// The numeric values of the message constants are those of the core enums.
// These checks make a change on either side a compile error, not a silent bug.
static_assert(
  fault_msgs::msg::Fault::SEVERITY_INFO == static_cast<int>(fault_core::Severity::Info) &&
  fault_msgs::msg::Fault::SEVERITY_WARNING == static_cast<int>(fault_core::Severity::Warning) &&
  fault_msgs::msg::Fault::SEVERITY_ERROR == static_cast<int>(fault_core::Severity::Error) &&
  fault_msgs::msg::Fault::SEVERITY_CRITICAL == static_cast<int>(fault_core::Severity::Critical),
  "severity constants out of step");
static_assert(
  fault_msgs::msg::Fault::STATE_INACTIVE == static_cast<int>(fault_core::FaultState::Inactive) &&
  fault_msgs::msg::Fault::STATE_PENDING == static_cast<int>(fault_core::FaultState::Pending) &&
  fault_msgs::msg::Fault::STATE_ACTIVE == static_cast<int>(fault_core::FaultState::Active) &&
  fault_msgs::msg::Fault::STATE_HEALING == static_cast<int>(fault_core::FaultState::Healing) &&
  fault_msgs::msg::Fault::STATE_AWAITING_ACK ==
  static_cast<int>(fault_core::FaultState::AwaitingAck),
  "state constants out of step");
static_assert(
  fault_msgs::msg::FaultEvent::TYPE_ACTIVATED ==
  static_cast<int>(fault_core::EventType::Activated) &&
  fault_msgs::msg::FaultEvent::TYPE_CLEARED == static_cast<int>(fault_core::EventType::Cleared) &&
  fault_msgs::msg::FaultEvent::TYPE_ACKNOWLEDGED ==
  static_cast<int>(fault_core::EventType::Acknowledged),
  "event type constants out of step");

inline builtin_interfaces::msg::Time to_msg(fault_core::WallTime time)
{
  const std::int64_t nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
  builtin_interfaces::msg::Time message;
  message.sec = static_cast<std::int32_t>(nanoseconds / 1000000000);
  message.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000);
  return message;
}

inline fault_msgs::msg::Fault to_msg(const fault_core::FaultRecord & record)
{
  fault_msgs::msg::Fault message;
  message.code = record.code;
  message.severity = static_cast<std::uint8_t>(record.severity);
  message.state = static_cast<std::uint8_t>(record.state);
  message.description = record.description;
  message.message = record.message;
  message.source = record.source;
  message.known = record.known;
  message.occurrence_count = record.occurrence_count;
  if (record.occurrence_count > 0) {
    message.first_activated = to_msg(record.first_activated);
    message.last_activated = to_msg(record.last_activated);
  }
  return message;
}

inline fault_msgs::msg::FaultEvent to_msg(const fault_core::FaultEvent & event)
{
  fault_msgs::msg::FaultEvent message;
  message.sequence = event.sequence;
  message.type = static_cast<std::uint8_t>(event.type);
  message.code = event.code;
  message.severity = static_cast<std::uint8_t>(event.severity);
  message.stamp = to_msg(event.time);
  message.message = event.message;
  message.source = event.source;
  message.occurrence_count = event.occurrence_count;
  return message;
}

}  // namespace fault_manager

#endif  // FAULT_MANAGER__CONVERSIONS_HPP_
