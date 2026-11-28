// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Data types of the fault-management core.
//
// Vocabulary:
//   report   a raw observation from a monitor: "condition X is failing / is healthy"
//   fault    the debounced, tracked result: something an operator should see
//   event    a change of a fault (activated, cleared, acknowledged), kept in history

#ifndef FAULT_CORE__TYPES_HPP_
#define FAULT_CORE__TYPES_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace fault_core
{

/// Monotonic time for all timing decisions (debounce, watchdogs). Passed in by
/// the caller so that behaviour is deterministic under test.
using TimePoint = std::chrono::steady_clock::time_point;
using Duration = std::chrono::milliseconds;

/// Wall-clock time, used only to stamp events for humans and logs.
using WallTime = std::chrono::system_clock::time_point;

/// Ordered: a higher value is more severe.
enum class Severity {Info = 0, Warning = 1, Error = 2, Critical = 3};

enum class FaultState
{
  Inactive,     ///< Nothing wrong.
  Pending,      ///< Failing, but not yet for long enough to count (debouncing).
  Active,       ///< Confirmed fault.
  Healing,      ///< Was active, now healthy, but not yet for long enough to clear.
  AwaitingAck,  ///< Latched fault whose cause is gone; stays until acknowledged.
};

/// Static description of one fault, from the catalogue.
struct FaultDefinition
{
  std::string code;         ///< Stable identifier, e.g. "NAV-001".
  Severity severity{Severity::Error};
  std::string description;

  /// The condition must fail for this long before the fault activates.
  /// Filters out glitches. Zero activates on the first failing report.
  Duration activate_after{0};

  /// The condition must be healthy for this long before the fault clears.
  /// Stops a flickering condition from producing a storm of events.
  Duration clear_after{0};

  /// A latched fault does not clear by itself: once its cause is gone it waits
  /// for an operator to acknowledge it. For faults that must not be missed.
  bool latched{false};

  /// Watchdog: if no report at all arrives for this long, the condition counts
  /// as failing. Zero disables. Catches a monitor that has died silently.
  Duration timeout{0};
};

/// Current status of one fault.
struct FaultRecord
{
  std::string code;
  FaultState state{FaultState::Inactive};
  Severity severity{Severity::Error};
  std::string description;
  std::string message;  ///< Detail from the latest failing report.
  std::string source;   ///< Who reported it.
  bool known{true};     ///< false: the code was not in the catalogue.
  std::uint64_t occurrence_count{0};  ///< How many times it has activated.
  WallTime first_activated{};
  WallTime last_activated{};
  WallTime last_changed{};
};

enum class EventType {Activated, Cleared, Acknowledged};

/// One entry of the fault history.
struct FaultEvent
{
  std::uint64_t sequence{0};  ///< Strictly increasing; lets a client ask "what is new since N".
  EventType type{EventType::Activated};
  std::string code;
  Severity severity{Severity::Error};
  WallTime time{};
  std::string message;
  std::string source;
  std::uint64_t occurrence_count{0};
};

/// True for the states in which a fault should be shown to an operator.
inline bool is_raised(FaultState state)
{
  return state == FaultState::Active || state == FaultState::Healing ||
         state == FaultState::AwaitingAck;
}

}  // namespace fault_core

#endif  // FAULT_CORE__TYPES_HPP_
