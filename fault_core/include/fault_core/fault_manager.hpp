// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// FaultManager: turns raw health reports into tracked faults.

#ifndef FAULT_CORE__FAULT_MANAGER_HPP_
#define FAULT_CORE__FAULT_MANAGER_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "fault_core/catalog.hpp"
#include "fault_core/types.hpp"

namespace fault_core
{

struct FaultManagerConfig
{
  /// How many events the in-memory history keeps. The oldest are dropped first.
  std::size_t history_capacity{1000};

  /// A report for a code that is not in the catalogue is still tracked (losing
  /// a fault because of a typo in a catalogue would be worse), with this severity
  /// and no debounce, and its record is marked `known = false`.
  bool accept_unknown_codes{true};
  Severity unknown_code_severity{Severity::Error};
};

/// Selects part of the history. Unset fields do not filter.
struct HistoryQuery
{
  std::optional<std::string> code;
  std::optional<Severity> min_severity;
  std::uint64_t after_sequence{0};  ///< Only events with a higher sequence number.
  std::size_t limit{0};             ///< Keep only the newest `limit` matches; 0 = all.
};

struct Summary
{
  std::size_t raised{0};                 ///< Faults currently shown to an operator.
  std::optional<Severity> highest;       ///< Most severe raised fault, if any.
  std::size_t by_severity[4]{0, 0, 0, 0};  ///< Raised faults per Severity value.
};

/// Not thread-safe and never blocks: the owner calls it from one thread and
/// passes the current time in. Same contract, for the same reasons, as
/// vda5050_core::OrderManager.
///
/// Lifecycle of a fault:
///
///   Inactive --failing--> Pending --still failing after activate_after--> Active
///   Pending  --healthy--> Inactive                       (a glitch: no event)
///   Active   --healthy--> Healing --still healthy after clear_after--> Inactive
///   Healing  --failing--> Active                         (a flicker: no event)
///   latched: Healing --> AwaitingAck --acknowledge()--> Inactive
///
/// Events are produced only when a fault is Activated, Cleared or Acknowledged.
class FaultManager
{
public:
  using WallClock = std::function<WallTime()>;

  explicit FaultManager(
    FaultCatalog catalog, FaultManagerConfig config = {},
    WallClock wall_clock = [] {return std::chrono::system_clock::now();});

  /// Feed one observation. `failing` is true when the monitored condition is
  /// bad right now. `source` and `message` are kept for the operator.
  void report(
    const std::string & code, bool failing, TimePoint now,
    const std::string & source = "", const std::string & message = "");

  /// Advance timers: debounce periods that have run out, and watchdogs.
  /// Call regularly (for example at 10 Hz).
  void tick(TimePoint now);

  /// Operator acknowledges a latched fault. Returns true if the fault was
  /// waiting for it. A fault whose cause is still present cannot be acknowledged.
  bool acknowledge(const std::string & code);

  /// Faults currently raised (Active, Healing or AwaitingAck), most severe first.
  std::vector<FaultRecord> raised() const;

  /// Status of one fault, if it has ever been reported or is in the catalogue.
  std::optional<FaultRecord> get(const std::string & code) const;

  Summary summary() const;

  /// Past events, oldest first.
  std::vector<FaultEvent> history(const HistoryQuery & query = {}) const;

  /// Events produced since the previous call, oldest first. For publishing
  /// and for persistence.
  std::vector<FaultEvent> take_new_events();

  /// Load events saved by an earlier run, so history and sequence numbers
  /// continue across a restart. Call before the first report.
  void restore_history(const std::vector<FaultEvent> & events);

  /// Reports received for codes that were refused (accept_unknown_codes = false).
  std::uint64_t dropped_reports() const noexcept {return dropped_reports_;}

private:
  struct Entry
  {
    FaultDefinition definition;
    FaultRecord record;
    TimePoint since{};        ///< When the current Pending/Healing period began.
    TimePoint last_report{};  ///< For the watchdog.
    bool watchdog_armed{false};
    bool watchdog_tripped{false};
  };

  Entry * entry_for(const std::string & code);
  void observe(Entry & entry, bool failing, TimePoint now);
  void activate(Entry & entry);
  void heal_complete(Entry & entry);
  void emit(Entry & entry, EventType type);

  FaultCatalog catalog_;
  FaultManagerConfig config_;
  WallClock wall_clock_;
  std::map<std::string, Entry> entries_;
  std::deque<FaultEvent> history_;
  std::vector<FaultEvent> new_events_;
  std::uint64_t next_sequence_{1};
  std::uint64_t dropped_reports_{0};
};

}  // namespace fault_core

#endif  // FAULT_CORE__FAULT_MANAGER_HPP_
