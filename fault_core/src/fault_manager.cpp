// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fault_core/fault_manager.hpp"

#include <algorithm>
#include <utility>

namespace fault_core
{

FaultManager::FaultManager(FaultCatalog catalog, FaultManagerConfig config, WallClock wall_clock)
: catalog_(std::move(catalog)), config_(config), wall_clock_(std::move(wall_clock))
{
  // Every catalogued fault exists from the start, so that its watchdog runs
  // even if its monitor never sends a single report.
  for (const auto & [code, definition] : catalog_.definitions()) {
    Entry entry;
    entry.definition = definition;
    entry.record.code = code;
    entry.record.severity = definition.severity;
    entry.record.description = definition.description;
    entries_.emplace(code, std::move(entry));
  }
}

FaultManager::Entry * FaultManager::entry_for(const std::string & code)
{
  const auto it = entries_.find(code);
  if (it != entries_.end()) {
    return &it->second;
  }
  if (!config_.accept_unknown_codes || code.empty()) {
    ++dropped_reports_;
    return nullptr;
  }
  Entry entry;
  entry.definition.code = code;
  entry.definition.severity = config_.unknown_code_severity;
  entry.definition.description = "fault code not in the catalogue";
  entry.record.code = code;
  entry.record.severity = config_.unknown_code_severity;
  entry.record.description = entry.definition.description;
  entry.record.known = false;
  return &entries_.emplace(code, std::move(entry)).first->second;
}

void FaultManager::report(
  const std::string & code, bool failing, TimePoint now,
  const std::string & source, const std::string & message)
{
  Entry * entry = entry_for(code);
  if (entry == nullptr) {
    return;
  }
  entry->last_report = now;
  entry->watchdog_armed = true;
  entry->watchdog_tripped = false;  // the monitor is alive; its report decides
  if (failing) {
    if (!message.empty()) {
      entry->record.message = message;
    }
    if (!source.empty()) {
      entry->record.source = source;
    }
  }
  observe(*entry, failing, now);
}

void FaultManager::tick(TimePoint now)
{
  for (auto & [code, entry] : entries_) {
    (void)code;
    const Duration timeout = entry.definition.timeout;
    if (timeout.count() > 0) {
      if (!entry.watchdog_armed) {
        // First tick: start counting from here.
        entry.watchdog_armed = true;
        entry.last_report = now;
      } else if (!entry.watchdog_tripped && now - entry.last_report >= timeout) {
        entry.watchdog_tripped = true;
        entry.record.message =
          "no report for " + std::to_string(timeout.count()) + " ms";
        entry.record.source = "watchdog";
      }
    }

    if (entry.watchdog_tripped) {
      observe(entry, true, now);
    } else if (entry.record.state == FaultState::Pending) {
      // No healthy report has arrived, so the condition is taken to persist.
      observe(entry, true, now);
    } else if (entry.record.state == FaultState::Healing) {
      observe(entry, false, now);
    }
  }
}

void FaultManager::observe(Entry & entry, bool failing, TimePoint now)
{
  FaultState & state = entry.record.state;
  switch (state) {
    case FaultState::Inactive:
      if (failing) {
        if (entry.definition.activate_after.count() == 0) {
          activate(entry);
        } else {
          state = FaultState::Pending;
          entry.since = now;
        }
      }
      break;
    case FaultState::Pending:
      if (!failing) {
        state = FaultState::Inactive;  // a glitch shorter than the debounce time
      } else if (now - entry.since >= entry.definition.activate_after) {
        activate(entry);
      }
      break;
    case FaultState::Active:
      if (!failing) {
        if (entry.definition.clear_after.count() == 0) {
          heal_complete(entry);
        } else {
          state = FaultState::Healing;
          entry.since = now;
        }
      }
      break;
    case FaultState::Healing:
      if (failing) {
        state = FaultState::Active;  // it flickered: same occurrence, no new event
      } else if (now - entry.since >= entry.definition.clear_after) {
        heal_complete(entry);
      }
      break;
    case FaultState::AwaitingAck:
      if (failing) {
        state = FaultState::Active;  // the cause came back before anyone acknowledged
      }
      break;
  }
}

void FaultManager::activate(Entry & entry)
{
  entry.record.state = FaultState::Active;
  ++entry.record.occurrence_count;
  emit(entry, EventType::Activated);
  if (entry.record.occurrence_count == 1) {
    entry.record.first_activated = entry.record.last_changed;
  }
  entry.record.last_activated = entry.record.last_changed;
}

void FaultManager::heal_complete(Entry & entry)
{
  if (entry.definition.latched) {
    // Stays visible until an operator has seen it.
    entry.record.state = FaultState::AwaitingAck;
    entry.record.last_changed = wall_clock_();
    return;
  }
  entry.record.state = FaultState::Inactive;
  emit(entry, EventType::Cleared);
}

bool FaultManager::acknowledge(const std::string & code)
{
  const auto it = entries_.find(code);
  if (it == entries_.end() || it->second.record.state != FaultState::AwaitingAck) {
    return false;
  }
  it->second.record.state = FaultState::Inactive;
  emit(it->second, EventType::Acknowledged);
  return true;
}

void FaultManager::emit(Entry & entry, EventType type)
{
  FaultEvent event;
  event.sequence = next_sequence_++;
  event.type = type;
  event.code = entry.record.code;
  event.severity = entry.record.severity;
  event.time = wall_clock_();
  event.message = entry.record.message;
  event.source = entry.record.source;
  event.occurrence_count = entry.record.occurrence_count;
  entry.record.last_changed = event.time;

  history_.push_back(event);
  while (history_.size() > config_.history_capacity) {
    history_.pop_front();
  }
  // Bounded as well, in case nobody ever collects them.
  new_events_.push_back(std::move(event));
  if (new_events_.size() > config_.history_capacity) {
    new_events_.erase(new_events_.begin());
  }
}

std::vector<FaultRecord> FaultManager::raised() const
{
  std::vector<FaultRecord> records;
  for (const auto & [code, entry] : entries_) {
    (void)code;
    if (is_raised(entry.record.state)) {
      records.push_back(entry.record);
    }
  }
  std::stable_sort(
    records.begin(), records.end(), [](const FaultRecord & a, const FaultRecord & b) {
      return static_cast<int>(a.severity) > static_cast<int>(b.severity);
    });
  return records;
}

std::optional<FaultRecord> FaultManager::get(const std::string & code) const
{
  const auto it = entries_.find(code);
  if (it == entries_.end()) {
    return std::nullopt;
  }
  return it->second.record;
}

Summary FaultManager::summary() const
{
  Summary summary;
  for (const auto & [code, entry] : entries_) {
    (void)code;
    if (!is_raised(entry.record.state)) {
      continue;
    }
    ++summary.raised;
    ++summary.by_severity[static_cast<int>(entry.record.severity)];
    if (!summary.highest ||
      static_cast<int>(entry.record.severity) > static_cast<int>(*summary.highest))
    {
      summary.highest = entry.record.severity;
    }
  }
  return summary;
}

std::vector<FaultEvent> FaultManager::history(const HistoryQuery & query) const
{
  std::vector<FaultEvent> events;
  for (const FaultEvent & event : history_) {
    if (event.sequence <= query.after_sequence) {
      continue;
    }
    if (query.code && event.code != *query.code) {
      continue;
    }
    if (query.min_severity &&
      static_cast<int>(event.severity) < static_cast<int>(*query.min_severity))
    {
      continue;
    }
    events.push_back(event);
  }
  if (query.limit > 0 && events.size() > query.limit) {
    events.erase(events.begin(), events.end() - static_cast<std::ptrdiff_t>(query.limit));
  }
  return events;
}

std::vector<FaultEvent> FaultManager::take_new_events()
{
  return std::exchange(new_events_, {});
}

void FaultManager::restore_history(const std::vector<FaultEvent> & events)
{
  for (const FaultEvent & event : events) {
    history_.push_back(event);
    next_sequence_ = std::max(next_sequence_, event.sequence + 1);

    // Carry the counters over so "this is the 7th time" stays true after a
    // restart. Whether a fault is active now is NOT restored: that is decided
    // afresh from live reports.
    Entry * entry = entry_for(event.code);
    if (entry != nullptr && event.type == EventType::Activated) {
      if (entry->record.occurrence_count == 0) {
        entry->record.first_activated = event.time;
      }
      entry->record.occurrence_count =
        std::max(entry->record.occurrence_count, event.occurrence_count);
      entry->record.last_activated = event.time;
    }
  }
  while (history_.size() > config_.history_capacity) {
    history_.pop_front();
  }
}

}  // namespace fault_core
