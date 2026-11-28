// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fleet_core/reservations.hpp"

#include <algorithm>

namespace fleet_core
{

ReservationTable::ReservationTable(const ConflictTable & conflicts)
: conflicts_(&conflicts), by_resource_(conflicts.size()) {}

void ReservationTable::reserve(ResourceId resource, double start, double end, int robot)
{
  if (!(end > start)) {
    return;
  }
  by_resource_[static_cast<std::size_t>(resource)].push_back(Reservation{start, end, robot});
}

std::vector<TimeInterval> ReservationTable::free_intervals(ResourceId resource, int robot) const
{
  // Everything other robots have reserved on this resource or a conflicting one.
  std::vector<TimeInterval> blocked;
  for (const ResourceId other : conflicts_->conflicts(resource)) {
    for (const Reservation & reservation : by_resource_[static_cast<std::size_t>(other)]) {
      if (reservation.robot != robot) {
        blocked.push_back(TimeInterval{reservation.start, reservation.end});
      }
    }
  }
  std::sort(
    blocked.begin(), blocked.end(),
    [](const TimeInterval & a, const TimeInterval & b) {return a.start < b.start;});

  // The complement of their union within [0, forever).
  std::vector<TimeInterval> free;
  double cursor = 0.0;
  for (const TimeInterval & interval : blocked) {
    if (interval.start > cursor) {
      free.push_back(TimeInterval{cursor, interval.start});
    }
    cursor = std::max(cursor, interval.end);
    if (cursor == kForever) {
      break;
    }
  }
  if (cursor < kForever) {
    free.push_back(TimeInterval{cursor, kForever});
  }
  return free;
}

}  // namespace fleet_core
