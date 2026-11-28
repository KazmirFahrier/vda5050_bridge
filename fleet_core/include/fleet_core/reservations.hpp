// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// ReservationTable: who plans to hold which resource during which time.
//
// Times are seconds from the start of the current planning round. The table
// answers one question for the path search: during which intervals may robot
// R hold resource X? That is every time at which no *other* robot has
// reserved X or anything that conflicts with X.

#ifndef FLEET_CORE__RESERVATIONS_HPP_
#define FLEET_CORE__RESERVATIONS_HPP_

#include <limits>
#include <utility>
#include <vector>

#include "fleet_core/conflicts.hpp"

namespace fleet_core
{

inline constexpr double kForever = std::numeric_limits<double>::infinity();

/// Half-open time interval [start, end).
struct TimeInterval
{
  double start{0.0};
  double end{kForever};
};

struct Reservation
{
  double start{0.0};
  double end{kForever};
  int robot{0};
};

class ReservationTable
{
public:
  explicit ReservationTable(const ConflictTable & conflicts);

  /// Robot `robot` holds `resource` during [start, end). Empty intervals are ignored.
  void reserve(ResourceId resource, double start, double end, int robot);

  /// The intervals during which `robot` may hold `resource`, ascending and
  /// disjoint. The robot's own reservations do not restrict it.
  std::vector<TimeInterval> free_intervals(ResourceId resource, int robot) const;

  /// Reservations made on exactly this resource (not on conflicting ones).
  const std::vector<Reservation> & reservations(ResourceId resource) const
  {
    return by_resource_[static_cast<std::size_t>(resource)];
  }
  const ConflictTable & conflicts() const noexcept {return *conflicts_;}

private:
  const ConflictTable * conflicts_;
  std::vector<std::vector<Reservation>> by_resource_;
};

}  // namespace fleet_core

#endif  // FLEET_CORE__RESERVATIONS_HPP_
