// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Which resources may not be held by two vehicles at the same time.
//
// A vehicle that holds a node may be anywhere within a small tolerance of it;
// a vehicle that holds an edge may be anywhere on it. Two resources conflict
// if vehicles on them could come closer than `clearance` (two vehicle radii
// plus a margin). This is decided from geometry, once, when the map is loaded:
// the distance between two points, a point and a segment, or two segments.
//
// The table is what makes traffic control independent of the map's shape:
// crossings, parallel lanes that are too close, a node next to a passing lane
// are all found by the same rule, with nothing to mark by hand.

#ifndef FLEET_CORE__CONFLICTS_HPP_
#define FLEET_CORE__CONFLICTS_HPP_

#include <vector>

#include "fleet_core/roadmap.hpp"

namespace fleet_core
{

class ConflictTable
{
public:
  ConflictTable() = default;
  /// `clearance`: the smallest distance allowed between the reference points
  /// of two vehicles, in metres.
  ConflictTable(const Roadmap & roadmap, double clearance);

  bool conflict(ResourceId a, ResourceId b) const
  {
    return matrix_[static_cast<std::size_t>(a) * size_ + static_cast<std::size_t>(b)];
  }
  /// Every resource that conflicts with `resource`, itself included, ascending.
  const std::vector<ResourceId> & conflicts(ResourceId resource) const
  {
    return lists_[static_cast<std::size_t>(resource)];
  }
  double clearance() const noexcept {return clearance_;}
  std::size_t size() const noexcept {return size_;}

private:
  std::size_t size_{0};
  double clearance_{0.0};
  std::vector<bool> matrix_;
  std::vector<std::vector<ResourceId>> lists_;
};

/// Smallest distance between segments p1-p2 and q1-q2 (a point is a segment
/// whose ends coincide). Exposed for tests.
double segment_distance(
  double p1x, double p1y, double p2x, double p2y,
  double q1x, double q1y, double q2x, double q2y);

}  // namespace fleet_core

#endif  // FLEET_CORE__CONFLICTS_HPP_
