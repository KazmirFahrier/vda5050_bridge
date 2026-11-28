// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Small roadmaps used by several tests.

#ifndef MAPS_HPP_
#define MAPS_HPP_

#include <cmath>
#include <string>

#include "fleet_core/roadmap.hpp"

namespace test_maps
{

/// Vehicle-to-vehicle clearance used with these maps (nodes are 2 m apart).
inline constexpr double kClearance = 1.0;

/// A - B - C - D - E in a line, 2 m apart, all two-way.
inline fleet_core::Roadmap corridor()
{
  fleet_core::Roadmap map;
  const char * names[] = {"A", "B", "C", "D", "E"};
  for (int i = 0; i < 5; ++i) {
    map.add_node(names[i], 2.0 * i, 0.0, i == 0 || i == 4);
  }
  for (int i = 0; i + 1 < 5; ++i) {
    map.add_bidirectional_edge(
      std::string(names[i]) + names[i + 1], names[i], names[i + 1]);
  }
  return map;
}

/// The corridor with a siding S above C: the one place where two vehicles
/// travelling in opposite directions can pass.
inline fleet_core::Roadmap corridor_with_siding()
{
  fleet_core::Roadmap map = corridor();
  map.add_node("S", 4.0, 2.0, true);
  map.add_bidirectional_edge("CS", "C", "S");
  return map;
}

/// width x height nodes, `spacing` metres apart, named "x,y", all two-way.
inline fleet_core::Roadmap grid(int width, int height, double spacing = 2.0)
{
  fleet_core::Roadmap map;
  const auto name = [](int x, int y) {return std::to_string(x) + "," + std::to_string(y);};
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      map.add_node(name(x, y), spacing * x, spacing * y);
    }
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (x + 1 < width) {
        map.add_bidirectional_edge("h" + name(x, y), name(x, y), name(x + 1, y));
      }
      if (y + 1 < height) {
        map.add_bidirectional_edge("v" + name(x, y), name(x, y), name(x, y + 1));
      }
    }
  }
  return map;
}

/// `count` nodes on a circle, each connected to the next, one-way.
inline fleet_core::Roadmap one_way_ring(int count, double spacing = 2.0)
{
  fleet_core::Roadmap map;
  const double radius = spacing * count / 6.283185307179586;
  for (int i = 0; i < count; ++i) {
    const double angle = 6.283185307179586 * i / count;
    map.add_node("R" + std::to_string(i), radius * std::cos(angle), radius * std::sin(angle));
  }
  for (int i = 0; i < count; ++i) {
    map.add_edge(
      "r" + std::to_string(i), "R" + std::to_string(i), "R" + std::to_string((i + 1) % count));
  }
  return map;
}

}  // namespace test_maps

#endif  // MAPS_HPP_
