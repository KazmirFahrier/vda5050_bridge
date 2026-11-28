// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fleet_core/conflicts.hpp"

#include <algorithm>
#include <cmath>

namespace fleet_core
{

namespace
{

struct Point
{
  double x;
  double y;
};

double dot(Point a, Point b) {return a.x * b.x + a.y * b.y;}
double cross(Point a, Point b) {return a.x * b.y - a.y * b.x;}
Point sub(Point a, Point b) {return Point{a.x - b.x, a.y - b.y};}

double point_segment_distance(Point p, Point a, Point b)
{
  const Point ab = sub(b, a);
  const double length_squared = dot(ab, ab);
  if (length_squared <= 0.0) {
    return std::hypot(p.x - a.x, p.y - a.y);
  }
  const double t = std::clamp(dot(sub(p, a), ab) / length_squared, 0.0, 1.0);
  return std::hypot(p.x - (a.x + t * ab.x), p.y - (a.y + t * ab.y));
}

/// 1 / -1 / 0: on which side of the line a-b the point c lies.
int side(Point a, Point b, Point c)
{
  const double value = cross(sub(b, a), sub(c, a));
  return value > 0.0 ? 1 : (value < 0.0 ? -1 : 0);
}

bool properly_intersect(Point p1, Point p2, Point q1, Point q2)
{
  const int d1 = side(q1, q2, p1);
  const int d2 = side(q1, q2, p2);
  const int d3 = side(p1, p2, q1);
  const int d4 = side(p1, p2, q2);
  return d1 * d2 < 0 && d3 * d4 < 0;
}

struct Segment
{
  Point a;
  Point b;
};

Segment geometry(const Roadmap & roadmap, ResourceId resource)
{
  if (roadmap.is_node(resource)) {
    const MapNode & node = roadmap.node(resource);
    return Segment{Point{node.x, node.y}, Point{node.x, node.y}};
  }
  const MapEdge & edge = roadmap.edge(resource - roadmap.node_count());
  const MapNode & from = roadmap.node(edge.from);
  const MapNode & to = roadmap.node(edge.to);
  return Segment{Point{from.x, from.y}, Point{to.x, to.y}};
}

}  // namespace

double segment_distance(
  double p1x, double p1y, double p2x, double p2y,
  double q1x, double q1y, double q2x, double q2y)
{
  const Point p1{p1x, p1y};
  const Point p2{p2x, p2y};
  const Point q1{q1x, q1y};
  const Point q2{q2x, q2y};
  if (properly_intersect(p1, p2, q1, q2)) {
    return 0.0;
  }
  // Otherwise the closest approach involves an end point of one segment.
  return std::min(
    std::min(point_segment_distance(p1, q1, q2), point_segment_distance(p2, q1, q2)),
    std::min(point_segment_distance(q1, p1, p2), point_segment_distance(q2, p1, p2)));
}

ConflictTable::ConflictTable(const Roadmap & roadmap, double clearance)
: size_(static_cast<std::size_t>(roadmap.resource_count())), clearance_(clearance),
  matrix_(size_ * size_, false), lists_(size_)
{
  std::vector<Segment> shapes;
  shapes.reserve(size_);
  for (ResourceId resource = 0; resource < roadmap.resource_count(); ++resource) {
    shapes.push_back(geometry(roadmap, resource));
  }
  for (std::size_t a = 0; a < size_; ++a) {
    for (std::size_t b = a; b < size_; ++b) {
      const bool hit = a == b || segment_distance(
        shapes[a].a.x, shapes[a].a.y, shapes[a].b.x, shapes[a].b.y,
        shapes[b].a.x, shapes[b].a.y, shapes[b].b.x, shapes[b].b.y) < clearance;
      matrix_[a * size_ + b] = hit;
      matrix_[b * size_ + a] = hit;
    }
  }
  for (std::size_t a = 0; a < size_; ++a) {
    for (std::size_t b = 0; b < size_; ++b) {
      if (matrix_[a * size_ + b]) {
        lists_[a].push_back(static_cast<ResourceId>(b));
      }
    }
  }
}

}  // namespace fleet_core
