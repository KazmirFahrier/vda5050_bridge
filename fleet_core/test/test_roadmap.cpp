// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <string>

#include "fleet_core/conflicts.hpp"
#include "fleet_core/roadmap.hpp"
#include "maps.hpp"

using namespace fleet_core;  // NOLINT(build/namespaces)

namespace
{

std::string roadmap_error(const std::string & text)
{
  try {
    Roadmap::from_json(text);
  } catch (const RoadmapError & error) {
    return error.what();
  }
  return "";
}

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

}  // namespace

TEST(Roadmap, ParsesNodesAndEdges)
{
  const Roadmap map = Roadmap::from_json(
    R"({"mapId":"hall","nodes":[{"id":"A","x":0,"y":0,"parking":true},)"
    R"({"id":"B","x":3,"y":4}],"edges":[{"id":"AB","from":"A","to":"B",)"
    R"("bidirectional":true,"maxSpeed":0.5}]})");
  EXPECT_EQ(map.map_id(), "hall");
  ASSERT_EQ(map.node_count(), 2);
  ASSERT_EQ(map.edge_count(), 2);
  EXPECT_TRUE(map.node(0).parking);
  EXPECT_FALSE(map.node(1).parking);

  const MapEdge & forward = map.edge(*map.find_edge("AB"));
  EXPECT_EQ(forward.from, 0);
  EXPECT_EQ(forward.to, 1);
  EXPECT_DOUBLE_EQ(forward.length, 5.0);
  EXPECT_DOUBLE_EQ(forward.max_speed, 0.5);
  const MapEdge & back = map.edge(*map.find_edge("AB.rev"));
  EXPECT_EQ(back.from, 1);
  EXPECT_EQ(back.to, 0);

  ASSERT_EQ(map.out_edges(0).size(), 1u);
  ASSERT_EQ(map.in_edges(0).size(), 1u);
  EXPECT_EQ(map.resource_count(), 4);
  EXPECT_TRUE(map.is_node(1));
  EXPECT_FALSE(map.is_node(2));
  EXPECT_EQ(map.resource_name(map.node_resource(1)), "node B");
  EXPECT_EQ(map.resource_name(map.edge_resource(0)), "edge AB");
  EXPECT_FALSE(map.find_node("Z").has_value());
}

TEST(Roadmap, OneWayEdgeHasNoReverse)
{
  const Roadmap map = Roadmap::from_json(
    R"({"nodes":[{"id":"A","x":0,"y":0},{"id":"B","x":1,"y":0}],)"
    R"("edges":[{"id":"AB","from":"A","to":"B"}]})");
  EXPECT_EQ(map.edge_count(), 1);
  EXPECT_TRUE(map.out_edges(1).empty());
  EXPECT_EQ(map.map_id(), "map");
}

TEST(Roadmap, ErrorsNameTheOffendingEntry)
{
  const std::string two = R"("nodes":[{"id":"A","x":0,"y":0},{"id":"B","x":1,"y":0}])";
  EXPECT_TRUE(contains(roadmap_error("nope"), "invalid JSON"));
  EXPECT_TRUE(contains(roadmap_error("[]"), "expected an object"));
  EXPECT_TRUE(contains(roadmap_error(R"({"nodes":[],"edges":[]})"), "roadmap.nodes"));
  EXPECT_TRUE(contains(roadmap_error(R"({"nodes":[{"x":0,"y":0}],"edges":[]})"), "nodes[0].id"));
  EXPECT_TRUE(
    contains(roadmap_error(R"({"nodes":[{"id":"A","x":"0","y":0}],"edges":[]})"), "nodes[0].x"));
  EXPECT_TRUE(
    contains(
      roadmap_error(R"({"nodes":[{"id":"A","x":0,"y":0},{"id":"A","x":1,"y":0}],"edges":[]})"),
      "duplicate node id"));
  EXPECT_TRUE(contains(roadmap_error("{" + two + "}"), "roadmap.edges"));
  EXPECT_TRUE(
    contains(
      roadmap_error("{" + two + R"(,"edges":[{"id":"e","from":"A","to":"Z"}]})"),
      "unknown node 'Z'"));
  EXPECT_TRUE(
    contains(
      roadmap_error("{" + two + R"(,"edges":[{"id":"e","from":"A","to":"A"}]})"),
      "same node"));
  EXPECT_TRUE(
    contains(
      roadmap_error(
        "{" + two + R"(,"edges":[{"id":"e","from":"A","to":"B"},{"id":"f","from":"A","to":"B"}]})"),
      "duplicates edge 'e'"));
  EXPECT_TRUE(
    contains(
      roadmap_error(
        "{" + two + R"(,"edges":[{"id":"e","from":"A","to":"B"},{"id":"e","from":"B","to":"A"}]})"),
      "duplicate edge id"));
  EXPECT_TRUE(
    contains(
      roadmap_error("{" + two + R"(,"edges":[{"id":"e","from":"A","to":"B","maxSpeed":-1}]})"),
      "maxSpeed"));
  EXPECT_TRUE(
    contains(
      roadmap_error(
        R"({"nodes":[{"id":"A","x":0,"y":0},{"id":"B","x":0,"y":0}],)"
        R"("edges":[{"id":"e","from":"A","to":"B"}]})"),
      "same position"));
  EXPECT_TRUE(
    contains(
      roadmap_error("{" + two + R"(,"edges":[{"id":"e","from":"A","to":"B","bidirectional":1}]})"),
      "bidirectional"));
}

TEST(Roadmap, NearestNodeRespectsTheTolerance)
{
  const Roadmap map = test_maps::corridor();
  EXPECT_EQ(*map.nearest_node(2.1, 0.1, 0.3), *map.find_node("B"));
  EXPECT_FALSE(map.nearest_node(3.0, 0.0, 0.3).has_value());  // halfway between B and C
  EXPECT_EQ(*map.nearest_node(2.9, 0.0, 2.0), *map.find_node("B"));  // nearest of several
}

// ---------------------------------------------------------------- conflicts ----

TEST(Conflicts, SegmentDistance)
{
  // Crossing segments touch.
  EXPECT_DOUBLE_EQ(segment_distance(0, 0, 2, 2, 0, 2, 2, 0), 0.0);
  // Parallel, 1 apart.
  EXPECT_DOUBLE_EQ(segment_distance(0, 0, 4, 0, 1, 1, 3, 1), 1.0);
  // End to end with a gap of 2.
  EXPECT_DOUBLE_EQ(segment_distance(0, 0, 1, 0, 3, 0, 5, 0), 2.0);
  // T shape: the end of one is 0.5 from the middle of the other.
  EXPECT_DOUBLE_EQ(segment_distance(0, 0, 4, 0, 2, 0.5, 2, 3), 0.5);
  // Two points.
  EXPECT_DOUBLE_EQ(segment_distance(0, 0, 0, 0, 3, 4, 3, 4), 5.0);
  // Point and segment, nearest to an end.
  EXPECT_DOUBLE_EQ(segment_distance(-3, 4, -3, 4, 0, 0, 5, 0), 5.0);
  // Sharing an end point.
  EXPECT_DOUBLE_EQ(segment_distance(0, 0, 1, 0, 1, 0, 1, 5), 0.0);
}

TEST(Conflicts, CorridorNeighboursConflictAndDistantOnesDoNot)
{
  const Roadmap map = test_maps::corridor();
  const ConflictTable table(map, test_maps::kClearance);
  const auto node = [&map](const char * id) {return map.node_resource(*map.find_node(id));};
  const auto edge = [&map](const char * id) {return map.edge_resource(*map.find_edge(id));};

  EXPECT_TRUE(table.conflict(node("A"), node("A")));
  EXPECT_FALSE(table.conflict(node("A"), node("B")));  // 2 m apart, clearance 1 m
  EXPECT_TRUE(table.conflict(edge("AB"), node("A")));
  EXPECT_TRUE(table.conflict(edge("AB"), node("B")));
  EXPECT_FALSE(table.conflict(edge("AB"), node("C")));
  EXPECT_TRUE(table.conflict(edge("AB"), edge("AB.rev")));  // same lane, other direction
  EXPECT_TRUE(table.conflict(edge("AB"), edge("BC")));       // they meet at B
  EXPECT_FALSE(table.conflict(edge("AB"), edge("CD")));
  EXPECT_TRUE(table.conflict(node("B"), edge("AB")));        // symmetric

  // conflicts() lists the same pairs, itself included.
  const std::vector<ResourceId> & of_ab = table.conflicts(edge("AB"));
  EXPECT_EQ(of_ab.size(), 6u);  // A, B, AB, AB.rev, BC, BC.rev
}

TEST(Conflicts, CrossingAndTooCloseParallelEdgesAreFoundFromGeometry)
{
  Roadmap map;
  map.add_node("W", 0, 0);
  map.add_node("E", 4, 0);
  map.add_node("S", 2, -2);
  map.add_node("N", 2, 2);
  map.add_node("P", 0, 0.6);   // a lane 0.6 m beside W-E
  map.add_node("Q", 4, 0.6);
  map.add_node("F", 0, 5);     // a lane far away
  map.add_node("G", 4, 5);
  map.add_edge("WE", "W", "E");
  map.add_edge("SN", "S", "N");
  map.add_edge("PQ", "P", "Q");
  map.add_edge("FG", "F", "G");
  const ConflictTable table(map, 1.0);
  const auto edge = [&map](const char * id) {return map.edge_resource(*map.find_edge(id));};

  EXPECT_TRUE(table.conflict(edge("WE"), edge("SN")));   // they cross; nothing marks it
  EXPECT_TRUE(table.conflict(edge("WE"), edge("PQ")));   // too close side by side
  EXPECT_FALSE(table.conflict(edge("WE"), edge("FG")));
  EXPECT_TRUE(table.conflict(map.node_resource(*map.find_node("P")), edge("WE")));

  // With a smaller vehicle the side-by-side lanes become usable together.
  const ConflictTable narrow(map, 0.5);
  EXPECT_FALSE(narrow.conflict(edge("WE"), edge("PQ")));
  EXPECT_TRUE(narrow.conflict(edge("WE"), edge("SN")));
}
