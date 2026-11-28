// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fleet_core/roadmap.hpp"

#include <cmath>
#include <limits>

#include "vda5050_core/json.hpp"

namespace fleet_core
{

namespace
{

using vda5050_core::Json;

[[noreturn]] void fail(const std::string & where, const std::string & reason)
{
  throw RoadmapError(where + ": " + reason);
}

std::string string_member(const Json & object, const char * key, const std::string & where)
{
  const Json * value = object.find(key);
  if (value == nullptr || !value->is_string() || value->as_string().empty()) {
    fail(where + "." + key, "expected a non-empty string");
  }
  return value->as_string();
}

double number_member(const Json & object, const char * key, const std::string & where)
{
  const Json * value = object.find(key);
  if (value == nullptr || !value->is_number() || !std::isfinite(value->as_number())) {
    fail(where + "." + key, "expected a number");
  }
  return value->as_number();
}

bool bool_member(const Json & object, const char * key, bool fallback, const std::string & where)
{
  const Json * value = object.find(key);
  if (value == nullptr) {
    return fallback;
  }
  if (!value->is_bool()) {
    fail(where + "." + key, "expected true or false");
  }
  return value->as_bool();
}

}  // namespace

int Roadmap::add_node(const std::string & id, double x, double y, bool parking)
{
  if (id.empty()) {
    fail("node", "id must not be empty");
  }
  if (node_index_.count(id) > 0) {
    fail("node " + id, "duplicate node id");
  }
  if (!std::isfinite(x) || !std::isfinite(y)) {
    fail("node " + id, "coordinates must be finite");
  }
  const int index = node_count();
  nodes_.push_back(MapNode{id, x, y, parking});
  out_edges_.emplace_back();
  in_edges_.emplace_back();
  node_index_[id] = index;
  return index;
}

int Roadmap::add_edge(
  const std::string & id, const std::string & from, const std::string & to, double max_speed)
{
  if (id.empty()) {
    fail("edge", "id must not be empty");
  }
  if (edge_index_.count(id) > 0) {
    fail("edge " + id, "duplicate edge id");
  }
  const std::optional<int> a = find_node(from);
  const std::optional<int> b = find_node(to);
  if (!a) {
    fail("edge " + id, "unknown node '" + from + "'");
  }
  if (!b) {
    fail("edge " + id, "unknown node '" + to + "'");
  }
  if (*a == *b) {
    fail("edge " + id, "starts and ends at the same node");
  }
  for (const int existing : out_edges(*a)) {
    if (edge(existing).to == *b) {
      fail("edge " + id, "duplicates edge '" + edge(existing).id + "'");
    }
  }
  if (!(max_speed >= 0.0) || !std::isfinite(max_speed)) {
    fail("edge " + id, "maxSpeed must be 0 or more");
  }
  const double length = std::hypot(node(*b).x - node(*a).x, node(*b).y - node(*a).y);
  if (length <= 0.0) {
    fail("edge " + id, "its two nodes are at the same position");
  }
  const int index = edge_count();
  edges_.push_back(MapEdge{id, *a, *b, length, max_speed});
  out_edges_[static_cast<std::size_t>(*a)].push_back(index);
  in_edges_[static_cast<std::size_t>(*b)].push_back(index);
  edge_index_[id] = index;
  return index;
}

void Roadmap::add_bidirectional_edge(
  const std::string & id, const std::string & a, const std::string & b, double max_speed)
{
  add_edge(id, a, b, max_speed);
  add_edge(id + ".rev", b, a, max_speed);
}

std::optional<int> Roadmap::find_node(const std::string & id) const
{
  const auto it = node_index_.find(id);
  return it == node_index_.end() ? std::nullopt : std::optional<int>(it->second);
}

std::optional<int> Roadmap::find_edge(const std::string & id) const
{
  const auto it = edge_index_.find(id);
  return it == edge_index_.end() ? std::nullopt : std::optional<int>(it->second);
}

std::string Roadmap::resource_name(ResourceId resource) const
{
  if (resource < 0 || resource >= resource_count()) {
    return "?";
  }
  return is_node(resource) ? "node " + node(resource).id :
         "edge " + edge(resource - node_count()).id;
}

std::optional<int> Roadmap::nearest_node(double x, double y, double tolerance) const
{
  std::optional<int> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int index = 0; index < node_count(); ++index) {
    const double distance = std::hypot(node(index).x - x, node(index).y - y);
    if (distance <= tolerance && distance < best_distance) {
      best = index;
      best_distance = distance;
    }
  }
  return best;
}

Roadmap Roadmap::from_json(std::string_view text)
{
  Json root;
  try {
    root = Json::parse(text);
  } catch (const vda5050_core::JsonParseError & error) {
    fail("roadmap", std::string("invalid JSON: ") + error.what());
  }
  if (!root.is_object()) {
    fail("roadmap", "expected an object");
  }
  Roadmap roadmap;
  if (const Json * map_id = root.find("mapId")) {
    if (!map_id->is_string()) {
      fail("roadmap.mapId", "expected a string");
    }
    roadmap.map_id_ = map_id->as_string();
  }

  const Json * nodes = root.find("nodes");
  if (nodes == nullptr || !nodes->is_array() || nodes->as_array().empty()) {
    fail("roadmap.nodes", "expected a non-empty array");
  }
  std::size_t index = 0;
  for (const Json & entry : nodes->as_array()) {
    const std::string where = "nodes[" + std::to_string(index++) + "]";
    if (!entry.is_object()) {
      fail(where, "expected an object");
    }
    roadmap.add_node(
      string_member(entry, "id", where), number_member(entry, "x", where),
      number_member(entry, "y", where), bool_member(entry, "parking", false, where));
  }

  const Json * edges = root.find("edges");
  if (edges == nullptr || !edges->is_array()) {
    fail("roadmap.edges", "expected an array");
  }
  index = 0;
  for (const Json & entry : edges->as_array()) {
    const std::string where = "edges[" + std::to_string(index++) + "]";
    if (!entry.is_object()) {
      fail(where, "expected an object");
    }
    const std::string id = string_member(entry, "id", where);
    const std::string from = string_member(entry, "from", where);
    const std::string to = string_member(entry, "to", where);
    const double max_speed =
      entry.find("maxSpeed") == nullptr ? 0.0 : number_member(entry, "maxSpeed", where);
    if (bool_member(entry, "bidirectional", false, where)) {
      roadmap.add_bidirectional_edge(id, from, to, max_speed);
    } else {
      roadmap.add_edge(id, from, to, max_speed);
    }
  }
  return roadmap;
}

}  // namespace fleet_core
