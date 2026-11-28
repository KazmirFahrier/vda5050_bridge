// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// The roadmap: the graph of nodes and directed edges that vehicles drive on.
//
// Nodes and edges are also the *resources* of traffic control. A vehicle may
// only be where it holds the resource, and two vehicles never hold resources
// that are too close to each other (see conflicts.hpp). Resources are numbered
// in one range: nodes first (0 .. node_count-1), then edges.

#ifndef FLEET_CORE__ROADMAP_HPP_
#define FLEET_CORE__ROADMAP_HPP_

#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace fleet_core
{

/// Index of a node or an edge in the combined resource numbering.
using ResourceId = int;

struct MapNode
{
  std::string id;
  double x{0.0};
  double y{0.0};
  bool parking{false};  ///< A good place for an idle vehicle to wait.
};

struct MapEdge
{
  std::string id;
  int from{0};            ///< Node index.
  int to{0};              ///< Node index.
  double length{0.0};     ///< Metres; straight line between the two nodes.
  double max_speed{0.0};  ///< m/s; 0 means "no limit of its own".
};

class RoadmapError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class Roadmap
{
public:
  Roadmap() = default;

  /// Parse
  ///   {"mapId": "hall",
  ///    "nodes": [{"id": "A", "x": 0, "y": 0, "parking": true}, ...],
  ///    "edges": [{"id": "A-B", "from": "A", "to": "B",
  ///               "bidirectional": true, "maxSpeed": 1.0}, ...]}
  /// A bidirectional edge becomes two directed edges; the reverse one gets the
  /// id "<id>.rev". Throws RoadmapError naming the offending entry.
  static Roadmap from_json(std::string_view text);

  void set_map_id(std::string map_id) {map_id_ = std::move(map_id);}
  /// Returns the index of the new node. Throws on a duplicate id.
  int add_node(const std::string & id, double x, double y, bool parking = false);
  /// Adds a directed edge and returns its index. Throws if an end is unknown,
  /// the ends coincide, or the same connection already exists.
  int add_edge(
    const std::string & id, const std::string & from, const std::string & to,
    double max_speed = 0.0);
  /// Adds both directions; the reverse edge gets the id "<id>.rev".
  void add_bidirectional_edge(
    const std::string & id, const std::string & a, const std::string & b,
    double max_speed = 0.0);

  const std::string & map_id() const noexcept {return map_id_;}
  int node_count() const noexcept {return static_cast<int>(nodes_.size());}
  int edge_count() const noexcept {return static_cast<int>(edges_.size());}
  int resource_count() const noexcept {return node_count() + edge_count();}

  const MapNode & node(int index) const {return nodes_.at(static_cast<std::size_t>(index));}
  const MapEdge & edge(int index) const {return edges_.at(static_cast<std::size_t>(index));}
  std::optional<int> find_node(const std::string & id) const;
  std::optional<int> find_edge(const std::string & id) const;
  /// Indices of the edges that start at `node`.
  const std::vector<int> & out_edges(int node) const
  {
    return out_edges_.at(static_cast<std::size_t>(node));
  }
  const std::vector<int> & in_edges(int node) const
  {
    return in_edges_.at(static_cast<std::size_t>(node));
  }

  ResourceId node_resource(int node) const noexcept {return node;}
  ResourceId edge_resource(int edge) const noexcept {return node_count() + edge;}
  bool is_node(ResourceId resource) const noexcept {return resource < node_count();}
  /// Human-readable name of a resource, for logs and reports.
  std::string resource_name(ResourceId resource) const;

  /// The node nearest to (x, y) within `tolerance` metres, if any.
  std::optional<int> nearest_node(double x, double y, double tolerance) const;

private:
  std::string map_id_{"map"};
  std::vector<MapNode> nodes_;
  std::vector<MapEdge> edges_;
  std::vector<std::vector<int>> out_edges_;
  std::vector<std::vector<int>> in_edges_;
  std::unordered_map<std::string, int> node_index_;
  std::unordered_map<std::string, int> edge_index_;
};

}  // namespace fleet_core

#endif  // FLEET_CORE__ROADMAP_HPP_
