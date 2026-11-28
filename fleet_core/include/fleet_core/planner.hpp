// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Planning.
//
// plan_route() finds the fastest route for ONE robot through the roadmap in
// space and time, around what other robots have already reserved. It may wait
// at nodes. The search is "safe interval path planning" (SIPP): instead of
// stepping through time in fixed ticks, a search state is a node together
// with one of the time intervals in which that node is free.
//
// plan_fleet() plans ALL robots, in two stages:
//   1. Prioritized planning: one robot after another in priority order, each
//      around the reservations of those planned before it. Fast, and always
//      yields a valid plan, but a robot planned early never yields to one
//      planned later, so some solvable situations (two robots meeting head-on
//      next to a siding) leave a robot without a route. If that happens, a
//      few other orders are tried, since who is planned first often decides
//      whether everybody fits.
//   2. Only if that happened: conflict-based search. Every robot is planned
//      alone; then, for a pair that would collide, the search tries both ways
//      of resolving it (this one gives way / that one gives way) and continues
//      with the cheaper total. It is bounded by a node budget; if the budget
//      runs out, the result of stage 1 stands.
//
// What a robot holds, and when (this is the contract with the executor):
//   - While it stands at a node it holds that node.
//   - To drive an edge from A to B it takes the edge AND node B at the moment
//     it starts, and keeps A until it has arrived at B. So during the move it
//     holds A, the edge and B; after it, only B.
// A plan in which no two robots hold conflicting resources at the same time is
// called valid; validate_fleet_plan() checks exactly that.

#ifndef FLEET_CORE__PLANNER_HPP_
#define FLEET_CORE__PLANNER_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "fleet_core/reservations.hpp"
#include "fleet_core/roadmap.hpp"

namespace fleet_core
{

/// A place the robot must visit, and how long it stays there (for actions).
struct Waypoint
{
  int node{0};
  double dwell{0.0};  ///< Seconds.
};

struct PlannerConfig
{
  double default_speed{1.0};   ///< m/s, when neither robot nor edge sets a limit.
  double node_overhead{1.0};   ///< Seconds added per edge for stopping and turning.
  /// Cost (seconds) of ending at a node that is not a parking node, when an
  /// idle robot has to get out of the way and may choose where to go.
  double rest_penalty{60.0};
  /// Upper bound on search states expanded for one robot; guards the caller
  /// against a pathological map. 0 means no bound.
  std::size_t max_expansions{200000};
  /// How many different planning orders stage 1 may try when the natural one
  /// (by priority) leaves a robot without a route. The orders are fixed
  /// permutations, not random ones: the same problem gives the same plan.
  std::size_t order_attempts{6};
  /// Node budget of the conflict-based search (stage 2 of plan_fleet).
  /// 0 turns the search off: prioritized planning only.
  std::size_t search_budget{300};
  /// Second bound on the same search: the total number of route-search states
  /// it may expand. Counted work, not wall-clock time, so that the same
  /// problem always gives the same answer.
  std::size_t search_work{30000};
};

struct PlannedMove
{
  int edge{0};
  int from{0};
  int to{0};
  double start{0.0};  ///< Takes the edge and `to`.
  double end{0.0};    ///< Arrives at `to`; releases `from` and the edge.
};

struct Plan
{
  std::vector<PlannedMove> moves;
  /// For each waypoint of the request: how many moves have been completed
  /// when the robot is at it (0 = at the start node).
  std::vector<std::size_t> waypoint_after_moves;
  double finish_time{0.0};  ///< When the robot is done (arrived and dwelt).
};

struct RouteRequest
{
  int robot{0};
  int start_node{0};
  double start_time{0.0};  ///< From when the robot is free to leave start_node.
  /// Nodes to visit in order; the robot must be able to stay at the last one
  /// indefinitely. Empty: go anywhere it can stay indefinitely, preferring to
  /// stay put, then parking nodes.
  std::vector<Waypoint> waypoints;
  double speed{0.0};       ///< m/s; 0 uses PlannerConfig::default_speed.
  /// Extra reservations that apply to this robot only, whoever made them.
  /// Used by the conflict-based search to make one robot give way.
  std::vector<std::pair<ResourceId, TimeInterval>> give_way;
};

/// Seconds a robot needs for an edge, including the per-edge overhead.
double travel_time(const MapEdge & edge, double robot_speed, const PlannerConfig & config);

/// Shortest travel time from every node to `goal` when nobody else is on the
/// map; kForever where `goal` cannot be reached.
std::vector<double> travel_times_to(
  const Roadmap & roadmap, int goal, double robot_speed, const PlannerConfig & config);

/// The fastest valid plan, or nullopt if there is none. `expansions`, if
/// given, is increased by the number of search states expanded.
std::optional<Plan> plan_route(
  const Roadmap & roadmap, const ReservationTable & table, const RouteRequest & request,
  const PlannerConfig & config = {}, std::size_t * expansions = nullptr);

// ------------------------------------------------------------- the fleet ----

struct AgentRequest
{
  int robot{0};
  /// Where the robot will be when it has finished what it is already
  /// committed to (the end of the released part of its order).
  int start_node{0};
  /// Estimated seconds until it gets there. 0 if it is standing there.
  double start_time{0.0};
  /// Everything the robot holds now. Kept until start_time.
  std::vector<ResourceId> held;
  std::vector<Waypoint> waypoints;
  double speed{0.0};
  /// A robot that cannot be given orders (offline, faulted). It keeps what it
  /// holds, indefinitely, and the others plan around it.
  bool immobile{false};
  /// Lower values are planned first. Ties are broken by position in the list.
  double priority{0.0};
};

enum class AgentOutcome
{
  Planned,   ///< A plan to all waypoints (or an idle robot's resting place).
  /// No route to the waypoints was found this time. The robot is treated like
  /// an idle one: its plan is empty (stay) or takes it out of the others' way.
  Blocked,
  Immobile,  ///< Was not planned because it cannot move.
};

struct AgentPlan
{
  int robot{0};
  AgentOutcome outcome{AgentOutcome::Blocked};
  /// Planned: the route to the waypoints (may be empty: already there).
  /// Blocked: empty, or moves that take the robot out of the way.
  Plan plan;
};

struct FleetPlan
{
  std::vector<AgentPlan> agents;  ///< Same order as the requests.
  std::size_t attempts{0};        ///< Prioritized passes needed (1 = first order worked).
  std::size_t search_nodes{0};    ///< Nodes the conflict-based search expanded (0 = not run).
  bool from_search{false};        ///< The plan is the result of the search.
};

/// Plan every robot. Never fails: a robot without a route to its waypoints is
/// reported as Blocked, and the plans of the others respect what it does instead.
FleetPlan plan_fleet(
  const Roadmap & roadmap, const ConflictTable & conflicts,
  const std::vector<AgentRequest> & requests, const PlannerConfig & config = {});

/// Check a fleet plan against the definition of validity above, independently
/// of how it was made. Returns a description of the first violation, or an
/// empty string. Used by tests, and by the fleet manager before it trusts a plan.
std::string validate_fleet_plan(
  const Roadmap & roadmap, const ConflictTable & conflicts,
  const std::vector<AgentRequest> & requests, const FleetPlan & plan);

}  // namespace fleet_core

#endif  // FLEET_CORE__PLANNER_HPP_
