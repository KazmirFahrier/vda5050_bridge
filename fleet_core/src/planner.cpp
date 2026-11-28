// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fleet_core/planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <queue>
#include <tuple>
#include <utility>

namespace fleet_core
{

namespace
{

// Times are sums of a few dozen doubles; comparisons allow for rounding.
constexpr double kEpsilon = 1e-9;

/// Robot number used for reservations that belong to no robot.
constexpr int kNobody = -1;

/// One node of the search tree.
struct SearchState
{
  int node{0};
  std::size_t interval{0};  ///< Index into the free intervals of `node`.
  std::size_t stage{0};     ///< Waypoints visited so far.
  double time{0.0};         ///< Earliest time the robot is ready to leave.
  int parent{-1};
  PlannedMove move{};       ///< The move that led here (unused for the root).
  std::size_t stage_on_arrival{0};  ///< Stage before any waypoint at this node was consumed.
};

class RouteSearch
{
public:
  RouteSearch(
    const Roadmap & roadmap, const ReservationTable & table, const RouteRequest & request,
    const PlannerConfig & config)
  : roadmap_(roadmap), table_(table), request_(request), config_(config),
    free_(static_cast<std::size_t>(roadmap.resource_count())),
    known_(static_cast<std::size_t>(roadmap.resource_count()), false)
  {
    if (!request_.give_way.empty()) {
      // The table plus this robot's private reservations, under a robot
      // number nobody has.
      private_table_ = table_;
      for (const auto & [other, interval] : request_.give_way) {
        private_table_->reserve(other, interval.start, interval.end, kNobody);
      }
    }
    // Remaining-time estimates: exact travel times on the empty map, so they
    // never overestimate and the first goal state taken from the queue is optimal.
    const std::size_t count = request_.waypoints.size();
    to_waypoint_.reserve(count);
    for (const Waypoint & waypoint : request_.waypoints) {
      to_waypoint_.push_back(travel_times_to(roadmap_, waypoint.node, request_.speed, config_));
    }
    after_.assign(count + 1, 0.0);
    for (std::size_t k = count; k-- > 0; ) {
      after_[k] = request_.waypoints[k].dwell + after_[k + 1];
      if (k + 1 < count) {
        after_[k] +=
          to_waypoint_[k + 1][static_cast<std::size_t>(request_.waypoints[k].node)];
      }
    }
  }

  std::optional<Plan> run()
  {
    const std::vector<TimeInterval> & start_intervals =
      intervals(roadmap_.node_resource(request_.start_node));
    std::optional<std::size_t> start_interval;
    for (std::size_t i = 0; i < start_intervals.size(); ++i) {
      if (start_intervals[i].start <= request_.start_time + kEpsilon &&
        request_.start_time < start_intervals[i].end)
      {
        start_interval = i;
      }
    }
    if (!start_interval) {
      return std::nullopt;  // somebody else is planned onto the robot's own node
    }

    SearchState root;
    root.node = request_.start_node;
    root.interval = *start_interval;
    root.time = request_.start_time;
    consume_waypoints(root);
    push(root);

    const bool rest_anywhere = request_.waypoints.empty();
    int best_rest = -1;
    double best_rest_cost = kForever;

    while (!open_.empty()) {
      const auto [priority, index] = open_.top();
      open_.pop();
      const SearchState state = states_[static_cast<std::size_t>(index)];
      if (closed_[static_cast<std::size_t>(index)]) {
        continue;
      }
      closed_[static_cast<std::size_t>(index)] = true;

      const bool can_stay =
        intervals(roadmap_.node_resource(state.node))[state.interval].end == kForever;
      if (rest_anywhere) {
        if (priority >= best_rest_cost) {
          break;  // nothing cheaper can follow
        }
        if (can_stay) {
          if (state.parent < 0) {
            return build(index);  // staying put is always preferred
          }
          const double cost = state.time +
            (roadmap_.node(state.node).parking ? 0.0 : config_.rest_penalty);
          if (cost < best_rest_cost) {
            best_rest_cost = cost;
            best_rest = index;
          }
        }
      } else if (state.stage == request_.waypoints.size()) {
        return build(index);  // at the last waypoint, with the node free for good
      }

      if (config_.max_expansions > 0 && expansions_ >= config_.max_expansions) {
        break;
      }
      ++expansions_;
      expand(index, state);
    }
    if (best_rest < 0) {
      return std::nullopt;
    }
    return build(best_rest);
  }

  std::size_t expansions() const noexcept {return expansions_;}

private:
  const std::vector<TimeInterval> & intervals(ResourceId resource)
  {
    const std::size_t index = static_cast<std::size_t>(resource);
    if (!known_[index]) {
      free_[index] = (private_table_ ? *private_table_ : table_).free_intervals(
        resource, request_.robot);
      known_[index] = true;
    }
    return free_[index];
  }

  /// Visit the waypoints that are at this node, if the node stays free long
  /// enough to dwell there. The last waypoint is where the robot ends: it
  /// counts as visited only if the robot can stay for good. (If it cannot,
  /// the robot may pass through and come back later.)
  void consume_waypoints(SearchState & state)
  {
    state.stage_on_arrival = state.stage;
    const double until = intervals(roadmap_.node_resource(state.node))[state.interval].end;
    while (state.stage < request_.waypoints.size() &&
      request_.waypoints[state.stage].node == state.node)
    {
      const bool last = state.stage + 1 == request_.waypoints.size();
      const double leave = state.time + request_.waypoints[state.stage].dwell;
      if (last ? until != kForever : leave > until + kEpsilon) {
        break;
      }
      state.time = leave;
      ++state.stage;
    }
  }

  double estimate(const SearchState & state) const
  {
    if (state.stage >= request_.waypoints.size()) {
      return 0.0;
    }
    return to_waypoint_[state.stage][static_cast<std::size_t>(state.node)] + after_[state.stage];
  }

  void push(const SearchState & state)
  {
    const double remaining = estimate(state);
    if (remaining == kForever) {
      return;  // the next waypoint cannot be reached from here at all
    }
    const auto key = std::make_tuple(state.node, state.interval, state.stage);
    const auto found = best_.find(key);
    if (found != best_.end()) {
      SearchState & known = states_[static_cast<std::size_t>(found->second)];
      if (state.time >= known.time - kEpsilon || closed_[static_cast<std::size_t>(found->second)]) {
        return;
      }
      known = state;
      open_.push({state.time + remaining, found->second});
      return;
    }
    const int index = static_cast<int>(states_.size());
    states_.push_back(state);
    closed_.push_back(false);
    best_[key] = index;
    open_.push({state.time + remaining, index});
  }

  void expand(int index, const SearchState & state)
  {
    const double node_free_until =
      intervals(roadmap_.node_resource(state.node))[state.interval].end;
    for (const int edge_index : roadmap_.out_edges(state.node)) {
      const MapEdge & edge = roadmap_.edge(edge_index);
      const double duration = travel_time(edge, request_.speed, config_);
      // Copies: intervals() may grow the cache while we iterate.
      const std::vector<TimeInterval> edge_free = intervals(roadmap_.edge_resource(edge_index));
      const std::vector<TimeInterval> target_free = intervals(roadmap_.node_resource(edge.to));
      for (const TimeInterval & edge_interval : edge_free) {
        if (edge_interval.end <= state.time) {
          continue;
        }
        for (std::size_t j = 0; j < target_free.size(); ++j) {
          const TimeInterval & target = target_free[j];
          if (target.end <= state.time) {
            continue;
          }
          // The move takes the edge and the target node when it starts and
          // keeps the current node until it ends.
          const double start = std::max({state.time, edge_interval.start, target.start});
          const double end = start + duration;
          if (end > std::min({node_free_until, edge_interval.end, target.end}) + kEpsilon) {
            continue;
          }
          SearchState next;
          next.node = edge.to;
          next.interval = j;
          next.stage = state.stage;
          next.time = end;
          next.parent = index;
          next.move = PlannedMove{edge_index, edge.from, edge.to, start, end};
          consume_waypoints(next);
          push(next);
        }
      }
    }
  }

  Plan build(int index) const
  {
    std::vector<int> chain;
    for (int at = index; at >= 0; at = states_[static_cast<std::size_t>(at)].parent) {
      chain.push_back(at);
    }
    std::reverse(chain.begin(), chain.end());

    Plan plan;
    plan.waypoint_after_moves.assign(request_.waypoints.size(), 0);
    for (const int at : chain) {
      const SearchState & state = states_[static_cast<std::size_t>(at)];
      if (state.parent >= 0) {
        plan.moves.push_back(state.move);
      }
      for (std::size_t k = state.stage_on_arrival; k < state.stage; ++k) {
        plan.waypoint_after_moves[k] = plan.moves.size();
      }
    }
    plan.finish_time = states_[static_cast<std::size_t>(index)].time;
    return plan;
  }

  const Roadmap & roadmap_;
  const ReservationTable & table_;
  const RouteRequest & request_;
  const PlannerConfig & config_;

  std::size_t expansions_{0};
  std::optional<ReservationTable> private_table_;
  std::vector<std::vector<TimeInterval>> free_;
  std::vector<bool> known_;
  std::vector<std::vector<double>> to_waypoint_;
  std::vector<double> after_;

  std::vector<SearchState> states_;
  std::vector<bool> closed_;
  std::map<std::tuple<int, std::size_t, std::size_t>, int> best_;
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open_;
};

/// A resource held by a robot during a planned interval.
struct Holding
{
  ResourceId resource{0};
  double start{0.0};
  double end{kForever};
};

/// Everything a robot holds according to its request and (possibly empty) plan.
std::vector<Holding> holdings(
  const Roadmap & roadmap, const AgentRequest & request, const Plan * plan)
{
  std::vector<Holding> result;
  if (request.immobile) {
    for (const ResourceId resource : request.held) {
      result.push_back(Holding{resource, 0.0, kForever});
    }
    return result;
  }
  for (const ResourceId resource : request.held) {
    result.push_back(Holding{resource, 0.0, request.start_time});
  }
  const bool moves = plan != nullptr && !plan->moves.empty();
  result.push_back(
    Holding{roadmap.node_resource(request.start_node), request.start_time,
      moves ? plan->moves.front().end : kForever});
  if (moves) {
    for (std::size_t k = 0; k < plan->moves.size(); ++k) {
      const PlannedMove & move = plan->moves[k];
      result.push_back(Holding{roadmap.edge_resource(move.edge), move.start, move.end});
      result.push_back(
        Holding{roadmap.node_resource(move.to), move.start,
          k + 1 < plan->moves.size() ? plan->moves[k + 1].end : kForever});
    }
  }
  return result;
}

/// What a robot holds before any plan for it exists: everything it has now
/// until it has finished its commitments, and then its node for as long as
/// it takes to drive off along its shortest edge. Nobody can be in that node
/// sooner than that, whatever the robot does next; saying so up front keeps
/// the others from planning into a place its occupant has had no time to leave.
void reserve_present(
  ReservationTable & table, const Roadmap & roadmap, const AgentRequest & request,
  const PlannerConfig & config)
{
  for (const ResourceId resource : request.held) {
    table.reserve(resource, 0.0, request.start_time, request.robot);
  }
  double leave = kForever;
  for (const int edge : roadmap.out_edges(request.start_node)) {
    leave = std::min(leave, travel_time(roadmap.edge(edge), request.speed, config));
  }
  table.reserve(
    roadmap.node_resource(request.start_node), request.start_time,
    request.start_time + leave, request.robot);
}

void reserve_all(ReservationTable & table, const std::vector<Holding> & items, int robot)
{
  for (const Holding & item : items) {
    table.reserve(item.resource, item.start, item.end, robot);
  }
}

}  // namespace

double travel_time(const MapEdge & edge, double robot_speed, const PlannerConfig & config)
{
  double speed = robot_speed > 0.0 ? robot_speed : config.default_speed;
  if (edge.max_speed > 0.0) {
    speed = std::min(speed, edge.max_speed);
  }
  if (!(speed > 0.0)) {
    speed = 1.0;
  }
  return std::max(edge.length / speed + config.node_overhead, 1e-3);
}

std::vector<double> travel_times_to(
  const Roadmap & roadmap, int goal, double speed, const PlannerConfig & config)
{
  std::vector<double> best(static_cast<std::size_t>(roadmap.node_count()), kForever);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  best[static_cast<std::size_t>(goal)] = 0.0;
  open.push({0.0, goal});
  while (!open.empty()) {
    const auto [time, node] = open.top();
    open.pop();
    if (time > best[static_cast<std::size_t>(node)]) {
      continue;
    }
    for (const int edge_index : roadmap.in_edges(node)) {
      const MapEdge & edge = roadmap.edge(edge_index);
      const double candidate = time + travel_time(edge, speed, config);
      if (candidate < best[static_cast<std::size_t>(edge.from)]) {
        best[static_cast<std::size_t>(edge.from)] = candidate;
        open.push({candidate, edge.from});
      }
    }
  }
  return best;
}

std::optional<Plan> plan_route(
  const Roadmap & roadmap, const ReservationTable & table, const RouteRequest & request,
  const PlannerConfig & config, std::size_t * expansions)
{
  RouteSearch search(roadmap, table, request, config);
  std::optional<Plan> plan = search.run();
  if (expansions != nullptr) {
    *expansions += search.expansions();
  }
  return plan;
}

namespace
{

RouteRequest route_request(const AgentRequest & request)
{
  RouteRequest route;
  route.robot = request.robot;
  route.start_node = request.start_node;
  route.start_time = request.start_time;
  route.waypoints = request.waypoints;
  route.speed = request.speed;
  return route;
}

/// The robots that can move, in the order in which pass number `variant` of
/// stage 1 plans them. Variant 0 is the natural order: by priority. Variant 1
/// keeps the robots with tasks in that order and reverses the idle ones
/// (which of two idle robots moves first often decides whether both can get
/// out of the way). Later variants are fixed shuffles of all of them.
std::vector<std::size_t> planning_order(
  const std::vector<AgentRequest> & requests, std::size_t variant)
{
  std::vector<std::size_t> order;
  for (std::size_t i = 0; i < requests.size(); ++i) {
    if (!requests[i].immobile) {
      order.push_back(i);
    }
  }
  std::stable_sort(
    order.begin(), order.end(), [&requests](std::size_t a, std::size_t b) {
      return requests[a].priority < requests[b].priority;
    });
  if (variant == 1) {
    const auto first_idle = std::find_if(
      order.begin(), order.end(),
      [&requests](std::size_t i) {return requests[i].waypoints.empty();});
    std::reverse(first_idle, order.end());
  } else if (variant > 1) {
    // A small linear congruential generator: the same shuffle every time.
    std::uint64_t state = 0x9E3779B97F4A7C15ull * (variant + 1) + requests.size();
    for (std::size_t i = order.size(); i > 1; --i) {
      state = state * 6364136223846793005ull + 1442695040888963407ull;
      std::swap(order[i - 1], order[static_cast<std::size_t>((state >> 33) % i)]);
    }
  }
  return order;
}

bool overlap(const Holding & a, const Holding & b)
{
  return a.start < b.end - kEpsilon && b.start < a.end - kEpsilon;
}

/// One pass of prioritized planning over the robots in a given order, with
/// "pushing": a robot that finds no route may ask resting idle robots that
/// are in its way to move on.
class OrderedPass
{
public:
  static constexpr int kMovedUp = 1;
  static constexpr int kSetAside = 2;
  static constexpr int kSetAsideMovedUp = 3;
  static constexpr int kStays = 4;

  OrderedPass(
    const Roadmap & roadmap, const ConflictTable & conflicts,
    const std::vector<AgentRequest> & requests, const PlannerConfig & config,
    const std::vector<int> & step)
  : roadmap_(roadmap), conflicts_(conflicts), requests_(requests), config_(config),
    step_(step), plans_(requests.size()), placed_(requests.size(), false) {}

  /// Plan robot `i` from scratch. False if it has no route.
  bool place(std::size_t i)
  {
    RouteRequest route = route_request(requests_[i]);
    if (idle_like(i)) {
      route.waypoints.clear();
    }
    return place(i, route, Plan{}, 0);
  }

  const Plan & plan(std::size_t i) const {return plans_[i];}

private:
  /// Has nothing it must reach in this pass: may be asked to move on.
  bool idle_like(std::size_t i) const
  {
    return requests_[i].waypoints.empty() || step_[i] >= kSetAside;
  }

  /// Seconds a robot needs at least to leave `node`.
  double time_to_leave(int node, double speed) const
  {
    double best = kForever;
    for (const int edge : roadmap_.out_edges(node)) {
      best = std::min(best, travel_time(roadmap_.edge(edge), speed, config_));
    }
    return best;
  }

  std::vector<Holding> holdings_of(std::size_t j) const
  {
    return holdings(roadmap_, requests_[j], &plans_[j]);
  }

  /// Everything the robots other than `except` hold or will hold. With
  /// `soft`, an idle robot's final rest is cut down to the time it needs to
  /// drive off again: "it could have moved on by then, if asked".
  ReservationTable table_without(std::size_t except, bool soft) const
  {
    ReservationTable table(conflicts_);
    for (std::size_t j = 0; j < requests_.size(); ++j) {
      if (j == except) {
        continue;
      }
      if (requests_[j].immobile || step_[j] == kStays) {
        reserve_all(table, holdings(roadmap_, requests_[j], nullptr), requests_[j].robot);
      } else if (!placed_[j]) {
        reserve_present(table, roadmap_, requests_[j], config_);
      } else {
        std::vector<Holding> items = holdings_of(j);
        if (soft && idle_like(j)) {
          const int rest = plans_[j].moves.empty() ?
            requests_[j].start_node : plans_[j].moves.back().to;
          const double since = plans_[j].moves.empty() ?
            requests_[j].start_time : plans_[j].moves.back().end;
          for (Holding & item : items) {
            if (item.end == kForever) {
              item.end = since + time_to_leave(rest, requests_[j].speed);
            }
          }
        }
        reserve_all(table, items, requests_[j].robot);
      }
    }
    return table;
  }

  /// Find a route for robot `i` that continues `prefix`, and make room for it
  /// if need be.
  bool place(std::size_t i, const RouteRequest & route, const Plan & prefix, int depth)
  {
    const auto adopt = [&](const Plan & found) {
        Plan whole = prefix;
        whole.moves.insert(whole.moves.end(), found.moves.begin(), found.moves.end());
        if (prefix.moves.empty()) {
          whole.waypoint_after_moves = found.waypoint_after_moves;
        }
        whole.finish_time = found.finish_time;
        plans_[i] = std::move(whole);
        placed_[i] = true;
      };

    if (const auto found = plan_route(roadmap_, table_without(i, false), route, config_)) {
      adopt(*found);
      return true;
    }
    // No route around everything. Is there one if resting idle robots move on?
    if (depth >= kMaxPushDepth) {
      return false;
    }
    const auto found = plan_route(roadmap_, table_without(i, true), route, config_);
    if (!found) {
      return false;
    }
    const std::vector<Plan> saved_plans = plans_;
    const std::vector<bool> saved_placed = placed_;
    adopt(*found);

    // Those whose rest this route runs into must find another place, from
    // where they are resting. They may push others in turn.
    for (std::size_t j = 0; j < requests_.size(); ++j) {
      if (j == i || !placed_[j] || !idle_like(j) || requests_[j].immobile ||
        step_[j] == kStays)
      {
        continue;
      }
      bool collides = false;
      const std::vector<Holding> mine = holdings_of(i);
      for (const Holding & theirs : holdings_of(j)) {
        for (const Holding & item : mine) {
          collides = collides ||
            (conflicts_.conflict(item.resource, theirs.resource) && overlap(item, theirs));
        }
      }
      if (!collides) {
        continue;
      }
      RouteRequest onward;
      onward.robot = requests_[j].robot;
      onward.speed = requests_[j].speed;
      onward.start_node = plans_[j].moves.empty() ?
        requests_[j].start_node : plans_[j].moves.back().to;
      onward.start_time = plans_[j].moves.empty() ?
        requests_[j].start_time : plans_[j].moves.back().end;
      const Plan so_far = plans_[j];
      placed_[j] = false;
      if (!place(j, onward, so_far, depth + 1)) {
        plans_ = saved_plans;
        placed_ = saved_placed;
        return false;
      }
    }
    return true;
  }

  static constexpr int kMaxPushDepth = 4;

  const Roadmap & roadmap_;
  const ConflictTable & conflicts_;
  const std::vector<AgentRequest> & requests_;
  const PlannerConfig & config_;
  const std::vector<int> & step_;
  std::vector<Plan> plans_;
  std::vector<bool> placed_;
};

FleetPlan plan_in_order(
  const Roadmap & roadmap, const ConflictTable & conflicts,
  const std::vector<AgentRequest> & requests, const PlannerConfig & config,
  std::vector<std::size_t> order)
{
  const std::size_t count = requests.size();
  FleetPlan result;
  result.agents.resize(count);

  // Repair. A robot that finds no route climbs one step at a time:
  //   1. it is moved to the front (it may only have been boxed in by robots
  //      planned before it);
  //   2. its task is set aside for this round and it is planned like an idle
  //      robot, free to go wherever it is out of the way;
  //   3. the same, from the front;
  //   4. it stays where it is for good, and the others plan around it.
  // Each failure moves exactly one robot one step, so this ends after at most
  // 4 * count + 1 passes, and what comes out is always a valid plan.
  std::vector<int> step(count, 0);

  while (true) {
    ++result.attempts;
    OrderedPass pass(roadmap, conflicts, requests, config, step);
    std::optional<std::size_t> failed;
    for (const std::size_t i : order) {
      if (step[i] != OrderedPass::kStays && !pass.place(i)) {
        failed = i;
        break;
      }
    }
    if (!failed) {
      for (std::size_t i = 0; i < count; ++i) {
        result.agents[i].robot = requests[i].robot;
        if (requests[i].immobile) {
          result.agents[i].outcome = AgentOutcome::Immobile;
        } else if (step[i] >= OrderedPass::kSetAside) {
          result.agents[i].outcome = AgentOutcome::Blocked;
          result.agents[i].plan = pass.plan(i);
        } else {
          result.agents[i].outcome = AgentOutcome::Planned;
          result.agents[i].plan = pass.plan(i);
        }
      }
      return result;
    }

    ++step[*failed];
    if (step[*failed] == OrderedPass::kMovedUp ||
      step[*failed] == OrderedPass::kSetAsideMovedUp)
    {
      order.erase(std::find(order.begin(), order.end(), *failed));
      order.insert(order.begin(), *failed);
    }
  }
}

/// Last step of both planners: a robot reported as Blocked gets one more try
/// at its real task, now that everybody else's plan is known. This catches
/// the robot whose task was only blocked by a plan that was later withdrawn.
void retry_blocked(
  const Roadmap & roadmap, const ConflictTable & conflicts,
  const std::vector<AgentRequest> & requests, const PlannerConfig & config, FleetPlan & plan)
{
  for (std::size_t i = 0; i < requests.size(); ++i) {
    if (plan.agents[i].outcome != AgentOutcome::Blocked || requests[i].waypoints.empty()) {
      continue;
    }
    ReservationTable table(conflicts);
    for (std::size_t j = 0; j < requests.size(); ++j) {
      if (j != i) {
        reserve_all(
          table, holdings(roadmap, requests[j], &plan.agents[j].plan), requests[j].robot);
      }
    }
    std::optional<Plan> better = plan_route(roadmap, table, route_request(requests[i]), config);
    if (better) {
      plan.agents[i].outcome = AgentOutcome::Planned;
      plan.agents[i].plan = std::move(*better);
    }
  }
}

// ------------------------------------------------ conflict-based search ----

struct Collision
{
  std::size_t first{0};
  std::size_t second{0};
  Holding first_holding{};
  Holding second_holding{};
  double time{0.0};  ///< When the overlap begins.
};

struct SearchNode
{
  /// Per robot: holdings of other robots that it must treat as reserved.
  std::vector<std::vector<Holding>> give_way;
  std::vector<Plan> plans;
  double cost{0.0};
  std::size_t collisions{0};
  std::optional<Collision> earliest;
};

struct WorseNode
{
  bool operator()(const SearchNode & a, const SearchNode & b) const
  {
    if (a.cost != b.cost) {
      return a.cost > b.cost;
    }
    return a.collisions > b.collisions;
  }
};

class FleetSearch
{
public:
  FleetSearch(
    const Roadmap & roadmap, const ConflictTable & conflicts,
    const std::vector<AgentRequest> & requests, const PlannerConfig & config)
  : roadmap_(roadmap), conflicts_(conflicts), requests_(requests), config_(config),
    set_aside_(requests.size(), false), stays_(requests.size(), false),
    weight_(requests.size(), 1.0)
  {
    // A second of a high-priority robot counts for more than a second of a
    // low-priority one (between 2 and 1), so that when somebody has to give
    // way, the search prefers it to be the less urgent robot.
    std::vector<std::size_t> order(requests.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
      order[i] = i;
    }
    std::stable_sort(
      order.begin(), order.end(), [&requests](std::size_t a, std::size_t b) {
        return requests[a].priority < requests[b].priority;
      });
    for (std::size_t rank = 0; rank < order.size(); ++rank) {
      weight_[order[rank]] =
        2.0 - static_cast<double>(rank) / static_cast<double>(order.size());
    }
    // Long enough for a resting robot to drive off along any one edge.
    for (int edge = 0; edge < roadmap.edge_count(); ++edge) {
      patience_ = std::max(patience_, travel_time(roadmap.edge(edge), 0.0, config));
    }
  }

  /// The collision-free plan found within the budget, if any.
  std::optional<FleetPlan> run()
  {
    std::optional<SearchNode> root = make_root();
    if (!root) {
      return std::nullopt;
    }
    std::priority_queue<SearchNode, std::vector<SearchNode>, WorseNode> open;
    open.push(std::move(*root));

    while (!open.empty() && expanded_ < config_.search_budget && work_ < config_.search_work) {
      SearchNode node = open.top();
      open.pop();
      ++expanded_;
      if (!node.earliest) {
        return to_fleet_plan(node);
      }
      // Two ways out of this collision: either robot treats what the other
      // one holds there as reserved, and plans again.
      const Collision & collision = *node.earliest;
      const std::pair<std::size_t, Holding> options[2] = {
        {collision.first, collision.second_holding},
        {collision.second, collision.first_holding}};
      for (const auto & [robot, other] : options) {
        Holding avoid = other;
        if (avoid.end == kForever) {
          // The other robot is resting there. "Never go there" would be too
          // much to ask: it may well move on if given the time. So this robot
          // only stays clear for a while; if the other one is still there
          // afterwards, the same choice comes up again, later.
          avoid.start = collision.time;
          avoid.end = collision.time + patience_;
        }
        SearchNode child = node;
        child.give_way[robot].push_back(avoid);
        // First try the generous way of giving way: around everything the
        // others currently plan to do. If that works, this robot is out of
        // every collision at once (an idle robot, for instance, goes straight
        // to a place nobody needs). Only if it does not is the robot asked to
        // avoid just this one thing, which may lead to further collisions
        // that are sorted out further down the tree.
        std::vector<Holding> everything = child.give_way[robot];
        for (std::size_t other_robot = 0; other_robot < requests_.size(); ++other_robot) {
          if (other_robot != robot && moves(other_robot)) {
            const std::vector<Holding> theirs =
              holdings(roadmap_, requests_[other_robot], &node.plans[other_robot]);
            everything.insert(everything.end(), theirs.begin(), theirs.end());
          }
        }
        std::optional<Plan> plan = plan_one(robot, everything);
        if (!plan) {
          plan = plan_one(robot, child.give_way[robot]);
        }
        if (!plan) {
          continue;  // this robot cannot give way here
        }
        child.plans[robot] = std::move(*plan);
        evaluate(child);
        open.push(std::move(child));
      }
    }
    return std::nullopt;
  }

  std::size_t expanded() const noexcept {return expanded_;}

private:
  bool moves(std::size_t i) const {return !requests_[i].immobile && !stays_[i];}

  ReservationTable base_table() const
  {
    ReservationTable table(conflicts_);
    for (std::size_t i = 0; i < requests_.size(); ++i) {
      if (moves(i)) {
        reserve_present(table, roadmap_, requests_[i], config_);
      } else {
        reserve_all(table, holdings(roadmap_, requests_[i], nullptr), requests_[i].robot);
      }
    }
    return table;
  }

  std::optional<Plan> plan_one(std::size_t i, const std::vector<Holding> & give_way)
  {
    RouteRequest route = route_request(requests_[i]);
    if (set_aside_[i]) {
      route.waypoints.clear();
    }
    for (const Holding & item : give_way) {
      route.give_way.push_back({item.resource, TimeInterval{item.start, item.end}});
    }
    // One route may use what is left of the work budget, no more.
    PlannerConfig limited = config_;
    const std::size_t left = config_.search_work > work_ ? config_.search_work - work_ : 1;
    limited.max_expansions =
      config_.max_expansions == 0 ? left : std::min(config_.max_expansions, left);
    return plan_route(roadmap_, base_, route, limited, &work_);
  }

  std::optional<SearchNode> make_root()
  {
    // Two robots that must end at the same place (or at places too close to
    // each other) cannot both be served, whatever the others do. Deciding
    // that here saves the search from discovering it the slow way: the one
    // with the lower priority has its task set aside for this round.
    std::vector<std::size_t> by_priority;
    for (std::size_t i = 0; i < requests_.size(); ++i) {
      if (moves(i) && !requests_[i].waypoints.empty()) {
        by_priority.push_back(i);
      }
    }
    std::stable_sort(
      by_priority.begin(), by_priority.end(), [this](std::size_t a, std::size_t b) {
        return requests_[a].priority < requests_[b].priority;
      });
    for (std::size_t a = 0; a < by_priority.size(); ++a) {
      const std::size_t i = by_priority[a];
      if (set_aside_[i]) {
        continue;
      }
      for (std::size_t b = a + 1; b < by_priority.size(); ++b) {
        const std::size_t j = by_priority[b];
        if (conflicts_.conflict(
            roadmap_.node_resource(requests_[i].waypoints.back().node),
            roadmap_.node_resource(requests_[j].waypoints.back().node)))
        {
          set_aside_[j] = true;
        }
      }
    }

    // Every robot planned on its own. One that has no route even then cannot
    // be helped by the others giving way: its task is set aside, and if it
    // cannot even be planned as an idle robot, it stays.
    for (std::size_t pass = 0; pass <= 2 * requests_.size(); ++pass) {
      base_ = base_table();
      SearchNode root;
      root.give_way.resize(requests_.size());
      root.plans.resize(requests_.size());
      bool again = false;
      for (std::size_t i = 0; i < requests_.size() && !again; ++i) {
        if (!moves(i)) {
          continue;
        }
        std::optional<Plan> plan = plan_one(i, {});
        if (plan) {
          root.plans[i] = std::move(*plan);
        } else if (!set_aside_[i]) {
          set_aside_[i] = true;
          again = true;
        } else {
          stays_[i] = true;
          again = true;
        }
      }
      if (!again) {
        evaluate(root);
        return root;
      }
    }
    return std::nullopt;
  }

  void evaluate(SearchNode & node) const
  {
    node.cost = 0.0;
    node.collisions = 0;
    node.earliest.reset();
    std::vector<std::vector<Holding>> all(requests_.size());
    for (std::size_t i = 0; i < requests_.size(); ++i) {
      if (moves(i)) {
        all[i] = holdings(roadmap_, requests_[i], &node.plans[i]);
        node.cost += weight_[i] * node.plans[i].finish_time;
      }
    }
    for (std::size_t i = 0; i < all.size(); ++i) {
      for (std::size_t j = i + 1; j < all.size(); ++j) {
        for (const Holding & a : all[i]) {
          for (const Holding & b : all[j]) {
            if (!conflicts_.conflict(a.resource, b.resource) ||
              !(a.start < b.end - kEpsilon && b.start < a.end - kEpsilon))
            {
              continue;
            }
            ++node.collisions;
            const double time = std::max(a.start, b.start);
            if (!node.earliest || time < node.earliest->time) {
              node.earliest = Collision{i, j, a, b, time};
            }
          }
        }
      }
    }
  }

  FleetPlan to_fleet_plan(const SearchNode & node) const
  {
    FleetPlan plan;
    plan.agents.resize(requests_.size());
    for (std::size_t i = 0; i < requests_.size(); ++i) {
      plan.agents[i].robot = requests_[i].robot;
      if (requests_[i].immobile) {
        plan.agents[i].outcome = AgentOutcome::Immobile;
      } else if (stays_[i]) {
        plan.agents[i].outcome = AgentOutcome::Blocked;
      } else {
        plan.agents[i].outcome =
          set_aside_[i] ? AgentOutcome::Blocked : AgentOutcome::Planned;
        plan.agents[i].plan = node.plans[i];
      }
    }
    plan.from_search = true;
    return plan;
  }

  const Roadmap & roadmap_;
  const ConflictTable & conflicts_;
  const std::vector<AgentRequest> & requests_;
  const PlannerConfig & config_;
  std::vector<bool> set_aside_;  ///< Planned as an idle robot this round.
  std::vector<bool> stays_;      ///< Not planned at all; keeps its node.
  std::vector<double> weight_;   ///< Cost of one second of each robot's time.
  double patience_{1.0};         ///< How long a robot stays clear of a resting one.
  ReservationTable base_{conflicts_};
  std::size_t expanded_{0};
  std::size_t work_{0};
};

std::size_t blocked_count(const FleetPlan & plan)
{
  std::size_t count = 0;
  for (const AgentPlan & agent : plan.agents) {
    count += agent.outcome == AgentOutcome::Blocked ? 1 : 0;
  }
  return count;
}

FleetPlan plan_prioritized(
  const Roadmap & roadmap, const ConflictTable & conflicts,
  const std::vector<AgentRequest> & requests, const PlannerConfig & config)
{
  FleetPlan best;
  std::size_t passes = 0;
  for (std::size_t variant = 0; variant < std::max<std::size_t>(1, config.order_attempts);
    ++variant)
  {
    FleetPlan candidate =
      plan_in_order(roadmap, conflicts, requests, config, planning_order(requests, variant));
    retry_blocked(roadmap, conflicts, requests, config, candidate);
    passes += candidate.attempts;
    if (variant == 0 || blocked_count(candidate) < blocked_count(best)) {
      best = std::move(candidate);
    }
    if (blocked_count(best) == 0) {
      break;
    }
  }
  best.attempts = passes;
  return best;
}

}  // namespace

FleetPlan plan_fleet(
  const Roadmap & roadmap, const ConflictTable & conflicts,
  const std::vector<AgentRequest> & requests, const PlannerConfig & config)
{
  FleetPlan prioritized = plan_prioritized(roadmap, conflicts, requests, config);
  if (blocked_count(prioritized) == 0 || config.search_budget == 0) {
    return prioritized;
  }
  FleetSearch search(roadmap, conflicts, requests, config);
  std::optional<FleetPlan> found = search.run();
  prioritized.search_nodes = search.expanded();
  if (found) {
    retry_blocked(roadmap, conflicts, requests, config, *found);
    if (blocked_count(*found) < blocked_count(prioritized)) {
      found->attempts = prioritized.attempts;
      found->search_nodes = search.expanded();
      return *found;
    }
  }
  return prioritized;
}

std::string validate_fleet_plan(
  const Roadmap & roadmap, const ConflictTable & conflicts,
  const std::vector<AgentRequest> & requests, const FleetPlan & plan)
{
  if (plan.agents.size() != requests.size()) {
    return "plan and requests differ in size";
  }

  // 1. Each plan is a connected walk with times that only move forward.
  for (std::size_t i = 0; i < requests.size(); ++i) {
    const AgentRequest & request = requests[i];
    const AgentPlan & agent = plan.agents[i];
    const std::string who = "robot " + std::to_string(request.robot);
    if (agent.outcome == AgentOutcome::Immobile && !agent.plan.moves.empty()) {
      return who + ": cannot move but has moves";
    }
    int at = request.start_node;
    double clock = request.start_time;
    for (const PlannedMove & move : agent.plan.moves) {
      if (move.edge < 0 || move.edge >= roadmap.edge_count()) {
        return who + ": unknown edge";
      }
      const MapEdge & edge = roadmap.edge(move.edge);
      if (edge.from != at || move.from != at || edge.to != move.to) {
        return who + ": moves do not form a path at edge " + edge.id;
      }
      if (move.start < clock - kEpsilon) {
        return who + ": edge " + edge.id + " starts before the robot is there";
      }
      if (!(move.end > move.start)) {
        return who + ": edge " + edge.id + " takes no time";
      }
      at = move.to;
      clock = move.end;
    }
    if (agent.outcome != AgentOutcome::Planned) {
      continue;  // not on its task: no waypoints to check
    }
    if (agent.plan.waypoint_after_moves.size() != request.waypoints.size()) {
      return who + ": waypoint bookkeeping has the wrong size";
    }
    std::size_t previous = 0;
    for (std::size_t k = 0; k < request.waypoints.size(); ++k) {
      const std::size_t after = agent.plan.waypoint_after_moves[k];
      if (after < previous || after > agent.plan.moves.size()) {
        return who + ": waypoints are not visited in order";
      }
      const int node = after == 0 ? request.start_node : agent.plan.moves[after - 1].to;
      if (node != request.waypoints[k].node) {
        return who + ": waypoint " + std::to_string(k) + " is not at its node";
      }
      previous = after;
    }
    if (!request.waypoints.empty() &&
      agent.plan.waypoint_after_moves.back() != agent.plan.moves.size())
    {
      return who + ": does not end at its last waypoint";
    }
  }

  // 2. No two robots hold conflicting resources at the same time.
  std::vector<std::vector<Holding>> all;
  for (std::size_t i = 0; i < requests.size(); ++i) {
    all.push_back(holdings(roadmap, requests[i], &plan.agents[i].plan));
  }
  for (std::size_t i = 0; i < all.size(); ++i) {
    for (std::size_t j = i + 1; j < all.size(); ++j) {
      for (const Holding & a : all[i]) {
        for (const Holding & b : all[j]) {
          if (!conflicts.conflict(a.resource, b.resource)) {
            continue;
          }
          if (a.start < b.end - kEpsilon && b.start < a.end - kEpsilon) {
            return "robot " + std::to_string(requests[i].robot) + " holds " +
                   roadmap.resource_name(a.resource) + " while robot " +
                   std::to_string(requests[j].robot) + " holds " +
                   roadmap.resource_name(b.resource);
          }
        }
      }
    }
  }
  return "";
}

}  // namespace fleet_core
