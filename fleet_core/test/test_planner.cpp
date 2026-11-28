// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Tests of the planner. Every fleet plan produced here is also passed through
// validate_fleet_plan(), an independent check of the one property that
// matters: no two robots ever hold conflicting resources at the same time.

#include <gtest/gtest.h>

#include <random>
#include <string>
#include <vector>

#include "fleet_core/planner.hpp"
#include "maps.hpp"

using namespace fleet_core;  // NOLINT(build/namespaces)

namespace
{

/// With these settings every 2 m edge of the test maps takes exactly 3 s.
PlannerConfig config()
{
  PlannerConfig result;
  result.default_speed = 1.0;
  result.node_overhead = 1.0;
  return result;
}

int node(const Roadmap & map, const std::string & id) {return *map.find_node(id);}

std::string path_of(const Roadmap & map, int start, const Plan & plan)
{
  std::string path = map.node(start).id;
  for (const PlannedMove & move : plan.moves) {
    path += ">" + map.node(move.to).id;
  }
  return path;
}

AgentRequest agent(
  const Roadmap & map, int robot, const std::string & at, const std::string & goal = "")
{
  AgentRequest request;
  request.robot = robot;
  request.start_node = node(map, at);
  request.held = {map.node_resource(request.start_node)};
  if (!goal.empty()) {
    request.waypoints = {Waypoint{node(map, goal), 0.0}};
  }
  request.priority = robot;
  return request;
}

}  // namespace

// ------------------------------------------------------------- one robot ----

class RouteTest : public ::testing::Test
{
protected:
  RouteRequest request(const std::string & from, const std::string & to)
  {
    RouteRequest result;
    result.robot = 1;
    result.start_node = node(map, from);
    result.waypoints = {Waypoint{node(map, to), 0.0}};
    return result;
  }
  ResourceId edge(const std::string & id) {return map.edge_resource(*map.find_edge(id));}
  ResourceId at(const std::string & id) {return map.node_resource(node(map, id));}

  Roadmap map = test_maps::corridor_with_siding();
  ConflictTable conflicts{map, test_maps::kClearance};
  ReservationTable table{conflicts};
};

TEST_F(RouteTest, TravelTimeUsesTheSlowerOfRobotAndEdgeLimit)
{
  MapEdge edge{"e", 0, 1, 6.0, 0.0};
  EXPECT_DOUBLE_EQ(travel_time(edge, 0.0, config()), 7.0);   // default 1 m/s + 1 s
  EXPECT_DOUBLE_EQ(travel_time(edge, 2.0, config()), 4.0);
  edge.max_speed = 0.5;
  EXPECT_DOUBLE_EQ(travel_time(edge, 2.0, config()), 13.0);  // the edge limit wins
}

TEST_F(RouteTest, EmptyMapGivesTheShortestPathWithoutWaiting)
{
  const auto plan = plan_route(map, table, request("A", "E"), config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(path_of(map, node(map, "A"), *plan), "A>B>C>D>E");
  EXPECT_DOUBLE_EQ(plan->moves[0].start, 0.0);
  EXPECT_DOUBLE_EQ(plan->moves[0].end, 3.0);
  EXPECT_DOUBLE_EQ(plan->moves[3].end, 12.0);
  EXPECT_DOUBLE_EQ(plan->finish_time, 12.0);
  ASSERT_EQ(plan->waypoint_after_moves.size(), 1u);
  EXPECT_EQ(plan->waypoint_after_moves[0], 4u);
}

TEST_F(RouteTest, AlreadyAtTheGoalIsAnEmptyPlan)
{
  const auto plan = plan_route(map, table, request("C", "C"), config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_TRUE(plan->moves.empty());
  EXPECT_EQ(plan->waypoint_after_moves[0], 0u);
}

TEST_F(RouteTest, StartTimeDelaysTheWholePlan)
{
  RouteRequest late = request("A", "B");
  late.start_time = 7.5;
  const auto plan = plan_route(map, table, late, config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_DOUBLE_EQ(plan->moves[0].start, 7.5);
  EXPECT_DOUBLE_EQ(plan->finish_time, 10.5);
}

TEST_F(RouteTest, WaitsForAReservationToEnd)
{
  table.reserve(at("C"), 0.0, 20.0, 2);  // somebody stands at C until t = 20
  const auto plan = plan_route(map, table, request("A", "E"), config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(path_of(map, node(map, "A"), *plan), "A>B>C>D>E");
  EXPECT_DOUBLE_EQ(plan->moves[0].start, 0.0);     // A>B is unaffected
  EXPECT_DOUBLE_EQ(plan->moves[1].start, 20.0);    // waits at B for C
  EXPECT_DOUBLE_EQ(plan->finish_time, 29.0);
}

TEST_F(RouteTest, WaitsWhereItDoesNotBlockTheOtherRobot)
{
  // Somebody will stand at B from t = 2 to t = 30. Driving A>B takes until
  // t = 3, so the robot cannot get past in time and must not be at B either.
  table.reserve(at("B"), 2.0, 30.0, 2);
  const auto plan = plan_route(map, table, request("A", "C"), config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_DOUBLE_EQ(plan->moves[0].start, 30.0);  // stays at A
}

TEST_F(RouteTest, ARobotsOwnReservationsDoNotBlockIt)
{
  table.reserve(at("C"), 0.0, kForever, 1);
  const auto plan = plan_route(map, table, request("A", "E"), config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_DOUBLE_EQ(plan->finish_time, 12.0);
}

TEST_F(RouteTest, TakesADetourWhenThatIsFasterThanWaiting)
{
  const Roadmap grid = test_maps::grid(3, 2);
  const ConflictTable grid_conflicts(grid, test_maps::kClearance);
  ReservationTable grid_table(grid_conflicts);
  // The direct way along the bottom row is closed at its middle for 100 s.
  grid_table.reserve(grid.node_resource(node(grid, "1,0")), 0.0, 100.0, 2);

  RouteRequest across;
  across.robot = 1;
  across.start_node = node(grid, "0,0");
  across.waypoints = {Waypoint{node(grid, "2,0"), 0.0}};
  const auto plan = plan_route(grid, grid_table, across, config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(path_of(grid, across.start_node, *plan), "0,0>0,1>1,1>2,1>2,0");
  EXPECT_DOUBLE_EQ(plan->finish_time, 12.0);
}

TEST_F(RouteTest, TheGoalMustBeFreeForGood)
{
  // C is needed by somebody from t = 50 on, for ever. The robot can reach C
  // long before that, but could not stay.
  table.reserve(at("C"), 50.0, kForever, 2);
  EXPECT_FALSE(plan_route(map, table, request("A", "C"), config()).has_value());

  // If the other one leaves again, arriving after it has gone is fine.
  ReservationTable passing(conflicts);
  passing.reserve(at("C"), 4.0, 50.0, 2);
  const auto plan = plan_route(map, passing, request("A", "C"), config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_DOUBLE_EQ(plan->finish_time, 53.0);
}

TEST_F(RouteTest, NoPlanWhenSomebodyIsPlannedOntoTheStartNode)
{
  table.reserve(at("A"), 0.0, 5.0, 2);
  EXPECT_FALSE(plan_route(map, table, request("A", "B"), config()).has_value());
}

TEST_F(RouteTest, NoPlanWhenTheGoalCannotBeReachedAtAll)
{
  Roadmap one_way;
  one_way.add_node("A", 0, 0);
  one_way.add_node("B", 2, 0);
  one_way.add_edge("AB", "A", "B");
  const ConflictTable one_way_conflicts(one_way, test_maps::kClearance);
  const ReservationTable empty(one_way_conflicts);
  RouteRequest back;
  back.start_node = 1;
  back.waypoints = {Waypoint{0, 0.0}};
  EXPECT_FALSE(plan_route(one_way, empty, back, config()).has_value());
}

TEST_F(RouteTest, VisitsWaypointsInOrderAndDwells)
{
  RouteRequest tour;
  tour.robot = 1;
  tour.start_node = node(map, "A");
  tour.waypoints = {
    Waypoint{node(map, "C"), 10.0}, Waypoint{node(map, "S"), 5.0}, Waypoint{node(map, "B"), 0.0}};
  const auto plan = plan_route(map, table, tour, config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(path_of(map, tour.start_node, *plan), "A>B>C>S>C>B");
  ASSERT_EQ(plan->waypoint_after_moves.size(), 3u);
  EXPECT_EQ(plan->waypoint_after_moves[0], 2u);
  EXPECT_EQ(plan->waypoint_after_moves[1], 3u);
  EXPECT_EQ(plan->waypoint_after_moves[2], 5u);
  EXPECT_DOUBLE_EQ(plan->moves[1].end, 6.0);     // at C
  EXPECT_DOUBLE_EQ(plan->moves[2].start, 16.0);  // after dwelling 10 s
  EXPECT_DOUBLE_EQ(plan->moves[3].start, 24.0);  // at S at 19, dwell 5
  EXPECT_DOUBLE_EQ(plan->finish_time, 30.0);
}

TEST_F(RouteTest, AWaypointAtTheStartIsVisitedBeforeLeaving)
{
  RouteRequest tour;
  tour.robot = 1;
  tour.start_node = node(map, "A");
  tour.waypoints = {Waypoint{node(map, "A"), 4.0}, Waypoint{node(map, "B"), 0.0}};
  const auto plan = plan_route(map, table, tour, config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->waypoint_after_moves[0], 0u);
  EXPECT_DOUBLE_EQ(plan->moves[0].start, 4.0);
}

TEST_F(RouteTest, DoesNotDwellWhereItWouldHaveToLeaveTooSoon)
{
  // C is free only until t = 10; dwelling 10 s after arriving at t = 6 does
  // not fit. The robot has to come back when C is free again.
  table.reserve(at("C"), 10.0, 40.0, 2);
  RouteRequest tour;
  tour.robot = 1;
  tour.start_node = node(map, "A");
  tour.waypoints = {Waypoint{node(map, "C"), 10.0}};
  const auto plan = plan_route(map, table, tour, config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_DOUBLE_EQ(plan->finish_time, 53.0);  // enters C at 40, arrives 43, dwells to 53
}

TEST_F(RouteTest, AnIdleRobotStaysWhereItIsIfNobodyNeedsThePlace)
{
  RouteRequest idle;
  idle.robot = 1;
  idle.start_node = node(map, "C");
  const auto plan = plan_route(map, table, idle, config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_TRUE(plan->moves.empty());
}

TEST_F(RouteTest, AnIdleRobotInTheWayGoesToAParkingNode)
{
  // Somebody needs C from t = 20 on. B and D are nearer than the parking
  // nodes, but a robot that must move anyway should end up somewhere useful.
  table.reserve(at("C"), 20.0, kForever, 2);
  RouteRequest idle;
  idle.robot = 1;
  idle.start_node = node(map, "C");
  const auto plan = plan_route(map, table, idle, config());
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(path_of(map, idle.start_node, *plan), "C>S");
}

TEST_F(RouteTest, AnIdleRobotTakesAnyRestingPlaceWhenParkingIsTooFar)
{
  PlannerConfig cheap = config();
  cheap.rest_penalty = 1.0;  // a parking node is worth at most 1 s of driving
  table.reserve(at("C"), 20.0, kForever, 2);
  table.reserve(at("S"), 0.0, kForever, 3);  // the siding is taken
  RouteRequest idle;
  idle.robot = 1;
  idle.start_node = node(map, "C");
  const auto plan = plan_route(map, table, idle, cheap);
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->moves.size(), 1u);  // one step to B or D
}

// -------------------------------------------------------------- the fleet ----

class FleetPlanTest : public ::testing::Test
{
protected:
  FleetPlan plan(const std::vector<AgentRequest> & requests)
  {
    const FleetPlan result = plan_fleet(map, conflicts, requests, config());
    EXPECT_EQ(validate_fleet_plan(map, conflicts, requests, result), "");
    return result;
  }

  Roadmap map = test_maps::corridor_with_siding();
  ConflictTable conflicts{map, test_maps::kClearance};
};

TEST_F(FleetPlanTest, RobotsFollowingEachOtherNeedNoRepair)
{
  const FleetPlan result = plan({agent(map, 1, "B", "E"), agent(map, 2, "A", "D")});
  EXPECT_EQ(result.attempts, 1u);
  EXPECT_TRUE(result.agents[0].outcome == AgentOutcome::Planned);
  EXPECT_TRUE(result.agents[1].outcome == AgentOutcome::Planned);
  EXPECT_EQ(path_of(map, node(map, "A"), result.agents[1].plan), "A>B>C>D");
  // The follower may enter A>B only when the leader has left everything that
  // conflicts with it, i.e. when it has arrived at C (t = 3).
  EXPECT_DOUBLE_EQ(result.agents[1].plan.moves[0].start, 3.0);
}

TEST_F(FleetPlanTest, HeadOnInACorridorIsSolvedWithTheSiding)
{
  const FleetPlan result = plan({agent(map, 1, "A", "E"), agent(map, 2, "E", "A")});
  ASSERT_TRUE(result.agents[0].outcome == AgentOutcome::Planned);
  ASSERT_TRUE(result.agents[1].outcome == AgentOutcome::Planned);
  // Exactly one of them steps aside into S.
  const std::string first = path_of(map, node(map, "A"), result.agents[0].plan);
  const std::string second = path_of(map, node(map, "E"), result.agents[1].plan);
  const bool first_yields = first.find("S") != std::string::npos;
  const bool second_yields = second.find("S") != std::string::npos;
  EXPECT_TRUE(first_yields != second_yields);
}

TEST_F(FleetPlanTest, HeadOnWithoutASidingLeavesSomebodyBlockedButThePlanIsValid)
{
  map = test_maps::corridor();
  conflicts = ConflictTable(map, test_maps::kClearance);
  const FleetPlan result = plan({agent(map, 1, "A", "E"), agent(map, 2, "E", "A")});
  // There is no solution: they cannot swap ends in a single lane. The planner
  // must say so, not loop and not invent one.
  EXPECT_GT(result.attempts, 1u);
  int blocked = 0;
  for (const AgentPlan & item : result.agents) {
    blocked += item.outcome == AgentOutcome::Blocked ? 1 : 0;
  }
  EXPECT_GE(blocked, 1);
  EXPECT_FALSE(result.from_search);  // the search found nothing better either
}

TEST_F(FleetPlanTest, AnIdleRobotIsMovedOutOfTheWay)
{
  // Robot 2 has nothing to do and stands in the middle of the corridor.
  const FleetPlan result = plan({agent(map, 1, "A", "E"), agent(map, 2, "C")});
  ASSERT_TRUE(result.agents[0].outcome == AgentOutcome::Planned);
  ASSERT_TRUE(result.agents[1].outcome == AgentOutcome::Planned);
  EXPECT_EQ(path_of(map, node(map, "A"), result.agents[0].plan), "A>B>C>D>E");
  EXPECT_EQ(path_of(map, node(map, "C"), result.agents[1].plan), "C>S");
}

TEST_F(FleetPlanTest, AnIdleRobotThatIsNotInTheWayStays)
{
  const FleetPlan result = plan({agent(map, 1, "A", "C"), agent(map, 2, "E")});
  EXPECT_TRUE(result.agents[1].plan.moves.empty());
  EXPECT_TRUE(result.agents[1].outcome == AgentOutcome::Planned);
}

TEST_F(FleetPlanTest, ALowerPriorityRobotThatWouldBeBoxedInIsMovedUp)
{
  // Robot 1 (planned first) goes A>...>E straight through D, where robot 2
  // stands and wants to go to A. Planned second, robot 2 would have nowhere
  // to go in time; moved to the front, it plans first and robot 1 fits around it.
  const FleetPlan result = plan({agent(map, 1, "C", "E"), agent(map, 2, "D", "A")});
  EXPECT_TRUE(result.agents[0].outcome == AgentOutcome::Planned);
  EXPECT_TRUE(result.agents[1].outcome == AgentOutcome::Planned);
  EXPECT_GT(result.attempts, 1u);
}

TEST_F(FleetPlanTest, AnImmobileRobotIsPlannedAround)
{
  const Roadmap grid = test_maps::grid(3, 2);
  const ConflictTable grid_conflicts(grid, test_maps::kClearance);
  AgentRequest broken = agent(grid, 2, "1,0");
  broken.immobile = true;
  const std::vector<AgentRequest> requests{agent(grid, 1, "0,0", "2,0"), broken};
  const FleetPlan result = plan_fleet(grid, grid_conflicts, requests, config());
  EXPECT_EQ(validate_fleet_plan(grid, grid_conflicts, requests, result), "");
  EXPECT_TRUE(result.agents[1].outcome == AgentOutcome::Immobile);
  EXPECT_EQ(path_of(grid, node(grid, "0,0"), result.agents[0].plan), "0,0>0,1>1,1>2,1>2,0");
}

TEST_F(FleetPlanTest, AnImmobileRobotInASingleLaneBlocksTheOther)
{
  AgentRequest broken = agent(map, 2, "D");
  broken.immobile = true;
  const FleetPlan result = plan({agent(map, 1, "A", "E"), broken});
  EXPECT_TRUE(result.agents[0].outcome == AgentOutcome::Blocked);
}

TEST_F(FleetPlanTest, WhatARobotStillHoldsIsRespectedUntilItArrives)
{
  // Robot 2 is driving B>C>D and will be at D in 5 s. Until then it holds
  // B, C, D and the edges between.
  AgentRequest moving = agent(map, 2, "D");
  moving.start_time = 5.0;
  moving.held = {
    map.node_resource(node(map, "B")), map.edge_resource(*map.find_edge("BC")),
    map.node_resource(node(map, "C")), map.edge_resource(*map.find_edge("CD")),
    map.node_resource(node(map, "D"))};
  const FleetPlan result = plan({agent(map, 1, "A", "C"), moving});
  ASSERT_TRUE(result.agents[0].outcome == AgentOutcome::Planned);
  EXPECT_DOUBLE_EQ(result.agents[0].plan.moves[0].start, 5.0);  // not before
  EXPECT_TRUE(result.agents[1].plan.moves.empty());             // idle: stays at D
}

TEST_F(FleetPlanTest, PriorityDecidesWhoCrossesFirst)
{
  // Two robots whose ways cross in the middle of a 3 x 3 grid, equally far
  // from the crossing. Whoever is more urgent goes through without waiting.
  const Roadmap grid = test_maps::grid(3, 3);
  const ConflictTable grid_conflicts(grid, test_maps::kClearance);
  for (const int urgent : {1, 2}) {
    AgentRequest across = agent(grid, 1, "0,1", "2,1");
    AgentRequest up = agent(grid, 2, "1,0", "1,2");
    across.priority = urgent == 1 ? -1.0 : 1.0;
    up.priority = urgent == 2 ? -1.0 : 1.0;
    const std::vector<AgentRequest> requests{across, up};
    const FleetPlan result = plan_fleet(grid, grid_conflicts, requests, config());
    EXPECT_EQ(validate_fleet_plan(grid, grid_conflicts, requests, result), "");
    const double first = result.agents[static_cast<std::size_t>(urgent - 1)].plan.finish_time;
    const double second = result.agents[static_cast<std::size_t>(2 - urgent)].plan.finish_time;
    EXPECT_DOUBLE_EQ(first, 6.0);  // two edges, no waiting
    EXPECT_GT(second, 6.0);
  }
}

TEST_F(FleetPlanTest, SearchAloneAlsoSolvesTheHeadOnMeetingAndFavoursTheUrgentRobot)
{
  PlannerConfig only_search = config();
  only_search.order_attempts = 1;  // stage 1 with the natural order only
  AgentRequest urgent = agent(map, 2, "E", "A");
  urgent.priority = -1.0;
  const std::vector<AgentRequest> requests{agent(map, 1, "A", "E"), urgent};

  const FleetPlan result = plan_fleet(map, conflicts, requests, only_search);
  EXPECT_EQ(validate_fleet_plan(map, conflicts, requests, result), "");
  EXPECT_TRUE(result.from_search);
  EXPECT_GT(result.search_nodes, 0u);
  EXPECT_LE(result.search_nodes, only_search.search_budget);
  // The urgent robot drives straight through; the other one uses the siding.
  EXPECT_EQ(path_of(map, node(map, "E"), result.agents[1].plan), "E>D>C>B>A");
  EXPECT_NE(path_of(map, node(map, "A"), result.agents[0].plan).find("S"), std::string::npos);

  // Without the search, and without trying other orders, nobody gets a full
  // route; the last stage then at least moves both one step.
  only_search.search_budget = 0;
  const FleetPlan stuck = plan_fleet(map, conflicts, requests, only_search);
  EXPECT_EQ(validate_fleet_plan(map, conflicts, requests, stuck), "");
  EXPECT_TRUE(stuck.agents[0].outcome == AgentOutcome::Blocked);
  EXPECT_TRUE(stuck.agents[1].outcome == AgentOutcome::Blocked);
  EXPECT_FALSE(stuck.from_search);
}

TEST(FleetPlanLoop, IdleRobotsAheadOnAOneWayLoopAreMovedOnAsAConvoy)
{
  // A one-way loop of 10 nodes. The robot at R9 must go to R6; three idle
  // robots stand ahead of it at R0, R2 and R3 and cannot be overtaken. They
  // all have to move on, the front one first, and end where the first robot
  // does not need to go: R7, R8 and R9 (which it leaves).
  const Roadmap loop = test_maps::one_way_ring(10);
  const ConflictTable loop_conflicts(loop, 1.3);
  const std::vector<AgentRequest> requests{
    agent(loop, 1, "R3"), agent(loop, 2, "R2"), agent(loop, 3, "R0"),
    agent(loop, 4, "R9", "R6")};
  const FleetPlan result = plan_fleet(loop, loop_conflicts, requests, config());
  EXPECT_EQ(validate_fleet_plan(loop, loop_conflicts, requests, result), "");
  for (const AgentPlan & item : result.agents) {
    EXPECT_TRUE(item.outcome == AgentOutcome::Planned);
  }
  EXPECT_EQ(loop.node(result.agents[3].plan.moves.back().to).id, "R6");
  EXPECT_EQ(loop.node(result.agents[0].plan.moves.back().to).id, "R9");
  EXPECT_EQ(loop.node(result.agents[1].plan.moves.back().to).id, "R8");
  EXPECT_EQ(loop.node(result.agents[2].plan.moves.back().to).id, "R7");
  EXPECT_EQ(result.search_nodes, 0u);  // found by ordering, not by search
}

TEST(FleetPlanLoop, WhenNoFullRouteIsFoundRobotsAreMovedOneStep)
{
  // The same loop, but now the robot at R3 must go almost all the way round,
  // through the three others. No complete set of routes is found within the
  // budget; stage 3 moves it one node, which is progress.
  const Roadmap loop = test_maps::one_way_ring(10);
  const ConflictTable loop_conflicts(loop, 1.3);
  AgentRequest far = agent(loop, 1, "R3", "R1");
  far.waypoints.front().dwell = 5.0;
  const std::vector<AgentRequest> requests{
    far, agent(loop, 2, "R2"), agent(loop, 3, "R0"), agent(loop, 4, "R9")};
  const FleetPlan result = plan_fleet(loop, loop_conflicts, requests, config());
  EXPECT_EQ(validate_fleet_plan(loop, loop_conflicts, requests, result), "");
  if (result.agents[0].outcome == AgentOutcome::Blocked) {
    EXPECT_EQ(result.nudged, 1u);
    ASSERT_EQ(result.agents[0].plan.moves.size(), 1u);
    EXPECT_EQ(loop.node(result.agents[0].plan.moves[0].to).id, "R4");
  }
}

TEST(FleetPlanPush, ARobotWorkingAtItsStopIsNotPushedAway)
{
  // Both have the same stop; one is already there. The other one cannot be
  // routed (the stop stays taken as far as this plan can see), and the one
  // at work must not be shoved aside to make room.
  const Roadmap grid = test_maps::grid(3, 2);
  const ConflictTable grid_conflicts(grid, 1.3);
  const std::vector<AgentRequest> requests{
    agent(grid, 1, "2,0", "2,1"), agent(grid, 2, "2,1", "2,1")};
  const FleetPlan result = plan_fleet(grid, grid_conflicts, requests, PlannerConfig{});
  EXPECT_EQ(validate_fleet_plan(grid, grid_conflicts, requests, result), "");
  EXPECT_TRUE(result.agents[0].outcome == AgentOutcome::Blocked);
  EXPECT_TRUE(result.agents[1].outcome == AgentOutcome::Planned);
  EXPECT_TRUE(result.agents[1].plan.moves.empty());
}

TEST(FleetPlanPush, ARestingIdleRobotIsPushedOnByAnotherIdleRobot)
{
  // Corridor A-B-C-D-E, no siding. A robot drives A>C and stays. Idle robots
  // stand at B and C. The one at C must leave; the nearest resting place, D,
  // is where... nobody is yet, so it goes there. The one at B must leave too,
  // towards D: it pushes the first idle robot on to E.
  const Roadmap map = test_maps::corridor();
  const ConflictTable conflicts(map, test_maps::kClearance);
  const std::vector<AgentRequest> requests{
    agent(map, 1, "A", "C"), agent(map, 2, "C"), agent(map, 3, "B")};
  const FleetPlan result = plan_fleet(map, conflicts, requests, config());
  EXPECT_EQ(validate_fleet_plan(map, conflicts, requests, result), "");
  EXPECT_TRUE(result.agents[0].outcome == AgentOutcome::Planned);
  EXPECT_EQ(path_of(map, node(map, "A"), result.agents[0].plan), "A>B>C");
  EXPECT_EQ(path_of(map, node(map, "C"), result.agents[1].plan), "C>D>E");
  EXPECT_EQ(path_of(map, node(map, "B"), result.agents[2].plan), "B>C>D");
}

// ------------------------------------------------------------ the checker ----

TEST_F(FleetPlanTest, TheValidatorRejectsACollision)
{
  const std::vector<AgentRequest> requests{agent(map, 1, "A", "C"), agent(map, 2, "C", "A")};
  FleetPlan bad;
  bad.agents.resize(2);
  for (std::size_t i = 0; i < 2; ++i) {
    RouteRequest route;
    route.robot = requests[i].robot;
    route.start_node = requests[i].start_node;
    route.waypoints = requests[i].waypoints;
    // Each planned alone, on an empty table: they meet head-on.
    bad.agents[i].robot = requests[i].robot;
    bad.agents[i].outcome = AgentOutcome::Planned;
    bad.agents[i].plan = *plan_route(map, ReservationTable(conflicts), route, config());
  }
  const std::string problem = validate_fleet_plan(map, conflicts, requests, bad);
  EXPECT_NE(problem.find("robot 1 holds"), std::string::npos);
  EXPECT_NE(problem.find("while robot 2 holds"), std::string::npos);
}

TEST_F(FleetPlanTest, TheValidatorRejectsBrokenPaths)
{
  const std::vector<AgentRequest> requests{agent(map, 1, "A", "C")};
  FleetPlan good = plan_fleet(map, conflicts, requests, config());
  ASSERT_EQ(validate_fleet_plan(map, conflicts, requests, good), "");

  FleetPlan gap = good;
  gap.agents[0].plan.moves.erase(gap.agents[0].plan.moves.begin());
  EXPECT_NE(validate_fleet_plan(map, conflicts, requests, gap).find("do not form a path"),
    std::string::npos);

  FleetPlan early = good;
  early.agents[0].plan.moves[1].start = 1.0;  // leaves B before arriving there
  EXPECT_NE(validate_fleet_plan(map, conflicts, requests, early).find("starts before"),
    std::string::npos);

  FleetPlan short_of_goal = good;
  short_of_goal.agents[0].plan.moves.pop_back();
  EXPECT_NE(validate_fleet_plan(map, conflicts, requests, short_of_goal), "");

  FleetPlan wrong_size = good;
  wrong_size.agents.clear();
  EXPECT_NE(validate_fleet_plan(map, conflicts, requests, wrong_size), "");
}

// -------------------------------------------------------- randomized check ----

TEST(FleetPlanRandom, EveryPlanOnRandomProblemsIsValid)
{
  // 400 random problems on grids: random start places, distinct random goals,
  // some with a stop on the way, some idle robots, some immobile ones, some
  // still driving. Whatever the planner returns must pass the independent
  // validity check.
  std::mt19937 random(20261128);
  std::size_t planned = 0;
  std::size_t blocked = 0;
  for (int round = 0; round < 400; ++round) {
    const int width = 3 + static_cast<int>(random() % 4);
    const int height = 2 + static_cast<int>(random() % 4);
    const Roadmap map = test_maps::grid(width, height);
    const ConflictTable conflicts(map, test_maps::kClearance);
    const std::size_t nodes = static_cast<std::size_t>(map.node_count());

    std::vector<int> places(nodes);
    for (std::size_t i = 0; i < nodes; ++i) {
      places[i] = static_cast<int>(i);
    }
    std::shuffle(places.begin(), places.end(), random);
    const std::size_t robots = 1 + random() % std::min<std::size_t>(8, nodes / 2);

    std::vector<AgentRequest> requests;
    std::vector<bool> taken(nodes, false);  // by a robot that cannot move
    for (std::size_t i = 0; i < robots; ++i) {
      AgentRequest request;
      request.robot = static_cast<int>(i) + 1;
      request.start_node = places[i];
      request.held = {map.node_resource(places[i])};
      request.priority = static_cast<double>(random() % 5);
      if (random() % 10 == 0) {
        request.immobile = true;
        taken[static_cast<std::size_t>(places[i])] = true;
      } else if (random() % 4 == 0) {
        request.start_time = static_cast<double>(1 + random() % 6);  // still arriving
      }
      requests.push_back(request);
    }
    std::vector<int> goals;
    for (std::size_t i = 0; i < nodes; ++i) {
      if (!taken[i]) {
        goals.push_back(static_cast<int>(i));
      }
    }
    std::shuffle(goals.begin(), goals.end(), random);
    std::size_t next = 0;
    for (AgentRequest & request : requests) {
      if (request.immobile || random() % 10 < 3 || next >= goals.size()) {
        continue;  // idle
      }
      if (random() % 3 == 0) {
        request.waypoints.push_back(
          Waypoint{goals[random() % goals.size()], static_cast<double>(random() % 4)});
      }
      request.waypoints.push_back(Waypoint{goals[next++], static_cast<double>(random() % 4)});
    }

    const FleetPlan result = plan_fleet(map, conflicts, requests, config());
    const std::string problem = validate_fleet_plan(map, conflicts, requests, result);
    EXPECT_EQ(problem, "");
    if (!problem.empty()) {
      EXPECT_EQ(round, -1);  // shows which round failed
      return;
    }
    EXPECT_LE(result.attempts, config().order_attempts * (4 * robots + 1));
    for (std::size_t i = 0; i < requests.size(); ++i) {
      const AgentPlan & item = result.agents[i];
      planned += item.outcome == AgentOutcome::Planned ? 1 : 0;
      blocked += item.outcome == AgentOutcome::Blocked ? 1 : 0;
      EXPECT_TRUE((item.outcome == AgentOutcome::Immobile) == requests[i].immobile);
    }
  }
  // The planner may set a task aside (some of these problems have no
  // solution), but not as a habit. Measured: about 7 % of the robots.
  EXPECT_GT(planned, 8 * blocked);
}
