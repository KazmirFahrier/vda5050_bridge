// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// What the fleet manager does when things go wrong: messages are lost,
// vehicles disappear, fail, are stopped, or turn up where nobody expected
// them. Uses the simulated fleet of sim.hpp. In every test the world's
// monitors must stay silent: whatever happens, vehicles are kept apart.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "maps.hpp"
#include "sim.hpp"

using namespace fleet_core;  // NOLINT(build/namespaces)

namespace
{

FleetConfig fleet_config()
{
  FleetConfig config;
  config.vehicle_radius = 0.4;
  config.attach_tolerance = 0.25;
  config.planner.node_overhead = 0.5;
  config.state_timeout = std::chrono::milliseconds(20000);
  return config;
}

TaskRequest go_to(const std::string & node, const std::string & vehicle = "")
{
  TaskRequest request;
  request.stops.push_back(TaskStop{node, {}, -1.0});
  request.vehicle = vehicle;
  return request;
}

VehicleView view_of(sim::World & world, const std::string & serial)
{
  for (const VehicleView & view : world.fleet().vehicles()) {
    if (view.serial_number == serial) {
      return view;
    }
  }
  return VehicleView{};
}

/// Collects events over the whole test, since take_events() empties the list.
class EventLog
{
public:
  explicit EventLog(sim::World & world)
  : world_(world) {}
  bool saw(const std::string & kind, const std::string & vehicle = "")
  {
    for (FleetEvent & event : world_.fleet().take_events()) {
      events_.push_back(std::move(event));
    }
    return std::any_of(
      events_.begin(), events_.end(), [&](const FleetEvent & event) {
        return event.kind == kind && (vehicle.empty() || event.vehicle == vehicle);
      });
  }

private:
  sim::World & world_;
  std::vector<FleetEvent> events_;
};

void expect_clean(sim::World & world)
{
  for (const std::string & violation : world.violations()) {
    EXPECT_EQ(violation, "");
  }
  EXPECT_EQ(world.fleet().stats().invalid_plans, 0u);
}

}  // namespace

// ------------------------------------------------------------- the orders ----

TEST(FleetOrders, BaseAndHorizonFollowTheProtocol)
{
  sim::World world(test_maps::corridor(), fleet_config());
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  world.fleet().submit(go_to("E"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));

  // First order: a new orderId, update 0, starting where the vehicle stands.
  ASSERT_FALSE(world.orders().empty());
  const vda5050_core::Order & first = world.orders().front();
  EXPECT_EQ(first.order_update_id, 0u);
  EXPECT_EQ(first.header.serial_number, "v1");
  EXPECT_EQ(first.nodes.front().node_id, "A");
  EXPECT_EQ(first.nodes.front().sequence_id, 0u);
  EXPECT_TRUE(first.nodes.front().released);
  // Three edges released (release_ahead), the fourth shown as horizon.
  ASSERT_EQ(first.edges.size(), 4u);
  ASSERT_EQ(first.nodes.size(), 5u);
  for (std::size_t k = 0; k < 4; ++k) {
    EXPECT_EQ(first.edges[k].sequence_id, 2 * k + 1);
    EXPECT_EQ(first.nodes[k + 1].sequence_id, 2 * k + 2);
    EXPECT_TRUE(first.edges[k].released == (k < 3));
    EXPECT_TRUE(first.nodes[k + 1].released == (k < 3));
    EXPECT_EQ(first.edges[k].start_node_id, first.nodes[k].node_id);
    EXPECT_EQ(first.edges[k].end_node_id, first.nodes[k + 1].node_id);
  }
  ASSERT_TRUE(first.nodes[1].node_position.has_value());
  EXPECT_DOUBLE_EQ(first.nodes[1].node_position->x, 2.0);
  EXPECT_EQ(first.nodes[1].node_position->map_id, "map");

  // Every later message is an update of the same order that starts at the
  // last node the vehicle was given before (stitching) and counts up.
  std::string last_base_node = "D";
  std::uint32_t last_base_sequence = 6;
  for (std::size_t k = 1; k < world.orders().size(); ++k) {
    const vda5050_core::Order & update = world.orders()[k];
    EXPECT_EQ(update.order_id, first.order_id);
    EXPECT_EQ(update.order_update_id, world.orders()[k - 1].order_update_id + 1);
    EXPECT_EQ(update.nodes.front().node_id, last_base_node);
    EXPECT_EQ(update.nodes.front().sequence_id, last_base_sequence);
    for (const vda5050_core::Node & node : update.nodes) {
      if (node.released) {
        last_base_node = node.node_id;
        last_base_sequence = node.sequence_id;
      }
    }
  }
  EXPECT_EQ(last_base_node, "E");
  EXPECT_EQ(world.fleet().stats().orders_resent, 0u);
  expect_clean(world);
}

TEST(FleetOrders, ANewTaskStartsANewOrder)
{
  sim::World world(test_maps::corridor(), fleet_config());
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  world.fleet().submit(go_to("B"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  const std::string first_id = world.orders().back().order_id;
  world.fleet().submit(go_to("A"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_NE(world.orders().back().order_id, first_id);
  EXPECT_EQ(world.orders().back().nodes.front().sequence_id, 0u);
  expect_clean(world);
}

TEST(FleetMessages, GarbageIsReportedAndIgnored)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  world.fleet().on_message("uagv/v2/sim/v9/state", "not json", world.now());
  world.fleet().on_message("uagv/v2/sim/v9/connection", "{}", world.now());
  world.fleet().on_message("something/else", "{}", world.now());
  world.fleet().on_message("other/v2/sim/v9/state", "{}", world.now());
  world.fleet().on_message("uagv/v2//v9/state", "{}", world.now());
  EXPECT_TRUE(log.saw("bad_message", "v9"));
  EXPECT_TRUE(world.fleet().vehicles().empty());  // nothing usable was learnt

  const std::vector<std::string> topics = world.fleet().subscriptions();
  ASSERT_EQ(topics.size(), 2u);
  EXPECT_EQ(topics[0], "uagv/v2/+/+/state");
  EXPECT_EQ(topics[1], "uagv/v2/+/+/connection");
}

// -------------------------------------------------------------- discovery ----

TEST(FleetDiscovery, NobodyIsMovedWhileAnAnnouncedVehicleIsNotYetLocated)
{
  sim::World world(test_maps::corridor_with_siding(), fleet_config());
  EventLog log(world);
  // The first state message of v2 is lost. The fleet manager knows from the
  // connection topic that v2 exists, but not where it is: it is at C, in the
  // middle of the way v1 is about to be sent.
  world.lose_next_to_fleet("/v2/state");
  world.add_vehicle("v1", "A");
  world.add_vehicle("v2", "C");
  world.fleet().submit(go_to("E", "v1"), world.now());

  world.run_for(0.1);
  EXPECT_TRUE(world.orders().empty());        // v1 is held back
  EXPECT_EQ(view_of(world, "v2").node, "");
  ASSERT_TRUE(world.run_until([&] {return view_of(world, "v2").node == "C";}, 5.0));
  // It was found by asking, not by waiting for its next regular state (5 s).
  EXPECT_LT(world.time(), 1.0);

  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 120.0));
  EXPECT_EQ(view_of(world, "v1").node, "E");
  EXPECT_EQ(view_of(world, "v2").node, "S");
  EXPECT_FALSE(log.saw("claim_conflict"));
  expect_clean(world);
}

TEST(FleetDiscovery, AVehicleThatNeverSaysWhereItIsDoesNotBlockTheFleetForEver)
{
  FleetConfig config = fleet_config();
  config.discovery_grace = std::chrono::milliseconds(3000);
  sim::World world(test_maps::corridor(), config);
  EventLog log(world);
  world.add_vehicle("v1", "A");
  world.run_for(0.5);
  // A vehicle announces itself and then says nothing more.
  world.fleet().on_message(
    "uagv/v2/sim/ghost/connection",
    R"({"headerId":1,"timestamp":"t","version":"2.0.0","manufacturer":"sim",)"
    R"("serialNumber":"ghost","connectionState":"ONLINE"})",
    world.now());
  world.fleet().submit(go_to("C", "v1"), world.now());
  world.run_for(2.0);
  EXPECT_TRUE(world.orders().empty());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_TRUE(log.saw("vehicle_unlocated", "ghost"));
  expect_clean(world);
}

TEST(FleetDiscovery, AVehicleAppearingOnAnotherOnesRouteStopsThatOne)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  sim::SimVehicle & v1 = world.add_vehicle("v1", "A");
  sim::SimVehicle & intruder = world.add_vehicle("v2", "D", false);  // switched off
  world.run_for(1.0);
  world.fleet().submit(go_to("E", "v1"), world.now());
  world.run_for(1.0);  // v1 is under way, with B, C and D already granted
  EXPECT_TRUE(v1.moving());

  intruder.connect(world.now());
  world.run_for(0.3);
  // v1 is stopped at once, on the edge, three nodes away from the newcomer.
  EXPECT_TRUE(log.saw("claim_conflict", "v1"));
  EXPECT_TRUE(log.saw("vehicle_resync", "v1"));
  EXPECT_LT(v1.x(), 2.0);

  // Then the fleet sorts itself out: v1 is sent on node by node, the
  // newcomer is moved out of its way to the end of the corridor. That end is
  // also where v1 was going, so v1 waits one node before it.
  world.run_for(30.0);
  EXPECT_TRUE(log.saw("claim_conflict_over"));
  EXPECT_EQ(view_of(world, "v1").node, "D");
  EXPECT_EQ(view_of(world, "v2").node, "E");
  EXPECT_TRUE(view_of(world, "v1").blocked);
  // The books were wrong for a moment (that is what the event says), but the
  // vehicles were never near each other.
  EXPECT_GE(world.closest_approach(), 2.0 - 1e-9);
  for (const std::string & violation : world.violations()) {
    EXPECT_EQ(violation.find("collision"), std::string::npos);
  }
  EXPECT_EQ(world.fleet().stats().invalid_plans, 0u);
}

TEST(FleetDiscovery, VehiclesSwitchedOnTooCloseToEachOtherAreNotMovedButOthersAre)
{
  // Nodes 0.8 m apart: two vehicles of 0.4 m radius cannot both stand there
  // by the rules, yet there they are when the system starts.
  Roadmap map;
  map.add_node("P", 0.0, 0.0);
  map.add_node("Q", 0.8, 0.0);
  map.add_node("R", 4.0, 0.0);
  map.add_node("T", 4.0, 3.0);
  map.add_node("U", 8.0, 0.0);
  map.add_bidirectional_edge("PQ", "P", "Q");
  map.add_bidirectional_edge("QR", "Q", "R");
  map.add_bidirectional_edge("RT", "R", "T");
  map.add_bidirectional_edge("RU", "R", "U");
  sim::World world(map, fleet_config());
  EventLog log(world);
  world.add_vehicle("v1", "P");
  world.add_vehicle("v2", "Q");
  world.add_vehicle("v3", "T");
  world.run_for(1.0);
  const TaskId stuck = *world.fleet().submit(go_to("U", "v2"), world.now());
  const TaskId fine = *world.fleet().submit(go_to("U", "v3"), world.now());
  ASSERT_TRUE(
    world.run_until(
      [&] {return world.fleet().task(fine)->state == TaskState::Finished;}, 60.0));
  world.run_for(10.0);
  EXPECT_TRUE(log.saw("claim_conflict"));
  EXPECT_TRUE(world.fleet().task(stuck)->state == TaskState::Assigned);
  EXPECT_EQ(view_of(world, "v1").node, "P");
  EXPECT_EQ(view_of(world, "v2").node, "Q");
  EXPECT_EQ(world.fleet().stats().invalid_plans, 0u);
}

TEST(FleetDiscovery, AVehicleBusyWithSomebodyElsesOrderIsStoppedBeforeItIsUsed)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  // An order from an earlier life of the system: drive A > B > C.
  vda5050_core::Order foreign;
  foreign.header.header_id = 1;
  foreign.header.timestamp = "2026-01-01T00:00:00.000Z";
  foreign.header.version = "2.0.0";
  foreign.header.manufacturer = "sim";
  foreign.header.serial_number = "v1";
  foreign.order_id = "left-over";
  const char * names[] = {"A", "B", "C"};
  for (std::uint32_t k = 0; k < 3; ++k) {
    vda5050_core::Node node;
    node.node_id = names[k];
    node.sequence_id = 2 * k;
    node.released = true;
    vda5050_core::NodePosition position;
    position.x = 2.0 * k;
    position.y = 0.0;
    position.map_id = "map";
    node.node_position = position;
    foreign.nodes.push_back(node);
    if (k > 0) {
      vda5050_core::Edge edge;
      edge.edge_id = std::string(names[k - 1]) + names[k];
      edge.sequence_id = 2 * k - 1;
      edge.released = true;
      edge.start_node_id = names[k - 1];
      edge.end_node_id = names[k];
      foreign.edges.push_back(edge);
    }
  }
  vehicle.deliver("uagv/v2/sim/v1/order", vda5050_core::encode_order(foreign));
  world.step();
  EXPECT_TRUE(vehicle.moving());
  world.run_for(1.0);
  EXPECT_TRUE(log.saw("vehicle_resync", "v1"));
  EXPECT_FALSE(vehicle.moving());
  EXPECT_LT(vehicle.x(), 0.5);  // stopped within a fraction of a second

  // It is stopped, sent on to the next node, and then takes tasks as usual.
  const TaskId task = *world.fleet().submit(go_to("E"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 120.0));
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Finished);
  EXPECT_NE(world.orders().front().order_id, "left-over");
  expect_clean(world);
}

TEST(FleetDiscovery, AVehicleThatIsNotOnTheRoadmapIsLeftAloneUntilItIs)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  sim::SimVehicle & lost = world.add_vehicle("v1", "A", false);
  lost.set_position(1.0, 1.5);  // next to the corridor, at no node
  world.run_for(0.1);
  lost.connect(world.now());
  world.run_for(1.0);
  EXPECT_EQ(view_of(world, "v1").node, "");
  EXPECT_FALSE(view_of(world, "v1").available);
  EXPECT_EQ(view_of(world, "v1").unavailable_reason, "not at a node of the roadmap");
  const TaskId task = *world.fleet().submit(go_to("C"), world.now());
  world.run_for(5.0);
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Queued);

  lost.set_position(2.0, 0.1);  // somebody drives it to B, near enough
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_EQ(view_of(world, "v1").node, "C");
  expect_clean(world);
}

TEST(FleetDiscovery, AVehicleFoundBetweenTwoNodesIsSentOnToTheNextOne)
{
  // The fleet manager starts while a vehicle stands in the middle of an
  // edge, between B and C, a little nearer to C.
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A", false);
  vehicle.set_position(3.2, 0.0);
  world.run_for(0.1);
  vehicle.connect(world.now());
  world.run_for(0.2);
  EXPECT_TRUE(log.saw("vehicle_recovering", "v1"));
  // Meanwhile it holds the whole edge and both its ends.
  EXPECT_EQ(view_of(world, "v1").claimed.size(), 3u);
  ASSERT_TRUE(
    world.run_until(
      [&] {return view_of(world, "v1").node == "C" && !vehicle.moving();}, 20.0));
  EXPECT_NEAR(vehicle.x(), 4.0, 1e-9);
  EXPECT_NEAR(vehicle.distance_driven(), 0.8, 1e-6);
  world.fleet().submit(go_to("A"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_EQ(view_of(world, "v1").node, "A");
  expect_clean(world);
}

TEST(FleetDiscovery, AVehicleFoundOnAnEdgeThatSomebodyElseHoldsWaits)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  world.add_vehicle("v1", "C");
  world.run_for(1.0);
  sim::SimVehicle & late = world.add_vehicle("v2", "A", false);
  late.set_position(3.0, 0.0);  // on the edge B-C, whose end C is taken
  world.run_for(0.1);
  late.connect(world.now());
  world.run_for(15.0);
  EXPECT_FALSE(log.saw("vehicle_recovering", "v2"));
  EXPECT_EQ(view_of(world, "v2").node, "");
  EXPECT_FALSE(late.moving());

  // The other one is sent away; now the edge can be given to the newcomer.
  world.fleet().submit(go_to("E", "v1"), world.now());
  ASSERT_TRUE(
    world.run_until(
      [&] {return !view_of(world, "v2").node.empty() && !late.moving();}, 60.0));
  EXPECT_TRUE(log.saw("vehicle_recovering", "v2"));
  EXPECT_GE(world.closest_approach(), 1.0 - 1e-9);
}

TEST(FleetDiscovery, AVehicleStoppedAwayFromTheRoadmapWaitsToBePutBack)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(1.0);
  const TaskId task = *world.fleet().submit(go_to("E"), world.now());
  world.run_for(2.0);
  ASSERT_TRUE(vehicle.moving());
  // Stopped by hand and pushed aside, 1.5 m off the edge it was on.
  world.fleet().cancel(task, world.now());
  vehicle.set_position(vehicle.x(), 1.5);
  world.run_for(3.0);
  EXPECT_TRUE(log.saw("vehicle_off_road", "v1"));
  EXPECT_FALSE(view_of(world, "v1").available);
  EXPECT_EQ(view_of(world, "v1").unavailable_reason, "stopped away from the roadmap");
  // It still counts as being where it was last known to be: nothing else is
  // let through there.
  EXPECT_FALSE(view_of(world, "v1").claimed.empty());

  vehicle.set_position(4.0, 0.0);  // put back, on node C
  ASSERT_TRUE(world.run_until([&] {return view_of(world, "v1").available;}, 20.0));
  EXPECT_EQ(view_of(world, "v1").node, "C");
  EXPECT_EQ(view_of(world, "v1").claimed.size(), 1u);
  world.fleet().submit(go_to("A"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_EQ(view_of(world, "v1").node, "A");
}

TEST(FleetDiscovery, AVehicleInManualModeIsNotGivenOrders)
{
  sim::World world(test_maps::corridor(), fleet_config());
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  vehicle.session().orders().set_operating_mode(vda5050_core::OperatingMode::Manual);
  world.run_for(1.0);
  EXPECT_FALSE(view_of(world, "v1").available);
  EXPECT_EQ(view_of(world, "v1").unavailable_reason, "operating mode MANUAL");
  const TaskId task = *world.fleet().submit(go_to("B"), world.now());
  world.run_for(5.0);
  EXPECT_TRUE(world.orders().empty());
  vehicle.session().orders().set_operating_mode(vda5050_core::OperatingMode::Automatic);
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Finished);
  expect_clean(world);
}

TEST(FleetDiscovery, AVehicleThatForgetsItsOrderIsBroughtBackIntoStep)
{
  // The vehicle's computer restarts under way: it reports no order at all,
  // although it was given one. (Here the report is made up; the simulated
  // vehicle itself drives on, and is stopped.)
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(1.0);
  const TaskId task = *world.fleet().submit(go_to("E"), world.now());
  world.run_for(1.5);
  vda5050_core::State blank = vehicle.session().orders().snapshot();
  blank.header.header_id = 9999;
  blank.header.manufacturer = "sim";
  blank.header.serial_number = "v1";
  blank.header.version = "2.0.0";
  blank.header.timestamp = "2026-01-01T00:00:00.000Z";
  blank.order_id = "";
  blank.order_update_id = 0;
  blank.node_states.clear();
  blank.edge_states.clear();
  blank.driving = false;
  world.fleet().on_message(
    "uagv/v2/sim/v1/state", vda5050_core::encode_state(blank), world.now());
  EXPECT_TRUE(log.saw("order_lost", "v1"));
  EXPECT_TRUE(log.saw("vehicle_resync", "v1"));
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 120.0));
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Finished);
  expect_clean(world);
}

TEST(FleetDiscovery, AVehicleThatStopsShortOfItsOrderIsNoticed)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(1.0);
  const TaskId task = *world.fleet().submit(go_to("E"), world.now());
  world.run_for(1.5);
  // It claims to be done with the order while it has not passed a single node.
  vda5050_core::State early = vehicle.session().orders().snapshot();
  early.header.header_id = 9999;
  early.header.manufacturer = "sim";
  early.header.serial_number = "v1";
  early.header.version = "2.0.0";
  early.header.timestamp = "2026-01-01T00:00:00.000Z";
  early.node_states.clear();
  early.edge_states.clear();
  early.driving = false;
  world.fleet().on_message(
    "uagv/v2/sim/v1/state", vda5050_core::encode_state(early), world.now());
  EXPECT_TRUE(log.saw("order_abandoned", "v1"));
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 120.0));
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Finished);
  expect_clean(world);
}

// ------------------------------------------------------------ the network ----

TEST(FleetNetwork, LostMessagesAreMadeUpFor)
{
  sim::WorldOptions options;
  options.loss = 0.3;
  options.jitter = 0.2;
  options.seed = 7;
  sim::World world(test_maps::corridor_with_siding(), fleet_config(), options);
  world.add_vehicle("v1", "A");
  world.add_vehicle("v2", "E");
  world.run_for(8.0);  // long enough for both to be heard despite the loss
  world.fleet().submit(go_to("E", "v1"), world.now());
  world.fleet().submit(go_to("A", "v2"), world.now());

  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 400.0));
  EXPECT_EQ(world.fleet().stats().tasks_finished, 2u);
  EXPECT_GT(world.fleet().stats().orders_resent, 0u);
  expect_clean(world);
}

TEST(FleetNetwork, ALostOrderIsSentAgain)
{
  sim::World world(test_maps::corridor(), fleet_config());
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(1.0);
  world.lose_next_to_vehicles("/v1/order");
  world.fleet().submit(go_to("C"), world.now());
  world.run_for(1.5);
  EXPECT_FALSE(vehicle.moving());  // it never heard of the order
  EXPECT_EQ(world.fleet().stats().orders_resent, 0u);
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_EQ(world.fleet().stats().orders_resent, 1u);
  EXPECT_EQ(world.fleet().stats().tasks_finished, 1u);
  // The same order twice, word for word apart from nothing at all.
  ASSERT_TRUE(world.orders().size() >= 2u);
  EXPECT_EQ(world.orders()[0].order_id, world.orders()[1].order_id);
  EXPECT_EQ(world.orders()[0].order_update_id, world.orders()[1].order_update_id);
  EXPECT_EQ(world.orders()[0].header.header_id, world.orders()[1].header.header_id);
  expect_clean(world);
}

TEST(FleetNetwork, AFailureIsNoticedEvenWhileAnOrderIsUnconfirmed)
{
  // The drive fails while an update of the order is under way and lost. The
  // update is sent again although the vehicle is not usable at that moment;
  // otherwise the failure would never be dealt with.
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(1.0);
  const TaskId task = *world.fleet().submit(go_to("E"), world.now());
  ASSERT_TRUE(world.run_until([&] {return vehicle.x() > 1.5;}, 10.0));
  world.lose_next_to_vehicles("/v1/order");  // the update sent on reaching B
  vehicle.fail_next_navigation();
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 120.0));
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Finished);
  EXPECT_TRUE(log.saw("navigation_failed", "v1"));
  EXPECT_GT(world.fleet().stats().orders_resent, 0u);
  expect_clean(world);
}

TEST(FleetNetwork, AnOrderOnItsWayToAVehicleThatIsToBeStoppedIsSeenThrough)
{
  // The task is cancelled in the very moment its first order has been sent.
  // The order cannot be called back; the vehicle will get it and start. The
  // fleet manager must not take the vehicle for idle meanwhile.
  sim::WorldOptions options;
  options.latency = 0.3;
  sim::World world(test_maps::corridor(), fleet_config(), options);
  EventLog log(world);
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(2.0);
  const TaskId task = *world.fleet().submit(go_to("E"), world.now());
  ASSERT_TRUE(world.run_until([&] {return !world.orders().empty();}, 5.0));
  EXPECT_TRUE(world.fleet().cancel(task, world.now()));

  // Until the vehicle has stopped for good, what it was granted stays claimed.
  bool ever_moved = false;
  for (int k = 0; k < 200; ++k) {
    world.step();
    ever_moved = ever_moved || vehicle.moving();
    if (vehicle.moving()) {
      EXPECT_GT(view_of(world, "v1").claimed.size(), 1u);
    }
  }
  EXPECT_TRUE(ever_moved);
  EXPECT_TRUE(log.saw("vehicle_resynchronized", "v1"));
  EXPECT_FALSE(vehicle.moving());
  EXPECT_TRUE(view_of(world, "v1").available);
  EXPECT_EQ(view_of(world, "v1").claimed.size(), 1u);
  expect_clean(world);
}

TEST(FleetNetwork, AVehicleThatGoesOfflineIsPlannedAroundAndComesBack)
{
  sim::World world(test_maps::grid(3, 2), fleet_config());
  EventLog log(world);
  world.add_vehicle("v1", "0,0");
  sim::SimVehicle & silent = world.add_vehicle("v2", "1,0");
  world.run_for(1.0);
  silent.drop_connection();
  world.run_for(0.5);
  EXPECT_FALSE(view_of(world, "v2").available);
  EXPECT_EQ(view_of(world, "v2").unavailable_reason, "offline");

  // The direct way leads through the node where the silent vehicle stands.
  sim::SimVehicle & v1 = world.vehicle("v1");
  world.fleet().submit(go_to("2,0", "v1"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_NEAR(v1.distance_driven(), 8.0, 1e-6);  // round the top row: 4 edges, not 2
  EXPECT_EQ(view_of(world, "v2").node, "1,0");

  silent.connect(world.now());
  ASSERT_TRUE(world.run_until([&] {return view_of(world, "v2").available;}, 10.0));
  world.fleet().submit(go_to("0,0", "v2"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_EQ(view_of(world, "v2").node, "0,0");
  EXPECT_TRUE(log.saw("vehicle_offline", "v2"));
  expect_clean(world);
}

TEST(FleetNetwork, AVehicleSilentForTooLongCountsAsLost)
{
  FleetConfig config = fleet_config();
  config.state_timeout = std::chrono::milliseconds(8000);
  sim::WorldOptions options;
  options.state_interval = std::chrono::milliseconds(60000);  // it says nothing unprompted
  sim::World world(test_maps::corridor(), config, options);
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  EXPECT_TRUE(view_of(world, "v1").available);
  world.run_for(9.0);
  EXPECT_FALSE(view_of(world, "v1").available);
  EXPECT_EQ(view_of(world, "v1").unavailable_reason, "no state for too long");
}

// ---------------------------------------------------------------- failures ----

TEST(FleetFailures, AFailedDriveIsStoppedAndTriedAgain)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(1.0);
  vehicle.fail_next_navigation();
  const TaskId task = *world.fleet().submit(go_to("C"), world.now());

  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Finished);
  EXPECT_TRUE(log.saw("navigation_failed", "v1"));
  EXPECT_TRUE(log.saw("vehicle_resynchronized", "v1"));
  EXPECT_EQ(view_of(world, "v1").node, "C");
  expect_clean(world);
}

TEST(FleetFailures, AVehicleThatKeepsFailingIsTakenOutOfServiceUntilReset)
{
  FleetConfig config = fleet_config();
  config.max_navigation_failures = 2;
  sim::World world(test_maps::corridor(), config);
  EventLog log(world);
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(1.0);
  const TaskId task = *world.fleet().submit(go_to("C"), world.now());
  // Fail every attempt.
  ASSERT_TRUE(
    world.run_until(
      [&] {
        vehicle.fail_next_navigation();
        return world.all_tasks_over();
      }, 60.0));
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Failed);
  EXPECT_EQ(world.fleet().task(task)->note, "vehicle taken out of service");
  EXPECT_TRUE(log.saw("vehicle_out_of_service", "v1"));
  world.run_for(2.0);
  EXPECT_EQ(view_of(world, "v1").unavailable_reason, "taken out of service");

  // A task for it now waits; after the reset it is done.
  const TaskId next = *world.fleet().submit(go_to("B"), world.now());
  world.run_for(5.0);
  EXPECT_TRUE(world.fleet().task(next)->state == TaskState::Queued);
  EXPECT_FALSE(world.fleet().reset_vehicle("nobody", world.now()));
  EXPECT_TRUE(world.fleet().reset_vehicle("v1", world.now()));
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_TRUE(world.fleet().task(next)->state == TaskState::Finished);
  expect_clean(world);
}

TEST(FleetFailures, AFailedActionFailsTheTask)
{
  sim::World world(test_maps::corridor(), fleet_config());
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  TaskRequest request;
  vda5050_core::Action broken;
  broken.action_type = "fail";
  broken.blocking_type = vda5050_core::BlockingType::Hard;
  request.stops.push_back(TaskStop{"B", {broken}, -1.0});
  request.stops.push_back(TaskStop{"D", {}, -1.0});
  const TaskId task = *world.fleet().submit(request, world.now());

  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  const TaskView view = *world.fleet().task(task);
  EXPECT_TRUE(view.state == TaskState::Failed);
  EXPECT_NE(view.note.find("action fail failed"), std::string::npos);
  EXPECT_EQ(view.stops_done, 0u);
  EXPECT_EQ(world.fleet().stats().tasks_failed, 1u);
  // The vehicle was stopped; it did not carry on to the second stop.
  world.run_for(10.0);
  EXPECT_LT(world.vehicle("v1").x(), 4.0 + 1e-9);
  // It is free for the next task, and that one is done properly.
  const TaskId next = *world.fleet().submit(go_to("A"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_TRUE(world.fleet().task(next)->state == TaskState::Finished);
  EXPECT_EQ(view_of(world, "v1").node, "A");
  EXPECT_NEAR(world.vehicle("v1").x(), 0.0, 1e-9);
  expect_clean(world);
}

TEST(FleetFailures, AnUnsupportedActionIsRejectedByTheVehicleAndTheTaskDoesNotHang)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  TaskRequest request;
  vda5050_core::Action unknown;
  unknown.action_type = "juggle";
  unknown.blocking_type = vda5050_core::BlockingType::Hard;
  request.stops.push_back(TaskStop{"B", {unknown}, -1.0});
  world.fleet().submit(request, world.now());
  world.run_for(10.0);
  EXPECT_TRUE(log.saw("order_rejected", "v1"));
  // The vehicle did not move, and it is still usable.
  EXPECT_EQ(view_of(world, "v1").node, "A");
  EXPECT_TRUE(view_of(world, "v1").available);
  expect_clean(world);
}

// --------------------------------------------------------------- operators ----

TEST(FleetOperators, CancellingATaskStopsTheVehicleAndFreesItForTheNext)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EventLog log(world);
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(1.0);
  const TaskId task = *world.fleet().submit(go_to("E"), world.now());
  world.run_for(3.0);  // somewhere between B and C
  ASSERT_TRUE(vehicle.moving());

  EXPECT_TRUE(world.fleet().cancel(task, world.now()));
  EXPECT_FALSE(world.fleet().cancel(task, world.now()));  // already over
  EXPECT_FALSE(world.fleet().cancel(999, world.now()));
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Cancelled);

  // It stops where it is, on the edge, and is then sent on to the next node.
  world.run_for(0.5);
  EXPECT_TRUE(log.saw("vehicle_recovering", "v1"));
  ASSERT_TRUE(
    world.run_until(
      [&] {return view_of(world, "v1").node == "C" && !vehicle.moving();}, 30.0));
  EXPECT_TRUE(view_of(world, "v1").available);
  EXPECT_NEAR(vehicle.x(), 4.0, 1e-9);
  EXPECT_EQ(view_of(world, "v1").claimed.size(), 1u);  // only the node it stands on

  world.fleet().submit(go_to("A"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_EQ(view_of(world, "v1").node, "A");
  expect_clean(world);
}

TEST(FleetOperators, CancellingAQueuedTaskJustRemovesIt)
{
  sim::World world(test_maps::corridor(), fleet_config());
  const TaskId task = *world.fleet().submit(go_to("E"), world.now());
  EXPECT_TRUE(world.fleet().cancel(task, world.now()));
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Cancelled);
  EXPECT_TRUE(world.orders().empty());
}

TEST(FleetOperators, AVehicleHeldByAFaultIsPlannedAroundAndResumesAfterwards)
{
  sim::World world(test_maps::grid(3, 2), fleet_config());
  world.add_vehicle("v1", "0,0");
  sim::SimVehicle & faulty = world.add_vehicle("v2", "2,0");
  world.run_for(1.0);
  const TaskId first = *world.fleet().submit(go_to("0,1", "v2"), world.now());
  world.run_for(2.5);  // v2 is on its way along the bottom row
  // A fatal fault: the vehicle side holds itself (see fault management).
  faulty.session().orders().set_external_errors(
    {vda5050_core::Error{"LIDAR-001", {}, "no scans", vda5050_core::ErrorLevel::Fatal}});
  faulty.session().orders().set_hold(true);
  world.run_for(1.0);
  EXPECT_FALSE(view_of(world, "v2").available);

  // Another vehicle gets a task meanwhile and drives round the stopped one.
  const TaskId second = *world.fleet().submit(go_to("2,1", "v1"), world.now());
  ASSERT_TRUE(
    world.run_until(
      [&] {return world.fleet().task(second)->state == TaskState::Finished;}, 60.0));
  EXPECT_TRUE(world.fleet().task(first)->state == TaskState::Assigned);

  // The fault clears; the held vehicle carries on with its own task.
  faulty.session().orders().set_external_errors({});
  faulty.session().orders().set_hold(false);
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_TRUE(world.fleet().task(first)->state == TaskState::Finished);
  expect_clean(world);
}

TEST(FleetOperators, UrgentTasksAreAssignedFirst)
{
  sim::World world(test_maps::corridor(), fleet_config());
  world.add_vehicle("v1", "C");
  world.run_for(1.0);
  // Three tasks at once for one vehicle; the last one submitted is urgent.
  TaskRequest routine_a = go_to("A");
  TaskRequest routine_b = go_to("E");
  TaskRequest urgent = go_to("D");
  urgent.priority = 5;
  const TaskId a = *world.fleet().submit(routine_a, world.now());
  const TaskId b = *world.fleet().submit(routine_b, world.now());
  const TaskId u = *world.fleet().submit(urgent, world.now());
  world.run_for(0.2);
  EXPECT_TRUE(world.fleet().task(u)->state == TaskState::Assigned);
  EXPECT_TRUE(world.fleet().task(a)->state == TaskState::Queued);
  EXPECT_TRUE(world.fleet().task(b)->state == TaskState::Queued);
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 120.0));
  expect_clean(world);
}

// ------------------------------------------------------------- hard cases ----

TEST(FleetHardCases, TwoRobotsWhoseStopsLieBehindEachOtherInACorridor)
{
  // v1 at A must end at C; v2 in the siding must end at B. Whoever goes
  // first shuts the other one out, so v1 has to overshoot to D, let v2
  // through, and come back. Found by the search stage.
  sim::World world(test_maps::corridor_with_siding(), fleet_config());
  sim::SimVehicle & v1 = world.add_vehicle("v1", "A");
  sim::SimVehicle & v2 = world.add_vehicle("v2", "S");
  world.run_for(1.0);
  world.fleet().submit(go_to("C", "v1"), world.now());
  world.fleet().submit(go_to("B", "v2"), world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 120.0));
  EXPECT_EQ(world.fleet().stats().tasks_finished, 2u);
  EXPECT_GT(world.fleet().stats().searches, 0u);
  // No wandering about: A>B>C>D>C and S>C>B, nothing more.
  EXPECT_NEAR(v1.distance_driven(), 8.0, 1e-6);
  EXPECT_NEAR(v2.distance_driven(), 4.0, 1e-6);
  expect_clean(world);
}

TEST(FleetHardCases, ACrowdedOneWayLoopKeepsMovingUntilEveryStopIsServed)
{
  // Four vehicles on a one-way loop of ten nodes; two have tasks with two
  // stops each that send them most of the way round, through the others. No
  // complete set of routes exists within the planner's budget. Progress
  // comes from routing to the next stop only and from moving everybody on.
  sim::World world(test_maps::one_way_ring(10), fleet_config());
  world.add_vehicle("v1", "R8");
  world.add_vehicle("v2", "R9");
  world.add_vehicle("v3", "R1");
  world.add_vehicle("v4", "R6");
  world.run_for(1.0);
  vda5050_core::Action pick;
  pick.action_type = "pick";
  pick.blocking_type = vda5050_core::BlockingType::Hard;
  TaskRequest first;
  first.vehicle = "v2";
  first.stops = {TaskStop{"R1", {pick}, -1.0}, TaskStop{"R8", {}, -1.0}};
  TaskRequest second;
  second.vehicle = "v4";
  second.stops = {TaskStop{"R4", {}, -1.0}, TaskStop{"R0", {pick}, -1.0}};
  world.fleet().submit(first, world.now());
  world.fleet().submit(second, world.now());
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 300.0));
  EXPECT_EQ(world.fleet().stats().tasks_finished, 2u);
  EXPECT_EQ(world.vehicle("v2").actions_started(), 1u);
  EXPECT_EQ(world.vehicle("v4").actions_started(), 1u);
  expect_clean(world);
}

// ---------------------------------------------------------- everything at once ----

namespace
{

struct Scenario
{
  std::size_t tasks{0};
  std::size_t finished{0};
  std::size_t open{0};
  std::vector<std::string> violations;
  std::uint64_t invalid_plans{0};
};

/// A random map, random vehicles, random tasks arriving over 30 s; with
/// `chaos`, random faults for the first minute. Returns what was left.
Scenario run_scenario(unsigned seed, bool chaos)
{
  std::mt19937 random(seed);
  const auto pick = [&random](unsigned n) {return static_cast<int>(random() % n);};

  Roadmap map;
  switch (pick(4)) {
    case 0: map = test_maps::grid(3 + pick(3), 2 + pick(3)); break;
    case 1: map = test_maps::corridor_with_siding(); break;
    case 2: map = test_maps::one_way_ring(6 + pick(5)); break;
    default: map = test_maps::grid(4 + pick(3), 3 + pick(2)); break;
  }
  sim::WorldOptions options;
  options.seed = seed;
  options.latency = 0.02 + 0.05 * pick(3);
  options.jitter = 0.1 * pick(2);
  options.loss = seed % 3 == 0 ? 0.1 : 0.0;
  sim::World world(map, fleet_config(), options);

  const int nodes = map.node_count();
  const int robots = 1 + pick(static_cast<unsigned>(std::max(1, std::min(6, nodes / 2 - 1))));
  std::vector<int> places(static_cast<std::size_t>(nodes));
  for (int i = 0; i < nodes; ++i) {
    places[static_cast<std::size_t>(i)] = i;
  }
  std::shuffle(places.begin(), places.end(), random);
  for (int i = 0; i < robots; ++i) {
    world.add_vehicle(
      "v" + std::to_string(i + 1), map.node(places[static_cast<std::size_t>(i)]).id)
    .set_speed(0.5 + 0.01 * pick(100));
  }
  world.run_for(1.0);

  std::vector<std::pair<double, TaskRequest>> schedule;
  const int tasks = 2 + pick(8);
  for (int k = 0; k < tasks; ++k) {
    TaskRequest request;
    const int stops = 1 + pick(2);
    for (int s = 0; s < stops; ++s) {
      TaskStop stop;
      stop.node = map.node(pick(static_cast<unsigned>(nodes))).id;
      if (pick(3) == 0) {
        vda5050_core::Action action;
        action.action_type = pick(2) == 0 ? "pick" : "drop";
        action.blocking_type = vda5050_core::BlockingType::Hard;
        stop.actions.push_back(action);
      }
      request.stops.push_back(stop);
    }
    request.priority = pick(3);
    schedule.push_back({0.1 * pick(300), request});
  }
  std::sort(
    schedule.begin(), schedule.end(),
    [](const auto & a, const auto & b) {return a.first < b.first;});

  struct Repair
  {
    double at;
    int vehicle;
    int kind;
  };
  std::vector<Repair> repairs;
  const double chaos_until = 60.0;
  const double limit = chaos ? 700.0 : 430.0;
  std::size_t next = 0;
  bool reset_done = false;

  while (world.time() < limit && world.violations().empty()) {
    while (next < schedule.size() && schedule[next].first <= world.time()) {
      world.fleet().submit(schedule[next].second, world.now());
      ++next;
    }
    if (chaos) {
      if (world.time() < chaos_until && pick(200) == 0) {
        const int index = pick(static_cast<unsigned>(robots));
        sim::SimVehicle & vehicle = *world.vehicles()[static_cast<std::size_t>(index)];
        switch (pick(6)) {
          case 0:
            if (vehicle.connected()) {
              vehicle.drop_connection();
              repairs.push_back({world.time() + 3 + pick(20), index, 0});
            }
            break;
          case 1: vehicle.fail_next_navigation(); break;
          case 2: vehicle.set_speed(0.2 + 0.01 * pick(130)); break;
          case 3:
            vehicle.session().orders().set_hold(true);
            repairs.push_back({world.time() + 2 + pick(15), index, 3});
            break;
          case 4:
            vehicle.session().orders().set_external_errors(
              {vda5050_core::Error{"LIDAR-001", {}, "no scans",
                  vda5050_core::ErrorLevel::Fatal}});
            vehicle.session().orders().set_hold(true);
            repairs.push_back({world.time() + 2 + pick(15), index, 4});
            break;
          default: {
              const std::vector<TaskView> all = world.fleet().tasks();
              if (!all.empty()) {
                world.fleet().cancel(all[random() % all.size()].id, world.now());
              }
            }
        }
      }
      for (auto it = repairs.begin(); it != repairs.end(); ) {
        if (it->at > world.time()) {
          ++it;
          continue;
        }
        sim::SimVehicle & vehicle = *world.vehicles()[static_cast<std::size_t>(it->vehicle)];
        if (it->kind == 0) {
          vehicle.connect(world.now());
        } else {
          vehicle.session().orders().set_external_errors({});
          vehicle.session().orders().set_hold(false);
        }
        it = repairs.erase(it);
      }
      if (!reset_done && world.time() > chaos_until + 40.0) {
        reset_done = true;  // the operator puts failed vehicles back into service
        for (const auto & vehicle : world.vehicles()) {
          world.fleet().reset_vehicle(vehicle->serial(), world.now());
        }
      }
    }
    if (next == schedule.size() && world.all_tasks_over() &&
      (!chaos || world.time() > chaos_until + 45.0))
    {
      break;
    }
    world.step();
  }

  Scenario result;
  result.tasks = static_cast<std::size_t>(tasks);
  result.violations = world.violations();
  result.invalid_plans = world.fleet().stats().invalid_plans;
  for (const TaskView & task : world.fleet().tasks()) {
    result.finished += task.state == TaskState::Finished ? 1 : 0;
    result.open +=
      (task.state == TaskState::Queued || task.state == TaskState::Assigned) ? 1 : 0;
  }
  return result;
}

/// How many scenarios a soak test runs. The default keeps the test suite
/// quick; FLEET_SOAK_SCENARIOS=10000 turns the same test into a long run.
unsigned scenario_count(unsigned by_default)
{
  const char * text = std::getenv("FLEET_SOAK_SCENARIOS");
  const long wanted = text == nullptr ? 0 : std::strtol(text, nullptr, 10);  // NOLINT(runtime/int)
  return wanted > 0 ? static_cast<unsigned>(wanted) : by_default;
}

void run_many(unsigned first_seed, unsigned count, bool chaos)
{
  std::size_t tasks = 0;
  std::size_t finished = 0;
  for (unsigned seed = first_seed; seed < first_seed + count; ++seed) {
    const Scenario result = run_scenario(seed, chaos);
    tasks += result.tasks;
    finished += result.finished;
    const bool good =
      result.violations.empty() && result.open == 0 && result.invalid_plans == 0;
    EXPECT_TRUE(good);
    if (!good) {
      EXPECT_EQ(seed, 0u);  // shows which seed failed
      for (const std::string & violation : result.violations) {
        EXPECT_EQ(violation, "");
      }
      EXPECT_EQ(result.open, 0u);
    }
  }
  if (!chaos) {
    EXPECT_EQ(finished, tasks);  // with nothing going wrong, every task is done
  } else {
    EXPECT_GT(finished, tasks / 2);
  }
  std::printf(
    "             %u scenarios, %zu tasks, %zu finished\n", count, tasks, finished);
}

}  // namespace

TEST(FleetSoak, RandomFleetsFinishEveryTaskWithoutAnyVehicleComingTooClose)
{
  // 40 random scenarios: grids, a corridor with a siding, one-way loops; up to
  // six vehicles of different speeds; tasks arriving over time; network delay,
  // jitter and (in every third scenario) 10 % message loss.
  run_many(1, scenario_count(40), false);
}

TEST(FleetSoak, WithFaultsInjectedNoVehicleComesTooCloseAndNoTaskIsLeftHanging)
{
  // The same, with a fault every ten seconds on average during the first
  // minute: connections dropped, drives failing, vehicles slowed, held, with
  // fatal errors, tasks cancelled. Afterwards every task must have ended one
  // way or another, and the monitors must have stayed silent throughout.
  run_many(1000001, scenario_count(25), true);
}
