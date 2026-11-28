// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// The fleet manager driving simulated vehicles. See sim.hpp for what is real
// and what is fake. Every test ends by checking the world's three monitors.

#include <gtest/gtest.h>

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
  config.attach_tolerance = 0.25;  // distance kept: 2 * 0.4 + 2 * 0.25 = 1.3 m
  config.planner.default_speed = 1.0;
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

vda5050_core::Action action(const std::string & type)
{
  vda5050_core::Action result;
  result.action_type = type;
  result.blocking_type = vda5050_core::BlockingType::Hard;
  return result;
}

std::string node_of(sim::World & world, const std::string & serial)
{
  for (const VehicleView & view : world.fleet().vehicles()) {
    if (view.serial_number == serial) {
      return view.node;
    }
  }
  return "?";
}

void expect_clean(sim::World & world)
{
  for (const std::string & violation : world.violations()) {
    EXPECT_EQ(violation, "");
  }
  EXPECT_EQ(world.fleet().stats().invalid_plans, 0u);
}

}  // namespace

TEST(FleetSim, OneVehicleDrivesToItsGoal)
{
  sim::World world(test_maps::corridor(), fleet_config());
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(1.0);
  const TaskId task = *world.fleet().submit(go_to("E"), world.now());

  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_TRUE(world.fleet().task(task)->state == TaskState::Finished);
  EXPECT_EQ(world.fleet().task(task)->vehicle, "v1");
  EXPECT_NEAR(vehicle.x(), 8.0, 1e-9);
  EXPECT_EQ(node_of(world, "v1"), "E");
  EXPECT_NEAR(vehicle.distance_driven(), 8.0, 1e-6);  // no detours
  expect_clean(world);
}

TEST(FleetSim, TheVehicleDrivesThroughWithoutStoppingAtEveryNode)
{
  // With three edges released ahead, a 4-edge trip at 1 m/s takes little
  // more than the 8 s of pure driving.
  sim::World world(test_maps::corridor(), fleet_config());
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  world.fleet().submit(go_to("E"), world.now());
  const double started = world.time();
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  EXPECT_LT(world.time() - started, 9.0);
  expect_clean(world);
}

TEST(FleetSim, ActionsAtStopsAreExecutedInOrder)
{
  sim::World world(test_maps::corridor(), fleet_config());
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "B");
  world.run_for(1.0);

  TaskRequest transport;
  transport.stops.push_back(TaskStop{"B", {action("pick")}, -1.0});            // where it stands
  transport.stops.push_back(TaskStop{"D", {action("drop"), action("wait")}, -1.0});
  transport.stops.push_back(TaskStop{"A", {}, -1.0});
  const TaskId task = *world.fleet().submit(transport, world.now());

  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 120.0));
  const TaskView view = *world.fleet().task(task);
  EXPECT_TRUE(view.state == TaskState::Finished);
  EXPECT_EQ(view.stops_done, 3u);
  ASSERT_EQ(vehicle.finished_actions().size(), 3u);
  EXPECT_EQ(vehicle.finished_actions()[0], "task1-stop0-action0");
  EXPECT_EQ(vehicle.finished_actions()[1], "task1-stop1-action0");
  EXPECT_EQ(vehicle.finished_actions()[2], "task1-stop1-action1");
  EXPECT_EQ(node_of(world, "v1"), "A");
  expect_clean(world);
}

TEST(FleetSim, TwoVehiclesMeetingHeadOnPassAtTheSiding)
{
  sim::World world(test_maps::corridor_with_siding(), fleet_config());
  world.add_vehicle("v1", "A");
  world.add_vehicle("v2", "E");
  world.run_for(1.0);
  world.fleet().submit(go_to("E", "v1"), world.now());
  world.fleet().submit(go_to("A", "v2"), world.now());

  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 180.0));
  EXPECT_EQ(node_of(world, "v1"), "E");
  EXPECT_EQ(node_of(world, "v2"), "A");
  EXPECT_EQ(world.fleet().stats().tasks_finished, 2u);
  EXPECT_GE(world.closest_approach(), 1.3);  // the distance the configuration asks for
  expect_clean(world);
}

TEST(FleetSim, AnIdleVehicleInTheWayIsMovedAside)
{
  sim::World world(test_maps::corridor_with_siding(), fleet_config());
  world.add_vehicle("worker", "A");
  world.add_vehicle("sleeper", "C");
  world.run_for(1.0);
  world.fleet().submit(go_to("E", "worker"), world.now());

  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 120.0));
  EXPECT_EQ(node_of(world, "worker"), "E");
  EXPECT_EQ(node_of(world, "sleeper"), "S");  // it went into the siding
  expect_clean(world);
}

TEST(FleetSim, TasksAreGivenToTheNearestFreeVehicle)
{
  sim::World world(test_maps::corridor(), fleet_config());
  world.add_vehicle("left", "A");
  world.add_vehicle("right", "E");
  world.run_for(1.0);
  const TaskId near_right = *world.fleet().submit(go_to("D"), world.now());
  world.run_for(0.2);
  EXPECT_EQ(world.fleet().task(near_right)->vehicle, "right");
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  expect_clean(world);
}

TEST(FleetSim, ATaskWaitsInTheQueueUntilAVehicleIsFree)
{
  sim::World world(test_maps::corridor(), fleet_config());
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  const TaskId first = *world.fleet().submit(go_to("C"), world.now());
  const TaskId second = *world.fleet().submit(go_to("A"), world.now());
  world.run_for(0.5);
  EXPECT_TRUE(world.fleet().task(first)->state == TaskState::Assigned);
  EXPECT_TRUE(world.fleet().task(second)->state == TaskState::Queued);

  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 120.0));
  EXPECT_TRUE(world.fleet().task(second)->state == TaskState::Finished);
  EXPECT_EQ(node_of(world, "v1"), "A");
  expect_clean(world);
}

TEST(FleetSim, UnusableTasksAreRefused)
{
  sim::World world(test_maps::corridor(), fleet_config());
  EXPECT_FALSE(world.fleet().submit(TaskRequest{}, world.now()).has_value());
  EXPECT_FALSE(world.fleet().submit(go_to("Nowhere"), world.now()).has_value());
  EXPECT_TRUE(world.fleet().tasks().empty());
}
