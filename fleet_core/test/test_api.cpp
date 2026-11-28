// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// The command and status interface: what a warehouse system sends and reads.

#include <gtest/gtest.h>

#include <string>

#include "fleet_core/api.hpp"
#include "maps.hpp"
#include "sim.hpp"
#include "vda5050_core/json.hpp"

using namespace fleet_core;  // NOLINT(build/namespaces)
using vda5050_core::Json;

namespace
{

FleetConfig fleet_config()
{
  FleetConfig config;
  config.planner.node_overhead = 0.5;
  return config;
}

Json send(sim::World & world, const std::string & command)
{
  return Json::parse(handle_command(world.fleet(), command, world.now()));
}

std::string error_of(sim::World & world, const std::string & command)
{
  const Json response = send(world, command);
  if (response.find("ok")->as_bool()) {
    EXPECT_EQ(command, "should have been refused");
    return "";
  }
  return response.find("error")->as_string();
}

}  // namespace

TEST(FleetApi, ATaskIsSubmittedCarriedOutAndReported)
{
  sim::World world(test_maps::corridor(), fleet_config());
  sim::SimVehicle & vehicle = world.add_vehicle("v1", "A");
  world.run_for(1.0);

  const Json response = send(
    world,
    R"({"command":"submit","requestId":"r-17","priority":2,"vehicle":"v1",
        "stops":[{"node":"C","dwell":1.5,
                  "actions":[{"actionType":"pick","blockingType":"HARD",
                              "actionDescription":"take the box",
                              "actionParameters":[{"key":"station","value":"S3"},
                                                  {"key":"height","value":0.4}]}]},
                 {"node":"E"}]})");
  EXPECT_TRUE(response.find("ok")->as_bool());
  EXPECT_EQ(response.find("requestId")->as_string(), "r-17");
  EXPECT_DOUBLE_EQ(response.find("taskId")->as_number(), 1.0);

  world.run_for(1.0);
  Json status = Json::parse(encode_status(world.fleet()));
  const Json & task = status.find("tasks")->as_array().at(0);
  EXPECT_EQ(task.find("state")->as_string(), "ASSIGNED");
  EXPECT_EQ(task.find("vehicle")->as_string(), "v1");
  EXPECT_DOUBLE_EQ(task.find("priority")->as_number(), 2.0);
  EXPECT_DOUBLE_EQ(task.find("stopsTotal")->as_number(), 2.0);
  const Json & shown = status.find("vehicles")->as_array().at(0);
  EXPECT_EQ(shown.find("serialNumber")->as_string(), "v1");
  EXPECT_EQ(shown.find("manufacturer")->as_string(), "sim");
  EXPECT_TRUE(shown.find("online")->as_bool());
  EXPECT_TRUE(shown.find("available")->as_bool());
  EXPECT_DOUBLE_EQ(shown.find("taskId")->as_number(), 1.0);
  EXPECT_EQ(shown.find("route")->as_array().back().as_string(), "E");
  EXPECT_FALSE(shown.find("claimed")->as_array().empty());
  EXPECT_EQ(status.find("mapId")->as_string(), "map");

  // The action reaches the vehicle as it was given.
  const vda5050_core::Order & order = world.orders().front();
  const vda5050_core::Action * sent = nullptr;
  for (const vda5050_core::Node & node : order.nodes) {
    if (!node.actions.empty()) {
      EXPECT_EQ(node.node_id, "C");
      sent = &node.actions.front();
    }
  }
  ASSERT_TRUE(sent != nullptr);
  EXPECT_EQ(sent->action_type, "pick");
  EXPECT_EQ(*sent->action_description, "take the box");
  ASSERT_EQ(sent->action_parameters.size(), 2u);
  EXPECT_EQ(sent->action_parameters[0].key, "station");
  EXPECT_EQ(sent->action_parameters[0].value.as_string(), "S3");
  EXPECT_DOUBLE_EQ(sent->action_parameters[1].value.as_number(), 0.4);

  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 60.0));
  status = Json::parse(encode_status(world.fleet()));
  EXPECT_EQ(status.find("tasks")->as_array().at(0).find("state")->as_string(), "FINISHED");
  EXPECT_DOUBLE_EQ(status.find("stats")->find("tasksFinished")->as_number(), 1.0);
  EXPECT_DOUBLE_EQ(status.find("stats")->find("invalidPlans")->as_number(), 0.0);
  EXPECT_GT(status.find("stats")->find("ordersSent")->as_number(), 0.0);
  EXPECT_EQ(vehicle.actions_started(), 1u);
  EXPECT_TRUE(
    status.find("vehicles")->as_array().at(0).find("taskId") == nullptr);
}

TEST(FleetApi, CancelAndResetAnswerTruthfully)
{
  sim::World world(test_maps::corridor(), fleet_config());
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  send(world, R"({"command":"submit","stops":[{"node":"E"}]})");
  Json response = send(world, R"({"command":"cancel","requestId":"c1","taskId":1})");
  EXPECT_TRUE(response.find("ok")->as_bool());
  EXPECT_EQ(response.find("requestId")->as_string(), "c1");
  EXPECT_EQ(
    error_of(world, R"({"command":"cancel","taskId":1})"),
    "no such task, or it is already over");
  EXPECT_EQ(
    error_of(world, R"({"command":"cancel","taskId":77})"),
    "no such task, or it is already over");
  EXPECT_EQ(
    error_of(world, R"({"command":"resetVehicle","serialNumber":"v1"})"),
    "no such vehicle, or it is not out of service");
  world.run_for(5.0);
  const Json status = Json::parse(encode_status(world.fleet()));
  EXPECT_EQ(status.find("tasks")->as_array().at(0).find("state")->as_string(), "CANCELLED");
}

TEST(FleetApi, EveryMalformedCommandIsRefusedWithAReason)
{
  sim::World world(test_maps::corridor(), fleet_config());
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  const auto refused = [&world](const std::string & command, const std::string & reason) {
      const std::string error = error_of(world, command);
      EXPECT_NE(error.find(reason), std::string::npos);
      if (error.find(reason) == std::string::npos) {
        EXPECT_EQ(error, reason);  // shows both texts
      }
    };
  refused("not json at all", "not valid JSON");
  refused("[1,2]", "must be a JSON object");
  refused("{}", "'command' is missing");
  refused(R"({"command":7})", "'command' must be a string");
  refused(R"({"command":"dance"})", "unknown command 'dance'");
  refused(R"({"command":"submit"})", "'stops' is missing");
  refused(R"({"command":"submit","stops":{}})", "'stops' must be an array");
  refused(R"({"command":"submit","stops":[]})", "cannot be carried out");
  refused(R"({"command":"submit","stops":["A"]})", "a stop must be an object");
  refused(R"({"command":"submit","stops":[{}]})", "'node' is missing");
  refused(R"({"command":"submit","stops":[{"node":"Nowhere"}]})", "cannot be carried out");
  refused(R"({"command":"submit","stops":[{"node":"B"}],"vehicle":3})", "'vehicle' must be");
  refused(R"({"command":"submit","stops":[{"node":"B","dwell":"long"}]})", "'dwell' must be");
  refused(R"({"command":"submit","stops":[{"node":"B","dwell":-1}]})", "must not be negative");
  refused(R"({"command":"submit","stops":[{"node":"B"}],"priority":1.5})", "out of range");
  refused(R"({"command":"submit","stops":[{"node":"B"}],"priority":1e9})", "out of range");
  refused(R"({"command":"submit","stops":[{"node":"B","actions":7}]})", "'actions' must be");
  refused(R"({"command":"submit","stops":[{"node":"B","actions":[7]}]})", "must be an object");
  refused(
    R"({"command":"submit","stops":[{"node":"B","actions":[{}]}]})", "'actionType' is missing");
  refused(
    R"({"command":"submit","stops":[{"node":"B","actions":[{"actionType":""}]}]})",
    "must not be empty");
  refused(
    R"({"command":"submit","stops":[{"node":"B","actions":[
        {"actionType":"pick","blockingType":"MAYBE"}]}]})",
    "NONE, SOFT or HARD");
  refused(
    R"({"command":"submit","stops":[{"node":"B","actions":[
        {"actionType":"pick","actionParameters":{}}]}]})",
    "'actionParameters' must be");
  refused(
    R"({"command":"submit","stops":[{"node":"B","actions":[
        {"actionType":"pick","actionParameters":[1]}]}]})",
    "parameter must be an object");
  refused(
    R"({"command":"submit","stops":[{"node":"B","actions":[
        {"actionType":"pick","actionParameters":[{"key":"k"}]}]}]})",
    "'value' is missing");
  refused(R"({"command":"cancel"})", "'taskId' is missing");
  refused(R"({"command":"cancel","taskId":"one"})", "'taskId' must be a number");
  refused(R"({"command":"cancel","taskId":-3})", "out of range");
  refused(R"({"command":"resetVehicle"})", "'serialNumber' is missing");
  // Nothing of all that was accepted.
  EXPECT_TRUE(world.fleet().tasks().empty());

  // A task for a vehicle that is not known (yet) is accepted and waits.
  EXPECT_TRUE(
    send(world, R"({"command":"submit","stops":[{"node":"B"}],"vehicle":"v9"})")
    .find("ok")->as_bool());
  world.run_for(3.0);
  EXPECT_TRUE(world.fleet().tasks().front().state == TaskState::Queued);

  // The request id comes back even with a refusal, if it could be read.
  const Json response = send(world, R"({"requestId":"x9","command":"dance"})");
  EXPECT_EQ(response.find("requestId")->as_string(), "x9");
  EXPECT_EQ(send(world, R"({"requestId":5,"command":"dance"})").find("requestId")->as_string(),
    "");
}

TEST(FleetApi, SoftAndNonBlockingActionsAreAccepted)
{
  sim::World world(test_maps::corridor(), fleet_config());
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  const Json response = send(
    world,
    R"({"command":"submit","stops":[{"node":"B","actions":[
        {"actionType":"wait","blockingType":"SOFT"},
        {"actionType":"wait","blockingType":"NONE"},
        {"actionType":"wait"}]}]})");
  EXPECT_TRUE(response.find("ok")->as_bool());
  world.run_for(0.5);
  const vda5050_core::Node & stop = world.orders().front().nodes.back();
  ASSERT_EQ(stop.actions.size(), 3u);
  EXPECT_TRUE(stop.actions[0].blocking_type == vda5050_core::BlockingType::Soft);
  EXPECT_TRUE(stop.actions[1].blocking_type == vda5050_core::BlockingType::None);
  EXPECT_TRUE(stop.actions[2].blocking_type == vda5050_core::BlockingType::Hard);
}

TEST(FleetApi, EventsAndTaskStatesHaveNames)
{
  FleetEvent event;
  event.kind = "task_finished";
  event.vehicle = "v1";
  event.text = "task 4";
  const Json out = Json::parse(encode_event(event, 12.5));
  EXPECT_DOUBLE_EQ(out.find("time")->as_number(), 12.5);
  EXPECT_EQ(out.find("kind")->as_string(), "task_finished");
  EXPECT_EQ(out.find("vehicle")->as_string(), "v1");
  EXPECT_EQ(out.find("text")->as_string(), "task 4");
  EXPECT_EQ(std::string(to_string(TaskState::Queued)), "QUEUED");
  EXPECT_EQ(std::string(to_string(TaskState::Failed)), "FAILED");
}

TEST(FleetApi, OnlyTheLatestFinishedTasksAreKept)
{
  FleetConfig config = fleet_config();
  config.finished_tasks_kept = 3;
  sim::World world(test_maps::corridor(), config);
  world.add_vehicle("v1", "A");
  world.run_for(1.0);
  for (int k = 0; k < 6; ++k) {
    send(world, k % 2 == 0 ? R"({"command":"submit","stops":[{"node":"B"}]})" :
      R"({"command":"submit","stops":[{"node":"A"}]})");
  }
  ASSERT_TRUE(world.run_until([&] {return world.all_tasks_over();}, 200.0));
  world.run_for(0.5);
  const std::vector<TaskView> kept = world.fleet().tasks();
  ASSERT_EQ(kept.size(), 3u);
  EXPECT_EQ(kept.front().id, 4u);
  EXPECT_EQ(kept.back().id, 6u);
  EXPECT_EQ(world.fleet().stats().tasks_finished, 6u);
  // A task that is not over is never forgotten, however old.
  EXPECT_TRUE(send(world, R"({"command":"cancel","taskId":1})").find("ok")->as_bool() == false);
}
