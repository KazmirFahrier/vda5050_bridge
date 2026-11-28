// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fleet_core/api.hpp"

#include <cmath>
#include <exception>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/json.hpp"

namespace fleet_core
{

namespace
{

using vda5050_core::Json;

/// Thrown for anything wrong with a command; the text goes to the client.
struct CommandError
{
  std::string text;
};

const Json & member(const Json & object, const char * key)
{
  const Json * found = object.find(key);
  if (found == nullptr) {
    throw CommandError{std::string("'") + key + "' is missing"};
  }
  return *found;
}

std::string text_of(const Json & object, const char * key)
{
  const Json & value = member(object, key);
  if (!value.is_string()) {
    throw CommandError{std::string("'") + key + "' must be a string"};
  }
  return value.as_string();
}

std::string optional_text(const Json & object, const char * key)
{
  return object.find(key) == nullptr ? std::string() : text_of(object, key);
}

double number_of(const Json & object, const char * key)
{
  const Json & value = member(object, key);
  if (!value.is_number() || !std::isfinite(value.as_number())) {
    throw CommandError{std::string("'") + key + "' must be a number"};
  }
  return value.as_number();
}

/// A whole number within [low, high].
double whole_number(const Json & object, const char * key, double low, double high)
{
  const double value = number_of(object, key);
  if (value != std::floor(value) || value < low || value > high) {
    throw CommandError{std::string("'") + key + "' is out of range"};
  }
  return value;
}

vda5050_core::Action action_of(const Json & value)
{
  if (!value.is_object()) {
    throw CommandError{"an action must be an object"};
  }
  vda5050_core::Action action;
  action.action_type = text_of(value, "actionType");
  if (action.action_type.empty()) {
    throw CommandError{"'actionType' must not be empty"};
  }
  const std::string blocking = optional_text(value, "blockingType");
  if (blocking.empty() || blocking == "HARD") {
    action.blocking_type = vda5050_core::BlockingType::Hard;
  } else if (blocking == "SOFT") {
    action.blocking_type = vda5050_core::BlockingType::Soft;
  } else if (blocking == "NONE") {
    action.blocking_type = vda5050_core::BlockingType::None;
  } else {
    throw CommandError{"'blockingType' must be NONE, SOFT or HARD"};
  }
  if (value.find("actionDescription") != nullptr) {
    action.action_description = text_of(value, "actionDescription");
  }
  if (const Json * parameters = value.find("actionParameters")) {
    if (!parameters->is_array()) {
      throw CommandError{"'actionParameters' must be an array"};
    }
    for (const Json & parameter : parameters->as_array()) {
      if (!parameter.is_object()) {
        throw CommandError{"an action parameter must be an object"};
      }
      vda5050_core::ActionParameter item;
      item.key = text_of(parameter, "key");
      item.value = member(parameter, "value");
      action.action_parameters.push_back(std::move(item));
    }
  }
  return action;
}

TaskRequest request_of(const Json & command)
{
  TaskRequest request;
  const Json & stops = member(command, "stops");
  if (!stops.is_array()) {
    throw CommandError{"'stops' must be an array"};
  }
  for (const Json & value : stops.as_array()) {
    if (!value.is_object()) {
      throw CommandError{"a stop must be an object"};
    }
    TaskStop stop;
    stop.node = text_of(value, "node");
    if (value.find("dwell") != nullptr) {
      stop.dwell = number_of(value, "dwell");
      if (stop.dwell < 0.0) {
        throw CommandError{"'dwell' must not be negative"};
      }
    }
    if (const Json * actions = value.find("actions")) {
      if (!actions->is_array()) {
        throw CommandError{"'actions' must be an array"};
      }
      for (const Json & action : actions->as_array()) {
        stop.actions.push_back(action_of(action));
      }
    }
    request.stops.push_back(std::move(stop));
  }
  request.vehicle = optional_text(command, "vehicle");
  if (command.find("priority") != nullptr) {
    request.priority = static_cast<int>(whole_number(command, "priority", -1000.0, 1000.0));
  }
  return request;
}

Json carry_out(FleetManager & fleet, const Json & command, TimePoint now)
{
  Json response = Json::Object{};
  const std::string name = text_of(command, "command");
  if (name == "submit") {
    const std::optional<TaskId> id = fleet.submit(request_of(command), now);
    if (!id) {
      throw CommandError{"the task cannot be carried out: no stops, or an unknown node"};
    }
    response["taskId"] = *id;
  } else if (name == "cancel") {
    const double id = whole_number(command, "taskId", 1.0, 9007199254740992.0);
    if (!fleet.cancel(static_cast<TaskId>(id), now)) {
      throw CommandError{"no such task, or it is already over"};
    }
  } else if (name == "resetVehicle") {
    if (!fleet.reset_vehicle(text_of(command, "serialNumber"), now)) {
      throw CommandError{"no such vehicle, or it is not out of service"};
    }
  } else {
    throw CommandError{"unknown command '" + name + "'"};
  }
  response["ok"] = true;
  return response;
}

}  // namespace

const char * to_string(TaskState state)
{
  switch (state) {
    case TaskState::Queued: return "QUEUED";
    case TaskState::Assigned: return "ASSIGNED";
    case TaskState::Finished: return "FINISHED";
    case TaskState::Failed: return "FAILED";
    case TaskState::Cancelled: return "CANCELLED";
  }
  return "FAILED";
}

std::string handle_command(FleetManager & fleet, std::string_view text, TimePoint now)
{
  Json response = Json::Object{};
  std::string request_id;
  try {
    const Json command = Json::parse(text);
    if (!command.is_object()) {
      throw CommandError{"a command must be a JSON object"};
    }
    // The request id is read first, so that even a refusal can be matched.
    if (const Json * id = command.find("requestId")) {
      if (id->is_string()) {
        request_id = id->as_string();
      }
    }
    response = carry_out(fleet, command, now);
  } catch (const CommandError & error) {
    response = Json::Object{};
    response["ok"] = false;
    response["error"] = error.text;
  } catch (const std::exception & error) {
    response = Json::Object{};
    response["ok"] = false;
    response["error"] = std::string("not valid JSON: ") + error.what();
  }
  response["requestId"] = request_id;
  return response.dump();
}

std::string encode_status(const FleetManager & fleet)
{
  Json status = Json::Object{};
  Json vehicles = Json::Array{};
  for (const VehicleView & view : fleet.vehicles()) {
    Json item = Json::Object{};
    item["serialNumber"] = view.serial_number;
    item["manufacturer"] = view.manufacturer;
    item["online"] = view.online;
    item["available"] = view.available;
    item["unavailableReason"] = view.unavailable_reason;
    item["node"] = view.node;
    item["orderId"] = view.order_id;
    item["blocked"] = view.blocked;
    item["grantedMoves"] = view.granted_moves;
    if (view.task) {
      item["taskId"] = *view.task;
    }
    Json claimed = Json::Array{};
    for (const std::string & name : view.claimed) {
      claimed.push_back(name);
    }
    item["claimed"] = std::move(claimed);
    Json route = Json::Array{};
    for (const std::string & node : view.route) {
      route.push_back(node);
    }
    item["route"] = std::move(route);
    vehicles.push_back(std::move(item));
  }
  status["vehicles"] = std::move(vehicles);

  Json tasks = Json::Array{};
  for (const TaskView & view : fleet.tasks()) {
    Json item = Json::Object{};
    item["taskId"] = view.id;
    item["state"] = to_string(view.state);
    item["vehicle"] = view.vehicle;
    item["stopsDone"] = view.stops_done;
    item["stopsTotal"] = view.stops_total;
    item["priority"] = view.priority;
    item["note"] = view.note;
    tasks.push_back(std::move(item));
  }
  status["tasks"] = std::move(tasks);

  const FleetStats stats = fleet.stats();
  Json counters = Json::Object{};
  counters["replans"] = stats.replans;
  counters["searches"] = stats.searches;
  counters["ordersSent"] = stats.orders_sent;
  counters["ordersResent"] = stats.orders_resent;
  counters["tasksFinished"] = stats.tasks_finished;
  counters["tasksFailed"] = stats.tasks_failed;
  counters["invalidPlans"] = stats.invalid_plans;
  status["stats"] = std::move(counters);
  status["mapId"] = fleet.roadmap().map_id();
  return status.dump();
}

std::string encode_event(const FleetEvent & event, double seconds)
{
  Json out = Json::Object{};
  out["time"] = seconds;
  out["kind"] = event.kind;
  out["vehicle"] = event.vehicle;
  out["text"] = event.text;
  return out.dump();
}

}  // namespace fleet_core
