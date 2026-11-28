// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "fleet_core/fleet_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "vda5050_core/codec.hpp"

namespace fleet_core
{

namespace
{

constexpr double kEpsilon = 1e-9;

using vda5050_core::ActionStatus;
using vda5050_core::ErrorLevel;
using vda5050_core::State;

/// The vehicle is not driving and has nothing released left to do.
bool idle(const State & state)
{
  if (state.driving) {
    return false;
  }
  for (const vda5050_core::NodeState & node : state.node_states) {
    if (node.released) {
      return false;
    }
  }
  for (const vda5050_core::EdgeState & edge : state.edge_states) {
    if (edge.released) {
      return false;
    }
  }
  return std::all_of(
    state.action_states.begin(), state.action_states.end(),
    [](const vda5050_core::ActionState & action) {
      return vda5050_core::is_terminal(action.action_status);
    });
}

/// Idle, and not even a preview (horizon) of more is left: only now does the
/// vehicle accept an order with a new orderId.
bool clean_idle(const State & state)
{
  return idle(state) && state.node_states.empty() && state.edge_states.empty();
}

/// Split "<interface>/<version>/<manufacturer>/<serial>/<topic>".
bool split_topic(const std::string & topic, std::vector<std::string> & parts)
{
  parts.clear();
  std::size_t begin = 0;
  while (true) {
    const std::size_t slash = topic.find('/', begin);
    if (slash == std::string::npos) {
      parts.push_back(topic.substr(begin));
      break;
    }
    parts.push_back(topic.substr(begin, slash - begin));
    begin = slash + 1;
  }
  return parts.size() == 5;
}

}  // namespace

// ------------------------------------------------------------ internals ----

struct GrantedMove
{
  int edge{0};
  int from{0};
  int to{0};
  std::uint32_t sequence_to{0};  ///< sequenceId of the node `to` in the order.
  bool sent{false};              ///< Part of an order the vehicle has confirmed.
  std::size_t stops{0};          ///< Task stops reached when this move is done.
};

struct PendingOrder
{
  std::string payload;
  std::uint32_t update_id{0};
  TimePoint sent_at{};
  std::size_t start_stops{0};    ///< Task stops at the first node of a new order.
  std::vector<int> horizon;
};

struct FleetManager::Task
{
  TaskId id{0};
  TaskRequest request;
  std::vector<int> nodes;        ///< Node index of each stop.
  std::vector<double> dwell;     ///< Planning estimate per stop, seconds.
  std::vector<std::vector<std::string>> action_ids;
  TaskState state{TaskState::Queued};
  int vehicle{-1};
  /// Stops, in order, go through three counters:
  std::size_t dispatched{0};  ///< actions handed to the vehicle in an order
  std::size_t reached{0};     ///< vehicle is (or was) at the stop
  std::size_t done{0};        ///< its actions have finished
  std::uint64_t sequence{0};  ///< Submission order.
  std::string note;
  bool blocked{false};
};

struct FleetManager::Vehicle
{
  Vehicle(int index_in, vda5050_core::Identity identity_in, vda5050_core::HeaderFactory::Clock clock)
  : index(index_in), identity(identity_in), headers(std::move(identity_in), std::move(clock)) {}

  int index;
  vda5050_core::Identity identity;
  vda5050_core::HeaderFactory headers;

  bool online{false};
  TimePoint online_since{};
  std::optional<TimePoint> state_requested_at;
  bool discovery_given_up{false};
  bool has_state{false};
  State state;
  TimePoint last_state_time{};

  /// Where the vehicle is for the purpose of claims: the node it stands on,
  /// or the node it left last while driving a granted move. -1: not on the map.
  int node{-1};
  std::deque<GrantedMove> committed;   ///< Granted, not yet completed.
  std::vector<PlannedMove> plan;       ///< Planned, not yet granted.
  /// For each task stop not yet dispatched: after how many moves of `plan`
  /// the vehicle is at it.
  std::vector<std::size_t> plan_stop_after;
  bool plan_blocked{false};
  TaskId task{0};

  // The order as the vehicle knows it.
  bool journey_open{false};
  std::string order_id;
  std::uint32_t order_update_id{0};
  std::uint32_t grant_sequence{0};     ///< sequenceId of the last granted node.
  int stitch_node{-1};                 ///< Last node of what has been sent.
  std::uint32_t stitch_sequence{0};
  std::optional<PendingOrder> pending;
  std::vector<int> sent_horizon;

  bool available{false};
  std::string unavailable_reason{"no state received yet"};
  bool needs_resync{false};
  std::optional<TimePoint> cancel_sent_at;
  std::uint32_t cancel_count{0};
  bool out_of_service{false};
  bool off_road{false};                ///< Stopped away from the roadmap.
  /// Set while a vehicle that stopped on an edge is being sent on: how far
  /// from the first node of its next order it may be.
  double recovery_deviation{0.0};
  int navigation_failures{0};
  std::string handled_failure;         ///< orderId whose navigation failure was counted.
};

// ---------------------------------------------------------------- set-up ----

FleetManager::FleetManager(
  Roadmap roadmap, FleetConfig config, vda5050_core::MqttTransport & transport,
  vda5050_core::HeaderFactory::Clock clock)
: roadmap_(std::move(roadmap)), config_(std::move(config)),
  conflicts_(roadmap_, 2.0 * config_.vehicle_radius + config_.safety_margin),
  transport_(transport), clock_(std::move(clock)) {}

FleetManager::~FleetManager() = default;

std::vector<std::string> FleetManager::subscriptions() const
{
  const std::string prefix =
    config_.interface_name + "/v" + config_.version.substr(0, config_.version.find('.')) + "/+/+/";
  return {prefix + "state", prefix + "connection"};
}

void FleetManager::event(
  TimePoint now, const std::string & kind, const std::string & vehicle, const std::string & text)
{
  events_.push_back(FleetEvent{now, kind, vehicle, text});
  if (events_.size() > 10000) {
    events_.erase(events_.begin(), events_.begin() + 5000);
  }
}

std::vector<FleetEvent> FleetManager::take_events()
{
  return std::exchange(events_, {});
}

FleetManager::Vehicle * FleetManager::find_vehicle(
  const std::string & manufacturer, const std::string & serial)
{
  for (const std::unique_ptr<Vehicle> & vehicle : vehicles_) {
    if (vehicle->identity.serial_number == serial &&
      vehicle->identity.manufacturer == manufacturer)
    {
      return vehicle.get();
    }
  }
  return nullptr;
}

FleetManager::Vehicle & FleetManager::vehicle_for(
  const std::string & manufacturer, const std::string & serial)
{
  if (Vehicle * known = find_vehicle(manufacturer, serial)) {
    return *known;
  }
  vda5050_core::Identity identity;
  identity.interface_name = config_.interface_name;
  identity.version = config_.version;
  identity.manufacturer = manufacturer;
  identity.serial_number = serial;
  vehicles_.push_back(
    std::make_unique<Vehicle>(static_cast<int>(vehicles_.size()), identity, clock_));
  return *vehicles_.back();
}

FleetManager::Task * FleetManager::find_task(TaskId id)
{
  for (Task & task : tasks_) {
    if (task.id == id) {
      return &task;
    }
  }
  return nullptr;
}

const FleetManager::Task * FleetManager::find_task(TaskId id) const
{
  for (const Task & task : tasks_) {
    if (task.id == id) {
      return &task;
    }
  }
  return nullptr;
}

// ------------------------------------------------------------- accounting ----

int FleetManager::base_end(const Vehicle & vehicle) const
{
  return vehicle.committed.empty() ? vehicle.node : vehicle.committed.back().to;
}

std::vector<ResourceId> FleetManager::claimed(const Vehicle & vehicle) const
{
  std::vector<ResourceId> result;
  if (vehicle.node < 0) {
    return result;
  }
  result.push_back(roadmap_.node_resource(vehicle.node));
  for (const GrantedMove & move : vehicle.committed) {
    result.push_back(roadmap_.edge_resource(move.edge));
    result.push_back(roadmap_.node_resource(move.to));
  }
  return result;
}

bool FleetManager::settled(const Vehicle & vehicle) const
{
  return vehicle.committed.empty() && !vehicle.pending && !vehicle.needs_resync &&
         vehicle.has_state && clean_idle(vehicle.state);
}

// --------------------------------------------------------------- incoming ----

void FleetManager::on_message(const std::string & topic, std::string_view payload, TimePoint now)
{
  std::vector<std::string> parts;
  if (!split_topic(topic, parts) || parts[0] != config_.interface_name) {
    return;
  }
  const std::string & manufacturer = parts[2];
  const std::string & serial = parts[3];
  if (manufacturer.empty() || serial.empty()) {
    return;
  }
  try {
    if (parts[4] == "state") {
      const State state = vda5050_core::decode_state(payload);
      on_state(vehicle_for(manufacturer, serial), state, now);
    } else if (parts[4] == "connection") {
      const vda5050_core::Connection connection = vda5050_core::decode_connection(payload);
      on_connection(vehicle_for(manufacturer, serial), connection.connection_state, now);
    }
  } catch (const vda5050_core::DecodeError & error) {
    event(now, "bad_message", serial, std::string(topic) + ": " + error.what());
  }
}

void FleetManager::on_connection(
  Vehicle & vehicle, vda5050_core::ConnectionState state, TimePoint now)
{
  const bool online = state == vda5050_core::ConnectionState::Online;
  if (online != vehicle.online) {
    vehicle.online = online;
    vehicle.online_since = now;
    vehicle.discovery_given_up = false;
    event(
      now, online ? "vehicle_online" : "vehicle_offline", vehicle.identity.serial_number,
      vda5050_core::to_string(state));
  }
  update_availability(vehicle, now);
}

void FleetManager::on_state(Vehicle & vehicle, const State & state, TimePoint now)
{
  const bool first = !vehicle.has_state;
  vehicle.state = state;
  vehicle.has_state = true;
  vehicle.last_state_time = now;
  if (!vehicle.online) {
    // A vehicle that reports state is evidently connected, whatever a stale
    // retained connection message said.
    vehicle.online = true;
    vehicle.online_since = now;
    event(now, "vehicle_online", vehicle.identity.serial_number, "state received");
  }
  if (first && !clean_idle(state)) {
    // It is in the middle of something this fleet manager did not send.
    resynchronize(vehicle, "vehicle is busy with an order from before", now);
  }

  check_order_feedback(vehicle, now);
  track_progress(vehicle, now);
  track_task(vehicle, now);
  locate(vehicle, now);

  if (vehicle.needs_resync && clean_idle(state)) {
    finish_resynchronization(vehicle, now);
  }
  update_availability(vehicle, now);
}

void FleetManager::finish_resynchronization(Vehicle & vehicle, TimePoint now)
{
  // The vehicle has come to rest. Where? A cancelled vehicle stops where it
  // is, which may be in the middle of an edge.
  const State & state = vehicle.state;
  std::optional<GrantedMove> underway;
  if (!vehicle.committed.empty() && vehicle.committed.front().sent) {
    underway = vehicle.committed.front();
  }
  vehicle.committed.clear();
  vehicle.plan.clear();
  vehicle.plan_stop_after.clear();
  vehicle.pending.reset();
  vehicle.sent_horizon.clear();
  vehicle.journey_open = false;
  vehicle.needs_resync = false;
  vehicle.cancel_sent_at.reset();
  vehicle.recovery_deviation = 0.0;
  vehicle.off_road = false;

  const bool has_position = state.agv_position && state.agv_position->position_initialized;
  std::optional<int> at_node;
  if (has_position) {
    at_node = roadmap_.nearest_node(
      state.agv_position->x, state.agv_position->y, config_.attach_tolerance);
  } else {
    at_node = roadmap_.find_node(state.last_node_id);
  }

  if (at_node) {
    vehicle.node = -1;
    place(vehicle, *at_node, now);
  } else if (underway && has_position) {
    // Between two nodes. It keeps the edge it is on and both its ends, and is
    // sent on to the end of that edge with an order of its own, whose first
    // node allows for the distance it has already covered.
    const MapNode & from = roadmap_.node(underway->from);
    const MapNode & to = roadmap_.node(underway->to);
    const double off_line = segment_distance(
      state.agv_position->x, state.agv_position->y, state.agv_position->x,
      state.agv_position->y, from.x, from.y, to.x, to.y);
    vehicle.node = underway->from;
    GrantedMove rest = *underway;
    rest.stops = 0;
    if (off_line <= config_.attach_tolerance) {
      rest.sent = false;
      rest.sequence_to = 2;
      vehicle.recovery_deviation =
        roadmap_.edge(rest.edge).length + config_.attach_tolerance;
      event(
        now, "vehicle_recovering", vehicle.identity.serial_number,
        "stopped on edge " + roadmap_.edge(rest.edge).id + "; sent on to its end");
    } else {
      // Not even on the edge. Nothing can be planned for it; what it last
      // held stays blocked until somebody puts it back on a node.
      vehicle.off_road = true;
      event(
        now, "vehicle_off_road", vehicle.identity.serial_number,
        "stopped away from the roadmap");
    }
    vehicle.committed.push_back(rest);
  } else {
    vehicle.node = -1;
    event(
      now, "vehicle_not_on_map", vehicle.identity.serial_number,
      "came to rest away from every node");
  }

  if (Task * task = find_task(vehicle.task)) {
    task->dispatched = task->reached;  // what was not reached is planned again
  }
  plan_dirty_ = true;
  event(now, "vehicle_resynchronized", vehicle.identity.serial_number, "");
}

void FleetManager::locate(Vehicle & vehicle, TimePoint now)
{
  const State & state = vehicle.state;
  std::optional<int> where;
  if (state.agv_position && state.agv_position->position_initialized) {
    where = roadmap_.nearest_node(
      state.agv_position->x, state.agv_position->y, config_.attach_tolerance);
  } else if (!state.agv_position) {
    where = roadmap_.find_node(state.last_node_id);  // no coordinates: trust the node id
  }
  if (vehicle.off_road) {
    if (where) {  // somebody has put it back on a node
      vehicle.off_road = false;
      vehicle.committed.clear();
      place(vehicle, *where, now);
    }
    return;
  }
  if (!vehicle.committed.empty() || vehicle.pending || vehicle.needs_resync) {
    return;  // while it drives, its place follows from its progress
  }
  if (where && *where != vehicle.node) {
    place(vehicle, *where, now);
  }
}

void FleetManager::place(Vehicle & vehicle, int node, TimePoint now)
{
  vehicle.node = node;
  plan_dirty_ = true;
  // A vehicle has turned up at a place nobody knew it was. If that place is
  // on the way of a vehicle that has already been given it, the only thing
  // left to do is to stop that vehicle.
  for (const std::unique_ptr<Vehicle> & owner : vehicles_) {
    Vehicle & other = *owner;
    if (&other == &vehicle) {
      continue;
    }
    bool in_the_way = false;
    for (const GrantedMove & move : other.committed) {
      in_the_way = in_the_way ||
        conflicts_.conflict(roadmap_.node_resource(node), roadmap_.edge_resource(move.edge)) ||
        conflicts_.conflict(roadmap_.node_resource(node), roadmap_.node_resource(move.to));
    }
    if (in_the_way) {
      event(
        now, "claim_conflict", other.identity.serial_number,
        vehicle.identity.serial_number + " appeared at " + roadmap_.node(node).id +
        ", which is on its route");
      resynchronize(other, "another vehicle appeared on its route", now);
      update_availability(other, now);
    }
  }
}

void FleetManager::check_order_feedback(Vehicle & vehicle, TimePoint now)
{
  const State & state = vehicle.state;
  const std::string & serial = vehicle.identity.serial_number;

  if (vehicle.pending) {
    if (state.order_id == vehicle.order_id && state.order_update_id >= vehicle.pending->update_id) {
      // Confirmed: the vehicle now knows everything granted so far.
      for (GrantedMove & move : vehicle.committed) {
        move.sent = true;
      }
      vehicle.sent_horizon = vehicle.pending->horizon;
      if (Task * task = find_task(vehicle.task)) {
        task->reached += vehicle.pending->start_stops;
      }
      vehicle.pending.reset();
    } else {
      for (const vda5050_core::Error & error : state.errors) {
        bool same_order = false;
        bool same_update = false;
        for (const vda5050_core::ErrorReference & reference : error.error_references) {
          same_order = same_order ||
            (reference.reference_key == "orderId" && reference.reference_value == vehicle.order_id);
          same_update = same_update ||
            (reference.reference_key == "orderUpdateId" &&
            reference.reference_value == std::to_string(vehicle.pending->update_id));
        }
        if (same_order && same_update) {
          event(now, "order_rejected", serial, error.error_type + ": " + error.error_description);
          resynchronize(vehicle, "order rejected", now);
          return;
        }
      }
    }
  }

  if (vehicle.needs_resync || !vehicle.journey_open || vehicle.pending) {
    return;
  }
  // The vehicle must be on the order it was given, and must not have dropped it.
  const bool has_sent_moves = std::any_of(
    vehicle.committed.begin(), vehicle.committed.end(),
    [](const GrantedMove & move) {return move.sent;});
  if (state.order_id != vehicle.order_id) {
    event(now, "order_lost", serial, "vehicle reports order '" + state.order_id + "'");
    resynchronize(vehicle, "vehicle is on another order", now);
    return;
  }
  for (const vda5050_core::Error & error : state.errors) {
    if (error.error_level != ErrorLevel::Fatal || error.error_type != "navigationError" ||
      vehicle.handled_failure == vehicle.order_id)
    {
      continue;
    }
    vehicle.handled_failure = vehicle.order_id;
    ++vehicle.navigation_failures;
    event(now, "navigation_failed", serial, error.error_description);
    if (vehicle.navigation_failures >= config_.max_navigation_failures) {
      vehicle.out_of_service = true;
      event(now, "vehicle_out_of_service", serial, "too many navigation failures in a row");
      if (Task * task = find_task(vehicle.task)) {
        finish_task(*task, TaskState::Failed, "vehicle taken out of service", now);
      }
    }
    resynchronize(vehicle, "navigation failed", now);
    return;
  }
  if (has_sent_moves && clean_idle(state)) {
    const GrantedMove & next = vehicle.committed.front();
    if (state.last_node_sequence_id < next.sequence_to) {
      event(now, "order_abandoned", serial, "vehicle stopped before the end of its order");
      resynchronize(vehicle, "vehicle stopped early", now);
    }
  }
}

void FleetManager::track_progress(Vehicle & vehicle, TimePoint now)
{
  (void)now;
  if (!vehicle.journey_open || vehicle.state.order_id != vehicle.order_id) {
    return;
  }
  Task * task = find_task(vehicle.task);
  while (!vehicle.committed.empty() && vehicle.committed.front().sent &&
    vehicle.state.last_node_sequence_id >= vehicle.committed.front().sequence_to)
  {
    const GrantedMove done = vehicle.committed.front();
    vehicle.committed.pop_front();
    vehicle.node = done.to;
    vehicle.navigation_failures = 0;
    if (task != nullptr) {
      task->reached += done.stops;
    }
  }
}

void FleetManager::track_task(Vehicle & vehicle, TimePoint now)
{
  Task * task = find_task(vehicle.task);
  if (task == nullptr || task->state != TaskState::Assigned ||
    vehicle.state.order_id != vehicle.order_id)
  {
    return;
  }
  while (task->done < task->reached) {
    bool finished = true;
    for (const std::string & id : task->action_ids[task->done]) {
      const auto found = std::find_if(
        vehicle.state.action_states.begin(), vehicle.state.action_states.end(),
        [&id](const vda5050_core::ActionState & action) {return action.action_id == id;});
      if (found == vehicle.state.action_states.end()) {
        finished = false;
      } else if (found->action_status == ActionStatus::Failed) {
        const std::string why = "action " + found->action_type + " failed" +
          (found->result_description ? ": " + *found->result_description : std::string());
        finish_task(*task, TaskState::Failed, why, now);
        return;
      } else if (found->action_status != ActionStatus::Finished) {
        finished = false;
      }
    }
    if (!finished) {
      return;
    }
    ++task->done;
  }
  if (task->done == task->nodes.size()) {
    finish_task(*task, TaskState::Finished, "", now);
  }
}

void FleetManager::finish_task(
  Task & task, TaskState state, const std::string & note, TimePoint now)
{
  task.state = state;
  task.note = note;
  task.blocked = false;
  std::string serial;
  if (task.vehicle >= 0) {
    Vehicle & vehicle = *vehicles_[static_cast<std::size_t>(task.vehicle)];
    serial = vehicle.identity.serial_number;
    vehicle.task = 0;
    vehicle.plan.clear();
    vehicle.plan_stop_after.clear();
    vehicle.plan_blocked = false;
  }
  if (state == TaskState::Finished) {
    ++stats_.tasks_finished;
    event(now, "task_finished", serial, "task " + std::to_string(task.id));
  } else if (state == TaskState::Failed) {
    ++stats_.tasks_failed;
    event(now, "task_failed", serial, "task " + std::to_string(task.id) + ": " + note);
  } else {
    event(now, "task_cancelled", serial, "task " + std::to_string(task.id));
  }
  plan_dirty_ = true;
}

void FleetManager::resynchronize(Vehicle & vehicle, const std::string & why, TimePoint now)
{
  if (vehicle.needs_resync) {
    return;
  }
  vehicle.needs_resync = true;
  vehicle.plan.clear();
  vehicle.plan_stop_after.clear();
  vehicle.cancel_sent_at.reset();
  plan_dirty_ = true;
  event(now, "vehicle_resync", vehicle.identity.serial_number, why);
  if (!clean_idle(vehicle.state)) {
    send_cancel(vehicle, now);
  }
}

void FleetManager::send_cancel(Vehicle & vehicle, TimePoint now)
{
  vda5050_core::InstantActions message;
  message.header = vehicle.headers.next("instantActions");
  vda5050_core::Action cancel;
  cancel.action_type = "cancelOrder";
  cancel.action_id = "fleet-cancel-" + std::to_string(vehicle.index) + "-" +
    std::to_string(++vehicle.cancel_count);
  cancel.blocking_type = vda5050_core::BlockingType::Hard;
  message.actions.push_back(cancel);
  transport_.publish(
    vda5050_core::MqttMessage{
      vda5050_core::topic_name(vehicle.identity, "instantActions"),
      vda5050_core::encode_instant_actions(message), 0, false});
  vehicle.cancel_sent_at = now;
}

void FleetManager::send_state_request(Vehicle & vehicle, TimePoint now)
{
  vda5050_core::InstantActions message;
  message.header = vehicle.headers.next("instantActions");
  vda5050_core::Action request;
  request.action_type = "stateRequest";
  request.action_id = "fleet-state-" + std::to_string(vehicle.index) + "-" +
    std::to_string(++vehicle.cancel_count);
  request.blocking_type = vda5050_core::BlockingType::None;
  message.actions.push_back(request);
  transport_.publish(
    vda5050_core::MqttMessage{
      vda5050_core::topic_name(vehicle.identity, "instantActions"),
      vda5050_core::encode_instant_actions(message), 0, false});
  vehicle.state_requested_at = now;
}

bool FleetManager::waiting_for_discovery(TimePoint now)
{
  bool waiting = false;
  for (const std::unique_ptr<Vehicle> & owner : vehicles_) {
    Vehicle & vehicle = *owner;
    if (!vehicle.online || vehicle.node >= 0 || vehicle.off_road) {
      continue;
    }
    if (now - vehicle.online_since < config_.discovery_grace) {
      waiting = true;
      // Its first state may have been lost on the way: ask for another.
      if (!vehicle.state_requested_at ||
        now - *vehicle.state_requested_at >= config_.order_resend)
      {
        send_state_request(vehicle, now);
      }
    } else if (!vehicle.discovery_given_up) {
      vehicle.discovery_given_up = true;
      event(
        now, "vehicle_unlocated", vehicle.identity.serial_number,
        "online, but its place on the roadmap is unknown; carrying on without it");
    }
  }
  return waiting;
}

void FleetManager::update_availability(Vehicle & vehicle, TimePoint now)
{
  std::string reason;
  const State & state = vehicle.state;
  if (!vehicle.has_state) {
    reason = "no state received yet";
  } else if (!vehicle.online) {
    reason = "offline";
  } else if (now - vehicle.last_state_time > config_.state_timeout) {
    reason = "no state for too long";
  } else if (vehicle.out_of_service) {
    reason = "taken out of service";
  } else if (vehicle.off_road) {
    reason = "stopped away from the roadmap";
  } else if (vehicle.node < 0) {
    reason = "not at a node of the roadmap";
  } else if (vehicle.needs_resync) {
    reason = "being brought to a halt";
  } else if (state.operating_mode != vda5050_core::OperatingMode::Automatic &&
    state.operating_mode != vda5050_core::OperatingMode::Semiautomatic)
  {
    reason = std::string("operating mode ") + vda5050_core::to_string(state.operating_mode);
  } else if (state.paused) {
    reason = "paused";
  } else {
    for (const vda5050_core::Error & error : state.errors) {
      // A navigation failure is over once the vehicle has been halted; the
      // error stays in its state until the next order and must not lock it out.
      const bool stale = error.error_type == "navigationError" && clean_idle(state);
      if (error.error_level == ErrorLevel::Fatal && !stale) {
        reason = "fatal error " + error.error_type;
        break;
      }
    }
  }
  const bool available = reason.empty();
  if (available != vehicle.available || reason != vehicle.unavailable_reason) {
    if (available != vehicle.available) {
      event(
        now, available ? "vehicle_available" : "vehicle_unavailable",
        vehicle.identity.serial_number, reason);
      plan_dirty_ = true;
    }
    vehicle.available = available;
    vehicle.unavailable_reason = reason;
  }
}

// ------------------------------------------------------------------ tasks ----

std::optional<TaskId> FleetManager::submit(const TaskRequest & request, TimePoint now)
{
  if (request.stops.empty()) {
    event(now, "task_refused", "", "a task needs at least one stop");
    return std::nullopt;
  }
  Task task;
  task.request = request;
  for (std::size_t k = 0; k < request.stops.size(); ++k) {
    const TaskStop & stop = request.stops[k];
    const std::optional<int> node = roadmap_.find_node(stop.node);
    if (!node) {
      event(now, "task_refused", "", "unknown node '" + stop.node + "'");
      return std::nullopt;
    }
    task.nodes.push_back(*node);
    task.dwell.push_back(
      stop.dwell >= 0.0 ? stop.dwell : (stop.actions.empty() ? 0.0 : config_.default_dwell));
    task.action_ids.emplace_back();
  }
  task.id = next_task_id_++;
  task.sequence = task.id;
  // Action ids must be unique for the vehicle; the task and stop make them so.
  for (std::size_t k = 0; k < task.request.stops.size(); ++k) {
    for (std::size_t a = 0; a < task.request.stops[k].actions.size(); ++a) {
      const std::string id = "task" + std::to_string(task.id) + "-stop" + std::to_string(k) +
        "-action" + std::to_string(a);
      task.request.stops[k].actions[a].action_id = id;
      task.action_ids[k].push_back(id);
    }
  }
  tasks_.push_back(std::move(task));
  plan_dirty_ = true;
  event(now, "task_queued", "", "task " + std::to_string(tasks_.back().id));
  return tasks_.back().id;
}

bool FleetManager::cancel(TaskId id, TimePoint now)
{
  Task * task = find_task(id);
  if (task == nullptr ||
    (task->state != TaskState::Queued && task->state != TaskState::Assigned))
  {
    return false;
  }
  Vehicle * vehicle =
    task->vehicle >= 0 ? vehicles_[static_cast<std::size_t>(task->vehicle)].get() : nullptr;
  finish_task(*task, TaskState::Cancelled, "", now);
  if (vehicle != nullptr && vehicle->has_state && !clean_idle(vehicle->state)) {
    resynchronize(*vehicle, "task cancelled", now);
    update_availability(*vehicle, now);
  }
  return true;
}

bool FleetManager::reset_vehicle(const std::string & serial_number, TimePoint now)
{
  for (const std::unique_ptr<Vehicle> & vehicle : vehicles_) {
    if (vehicle->identity.serial_number == serial_number && vehicle->out_of_service) {
      vehicle->out_of_service = false;
      vehicle->navigation_failures = 0;
      update_availability(*vehicle, now);
      return true;
    }
  }
  return false;
}

void FleetManager::assign_tasks(TimePoint now)
{
  std::vector<Task *> queue;
  for (Task & task : tasks_) {
    if (task.state == TaskState::Queued) {
      queue.push_back(&task);
    }
  }
  std::stable_sort(
    queue.begin(), queue.end(), [](const Task * a, const Task * b) {
      return a->request.priority > b->request.priority;
    });
  for (Task * task : queue) {
    // The free vehicle that can be at the first stop soonest.
    const std::vector<double> times =
      travel_times_to(roadmap_, task->nodes.front(), 0.0, config_.planner);
    Vehicle * best = nullptr;
    double best_time = kForever;
    for (const std::unique_ptr<Vehicle> & vehicle : vehicles_) {
      if (!vehicle->available || vehicle->task != 0 || vehicle->node < 0) {
        continue;
      }
      if (!task->request.vehicle.empty() &&
        task->request.vehicle != vehicle->identity.serial_number)
      {
        continue;
      }
      const double time = times[static_cast<std::size_t>(base_end(*vehicle))];
      if (time < best_time) {
        best_time = time;
        best = vehicle.get();
      }
    }
    if (best == nullptr) {
      continue;
    }
    task->state = TaskState::Assigned;
    task->vehicle = best->index;
    best->task = task->id;
    plan_dirty_ = true;
    event(
      now, "task_assigned", best->identity.serial_number, "task " + std::to_string(task->id));
  }
}

// --------------------------------------------------------------- planning ----

void FleetManager::replan(TimePoint now)
{
  std::vector<AgentRequest> requests;
  std::vector<Vehicle *> planned;
  tasks_waiting_ = false;

  for (const std::unique_ptr<Vehicle> & owner : vehicles_) {
    Vehicle & vehicle = *owner;
    vehicle.plan.clear();
    vehicle.plan_stop_after.clear();
    vehicle.plan_blocked = false;
    if (vehicle.node < 0) {
      continue;  // not on the map: cannot be accounted for at all
    }
    AgentRequest request;
    request.robot = vehicle.index;
    request.start_node = base_end(vehicle);
    request.held = claimed(vehicle);
    request.immobile = !vehicle.available;
    request.priority = 1e12 + vehicle.index;  // idle vehicles are planned last
    for (const GrantedMove & move : vehicle.committed) {
      request.start_time += travel_time(roadmap_.edge(move.edge), 0.0, config_.planner);
    }

    Task * task = find_task(vehicle.task);
    if (task != nullptr && !request.immobile) {
      for (std::size_t k = task->done; k < task->dispatched; ++k) {
        request.start_time += task->dwell[k];  // still busy at a stop
      }
      // Actions for a stop go out with the order that brings the vehicle
      // there, or with a new order if it is already there. A vehicle that is
      // still finishing an order while standing at its next stop gets neither;
      // it waits one round, until it has come to rest.
      const bool at_next_stop = task->dispatched < task->nodes.size() &&
        task->nodes[task->dispatched] == request.start_node;
      if (at_next_stop && !settled(vehicle)) {
        tasks_waiting_ = true;
      } else {
        for (std::size_t k = task->dispatched; k < task->nodes.size(); ++k) {
          request.waypoints.push_back(Waypoint{task->nodes[k], task->dwell[k]});
        }
        request.priority =
          -1e6 * task->request.priority + static_cast<double>(task->sequence);
      }
    }
    requests.push_back(std::move(request));
    planned.push_back(&vehicle);
  }

  FleetPlan plan = plan_fleet(roadmap_, conflicts_, requests, config_.planner);
  ++stats_.replans;
  stats_.searches += plan.search_nodes > 0 ? 1 : 0;
  last_plan_time_ = now;
  plan_dirty_ = false;

  // The planner is checked every time. A plan that is not valid is not used:
  // everybody keeps what they have and planning is tried again later.
  const std::string problem = validate_fleet_plan(roadmap_, conflicts_, requests, plan);
  if (!problem.empty()) {
    ++stats_.invalid_plans;
    tasks_waiting_ = true;
    event(now, "invalid_plan", "", problem);
    return;
  }

  for (std::size_t i = 0; i < planned.size(); ++i) {
    Vehicle & vehicle = *planned[i];
    const AgentPlan & agent = plan.agents[i];
    vehicle.plan = agent.plan.moves;
    Task * task = find_task(vehicle.task);
    if (agent.outcome == AgentOutcome::Planned && !requests[i].waypoints.empty()) {
      vehicle.plan_stop_after = agent.plan.waypoint_after_moves;
    }
    const bool blocked = task != nullptr && !requests[i].immobile &&
      (agent.outcome == AgentOutcome::Blocked || requests[i].waypoints.empty());
    vehicle.plan_blocked = blocked;
    if (task != nullptr) {
      if (blocked && !task->blocked) {
        event(
          now, "task_blocked", vehicle.identity.serial_number,
          "task " + std::to_string(task->id) + " has no route at the moment");
      }
      task->blocked = blocked;
      task->note = blocked ? "no route at the moment" : "";
    }
    tasks_waiting_ = tasks_waiting_ || blocked;
  }
  for (const Task & task : tasks_) {
    tasks_waiting_ = tasks_waiting_ || task.state == TaskState::Queued;
  }
}

// -------------------------------------------------------------- execution ----

bool FleetManager::grantable(const Vehicle & vehicle, const PlannedMove & move) const
{
  const ResourceId wanted[2] = {
    roadmap_.edge_resource(move.edge), roadmap_.node_resource(move.to)};

  // Order: nobody who is planned to use a conflicting resource before this
  // move may still have that use ahead of them.
  for (const std::unique_ptr<Vehicle> & owner : vehicles_) {
    const Vehicle & other = *owner;
    if (&other == &vehicle) {
      continue;
    }
    for (const PlannedMove & ahead : other.plan) {
      const bool before = ahead.start < move.start - kEpsilon ||
        (std::abs(ahead.start - move.start) <= kEpsilon && other.index < vehicle.index);
      if (!before) {
        continue;
      }
      for (const ResourceId resource : wanted) {
        if (conflicts_.conflict(resource, roadmap_.edge_resource(ahead.edge)) ||
          conflicts_.conflict(resource, roadmap_.node_resource(ahead.to)))
        {
          return false;
        }
      }
    }
  }
  return true;
}

void FleetManager::grant_and_send(TimePoint now)
{
  for (const std::unique_ptr<Vehicle> & owner : vehicles_) {
    Vehicle & vehicle = *owner;
    if (!vehicle.available || vehicle.node < 0) {
      continue;
    }

    if (vehicle.pending) {
      if (now - vehicle.pending->sent_at >= config_.order_resend) {
        // QoS 0 can lose a message. Sending the same order again is harmless:
        // the vehicle ignores an order it already has.
        transport_.publish(
          vda5050_core::MqttMessage{
            vda5050_core::topic_name(vehicle.identity, "order"), vehicle.pending->payload, 0,
            false});
        vehicle.pending->sent_at = now;
        ++stats_.orders_resent;
      }
      continue;
    }

    // A vehicle that has come to rest starts its next order from scratch.
    if (vehicle.journey_open && settled(vehicle)) {
      vehicle.journey_open = false;
      vehicle.sent_horizon.clear();
    }
    if (!vehicle.journey_open) {
      vehicle.grant_sequence =
        vehicle.committed.empty() ? 0 : vehicle.committed.back().sequence_to;
    }

    Task * task = find_task(vehicle.task);
    while (!vehicle.plan.empty() && vehicle.committed.size() < config_.release_ahead) {
      // Stops at the place the vehicle starts from must go out first, with a
      // new order; they cannot be attached to a node it already has.
      if (!vehicle.plan_stop_after.empty() && vehicle.plan_stop_after.front() == 0 &&
        vehicle.journey_open)
      {
        plan_dirty_ = true;
        break;
      }
      const PlannedMove & move = vehicle.plan.front();
      if (!grantable(vehicle, move)) {
        break;
      }
      // Claims: the rule that keeps vehicles apart whatever the plan says.
      bool free = true;
      for (const std::unique_ptr<Vehicle> & other : vehicles_) {
        if (other.get() == &vehicle) {
          continue;
        }
        for (const ResourceId held : claimed(*other)) {
          free = free &&
            !conflicts_.conflict(held, roadmap_.edge_resource(move.edge)) &&
            !conflicts_.conflict(held, roadmap_.node_resource(move.to));
        }
      }
      if (!free) {
        break;
      }

      GrantedMove grant;
      grant.edge = move.edge;
      grant.from = move.from;
      grant.to = move.to;
      vehicle.grant_sequence += 2;
      grant.sequence_to = vehicle.grant_sequence;
      vehicle.plan.erase(vehicle.plan.begin());
      // Stops reached by this move are handed over with it.
      std::size_t at_start = 0;
      while (at_start < vehicle.plan_stop_after.size() &&
        vehicle.plan_stop_after[at_start] == 0)
      {
        ++at_start;
      }
      std::size_t arriving = 0;
      for (std::size_t k = at_start; k < vehicle.plan_stop_after.size(); ++k) {
        if (vehicle.plan_stop_after[k] == 1) {
          ++arriving;
        }
        --vehicle.plan_stop_after[k];
      }
      // The stops at the start stay at the front until the order goes out;
      // those arriving with this move are remembered in the grant.
      grant.stops = arriving;
      vehicle.plan_stop_after.erase(
        vehicle.plan_stop_after.begin() + static_cast<std::ptrdiff_t>(at_start),
        vehicle.plan_stop_after.begin() + static_cast<std::ptrdiff_t>(at_start + arriving));
      vehicle.committed.push_back(grant);
    }

    std::vector<int> horizon;
    for (std::size_t k = 0; k < vehicle.plan.size() && k < config_.horizon; ++k) {
      horizon.push_back(vehicle.plan[k].edge);
    }
    const bool start_stops = !vehicle.journey_open && vehicle.committed.empty() &&
      !vehicle.plan_stop_after.empty() && vehicle.plan_stop_after.front() == 0 &&
      task != nullptr && settled(vehicle);
    const bool unsent = std::any_of(
      vehicle.committed.begin(), vehicle.committed.end(),
      [](const GrantedMove & move) {return !move.sent;});
    const bool horizon_changed = vehicle.journey_open && horizon != vehicle.sent_horizon;
    if (unsent || start_stops || horizon_changed) {
      send_order(vehicle, now);
    }
  }
}

void FleetManager::send_order(Vehicle & vehicle, TimePoint now)
{
  Task * task = find_task(vehicle.task);
  PendingOrder pending;

  const auto node_entry = [this](int index, std::uint32_t sequence, bool released) {
      const MapNode & node = roadmap_.node(index);
      vda5050_core::Node entry;
      entry.node_id = node.id;
      entry.sequence_id = sequence;
      entry.released = released;
      vda5050_core::NodePosition position;
      position.x = node.x;
      position.y = node.y;
      position.map_id = roadmap_.map_id();
      position.allowed_deviation_xy = config_.attach_tolerance;
      entry.node_position = position;
      return entry;
    };
  const auto edge_entry = [this](int index, std::uint32_t sequence, bool released) {
      const MapEdge & edge = roadmap_.edge(index);
      vda5050_core::Edge entry;
      entry.edge_id = edge.id;
      entry.sequence_id = sequence;
      entry.released = released;
      entry.start_node_id = roadmap_.node(edge.from).id;
      entry.end_node_id = roadmap_.node(edge.to).id;
      if (edge.max_speed > 0.0) {
        entry.max_speed = edge.max_speed;
      }
      return entry;
    };

  vda5050_core::Order order;
  if (!vehicle.journey_open) {
    vehicle.journey_open = true;
    vehicle.order_id = "fleet-" + std::to_string(next_order_number_++);
    vehicle.order_update_id = 0;
    vehicle.stitch_node = vehicle.node;
    vehicle.stitch_sequence = 0;
    vehicle.sent_horizon.clear();
  } else {
    ++vehicle.order_update_id;
  }
  order.header = vehicle.headers.next("order");
  order.order_id = vehicle.order_id;
  order.order_update_id = vehicle.order_update_id;

  // The first node is where the vehicle's knowledge ends: its position for a
  // new order, the end of its base for an update.
  order.nodes.push_back(node_entry(vehicle.stitch_node, vehicle.stitch_sequence, true));
  if (vehicle.order_update_id == 0 && vehicle.recovery_deviation > 0.0) {
    order.nodes.front().node_position->allowed_deviation_xy = vehicle.recovery_deviation;
    vehicle.recovery_deviation = 0.0;
  }
  if (vehicle.order_update_id == 0 && task != nullptr) {
    while (!vehicle.plan_stop_after.empty() && vehicle.plan_stop_after.front() == 0) {
      const std::vector<vda5050_core::Action> & actions =
        task->request.stops[task->dispatched].actions;
      order.nodes.front().actions.insert(
        order.nodes.front().actions.end(), actions.begin(), actions.end());
      ++task->dispatched;
      ++pending.start_stops;
      vehicle.plan_stop_after.erase(vehicle.plan_stop_after.begin());
    }
  }

  std::uint32_t sequence = vehicle.stitch_sequence;
  for (GrantedMove & move : vehicle.committed) {
    if (move.sent) {
      continue;
    }
    order.edges.push_back(edge_entry(move.edge, move.sequence_to - 1, true));
    vda5050_core::Node node = node_entry(move.to, move.sequence_to, true);
    for (std::size_t k = 0; k < move.stops && task != nullptr; ++k) {
      const std::vector<vda5050_core::Action> & actions =
        task->request.stops[task->dispatched].actions;
      node.actions.insert(node.actions.end(), actions.begin(), actions.end());
      ++task->dispatched;
    }
    order.nodes.push_back(std::move(node));
    sequence = move.sequence_to;
    vehicle.stitch_node = move.to;
    vehicle.stitch_sequence = move.sequence_to;
  }
  for (std::size_t k = 0; k < vehicle.plan.size() && k < config_.horizon; ++k) {
    sequence += 2;
    order.edges.push_back(edge_entry(vehicle.plan[k].edge, sequence - 1, false));
    order.nodes.push_back(node_entry(vehicle.plan[k].to, sequence, false));
    pending.horizon.push_back(vehicle.plan[k].edge);
  }

  pending.payload = vda5050_core::encode_order(order);
  pending.update_id = vehicle.order_update_id;
  pending.sent_at = now;
  transport_.publish(
    vda5050_core::MqttMessage{
      vda5050_core::topic_name(vehicle.identity, "order"), pending.payload, 0, false});
  vehicle.pending = std::move(pending);
  ++stats_.orders_sent;
}

void FleetManager::tick(TimePoint now)
{
  for (const std::unique_ptr<Vehicle> & vehicle : vehicles_) {
    if (vehicle->needs_resync && vehicle->has_state && clean_idle(vehicle->state)) {
      finish_resynchronization(*vehicle, now);
    }
    update_availability(*vehicle, now);
    if (vehicle->needs_resync && vehicle->has_state && !clean_idle(vehicle->state) &&
      vehicle->online &&
      (!vehicle->cancel_sent_at || now - *vehicle->cancel_sent_at >= config_.order_resend))
    {
      send_cancel(*vehicle, now);
    }
  }
  assign_tasks(now);
  const bool retry = tasks_waiting_ && last_plan_time_ &&
    now - *last_plan_time_ >= config_.retry_interval;
  if (plan_dirty_ || retry || !last_plan_time_) {
    replan(now);
  }
  if (!waiting_for_discovery(now)) {
    grant_and_send(now);
  }
}

// ------------------------------------------------------------------ views ----

std::vector<VehicleView> FleetManager::vehicles() const
{
  std::vector<VehicleView> result;
  for (const std::unique_ptr<Vehicle> & owner : vehicles_) {
    const Vehicle & vehicle = *owner;
    VehicleView view;
    view.serial_number = vehicle.identity.serial_number;
    view.manufacturer = vehicle.identity.manufacturer;
    view.online = vehicle.online;
    view.available = vehicle.available;
    view.unavailable_reason = vehicle.unavailable_reason;
    view.node = vehicle.node >= 0 ? roadmap_.node(vehicle.node).id : "";
    for (const ResourceId resource : claimed(vehicle)) {
      view.claimed.push_back(roadmap_.resource_name(resource));
    }
    for (const GrantedMove & move : vehicle.committed) {
      view.route.push_back(roadmap_.node(move.to).id);
    }
    view.granted_moves = vehicle.committed.size();
    for (const PlannedMove & move : vehicle.plan) {
      view.route.push_back(roadmap_.node(move.to).id);
    }
    if (vehicle.task != 0) {
      view.task = vehicle.task;
    }
    view.blocked = vehicle.plan_blocked;
    view.order_id = vehicle.journey_open ? vehicle.order_id : "";
    result.push_back(std::move(view));
  }
  return result;
}

std::vector<std::vector<ResourceId>> FleetManager::claims() const
{
  std::vector<std::vector<ResourceId>> result;
  for (const std::unique_ptr<Vehicle> & vehicle : vehicles_) {
    result.push_back(claimed(*vehicle));
  }
  return result;
}

std::vector<TaskView> FleetManager::tasks() const
{
  std::vector<TaskView> result;
  for (const Task & item : tasks_) {
    result.push_back(*task(item.id));
  }
  return result;
}

std::optional<TaskView> FleetManager::task(TaskId id) const
{
  const Task * found = find_task(id);
  if (found == nullptr) {
    return std::nullopt;
  }
  TaskView view;
  view.id = found->id;
  view.state = found->state;
  view.vehicle = found->vehicle >= 0 ?
    vehicles_[static_cast<std::size_t>(found->vehicle)]->identity.serial_number : "";
  view.stops_done = found->done;
  view.stops_total = found->nodes.size();
  view.priority = found->request.priority;
  view.note = found->note;
  return view;
}

}  // namespace fleet_core
