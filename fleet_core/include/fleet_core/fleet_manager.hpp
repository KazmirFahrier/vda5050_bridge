// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// FleetManager: the master control side of VDA 5050 for a fleet on one roadmap.
//
// It takes tasks ("go to these nodes, do these actions"), assigns them to
// vehicles, plans collision-free routes for all of them together, and hands
// each vehicle its route piece by piece, so that no two vehicles are ever
// given places that are too close to each other.
//
// How safety and progress are obtained (details: docs/fleet_management.md):
//
//   Claims.   A vehicle may only be on resources (nodes, edges) it has been
//             given: the released part ("base") of its order. A resource is
//             given only if it conflicts with nothing another vehicle holds.
//             This rule alone prevents collisions, whatever the plan says.
//
//   Order.    The joint plan fixes, for every pair of conflicting uses, who
//             goes first. A vehicle is given its next resource only when
//             every use planned before it has been released. Because the plan
//             is valid in time, this order has no cycles, so some vehicle can
//             always move: no deadlock, however late anybody is.
//
// The class does no I/O and reads no clock. Messages come in through
// on_message(), go out through the MqttTransport, and time is an argument.
// One thread only.

#ifndef FLEET_CORE__FLEET_MANAGER_HPP_
#define FLEET_CORE__FLEET_MANAGER_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "fleet_core/conflicts.hpp"
#include "fleet_core/planner.hpp"
#include "fleet_core/roadmap.hpp"
#include "vda5050_core/header.hpp"
#include "vda5050_core/session.hpp"
#include "vda5050_core/types.hpp"

namespace fleet_core
{

using TimePoint = std::chrono::steady_clock::time_point;
using TaskId = std::uint64_t;

struct FleetConfig
{
  std::string interface_name{"uagv"};
  std::string version{"2.0.0"};

  /// Vehicles are treated as discs of this radius (metres), fleet-wide.
  double vehicle_radius{0.4};
  /// Extra distance kept between two vehicles, on top of two radii. It has to
  /// cover how far a vehicle may be from where the books say it is, so the
  /// distance actually kept is 2 * radius + max(safety_margin,
  /// 2 * attach_tolerance): each of two vehicles may be off by the tolerance.
  double safety_margin{0.2};

  PlannerConfig planner;

  /// A vehicle counts as being at a node if it is within this distance of it.
  /// Keep it as small as the vehicles' stopping accuracy allows: it widens
  /// the distance kept between vehicles (see safety_margin).
  double attach_tolerance{0.25};
  /// How many edges ahead a vehicle is given at most (the length of its base).
  std::size_t release_ahead{3};
  /// How many further planned edges are shown to the vehicle as horizon.
  std::size_t horizon{3};

  /// Without a state message for this long a vehicle counts as lost.
  /// Must be longer than the vehicles' own state interval (30 s by default).
  std::chrono::milliseconds state_timeout{60000};
  /// A vehicle that has announced itself but whose place is not known yet is
  /// a vehicle the others could run into. For this long after it came online,
  /// nobody is given new ground; after that the fleet carries on without it
  /// and says so in an event.
  std::chrono::milliseconds discovery_grace{10000};
  /// An order that the vehicle has not confirmed is sent again after this long.
  std::chrono::milliseconds order_resend{2000};
  /// While some task could not be planned, planning is tried again this often.
  std::chrono::milliseconds retry_interval{2000};
  /// After this many navigation failures in a row a vehicle is taken out of
  /// service until reset_vehicle() is called.
  int max_navigation_failures{3};
  /// Seconds assumed for the actions at a stop that gives no estimate.
  double default_dwell{5.0};
};

/// One stop of a task.
struct TaskStop
{
  std::string node;
  /// Executed at the node. actionId is assigned by the fleet manager.
  std::vector<vda5050_core::Action> actions;
  /// Expected duration of the actions in seconds; negative = use the default
  /// (0 if there are no actions).
  double dwell{-1.0};
};

struct TaskRequest
{
  std::vector<TaskStop> stops;
  /// Serial number of the vehicle that must do it; empty = any.
  std::string vehicle;
  /// Higher is more urgent.
  int priority{0};
};

enum class TaskState
{
  Queued,     ///< Waiting for a vehicle.
  Assigned,   ///< A vehicle has it; it may be driving, working or waiting.
  Finished,
  Failed,
  Cancelled,
};

struct TaskView
{
  TaskId id{0};
  TaskState state{TaskState::Queued};
  std::string vehicle;        ///< Serial number, empty while queued.
  std::size_t stops_done{0};
  std::size_t stops_total{0};
  int priority{0};
  std::string note;           ///< Why it failed, or why it is not progressing.
};

struct VehicleView
{
  std::string serial_number;
  std::string manufacturer;
  bool online{false};
  /// Whether the fleet manager may give it orders right now.
  bool available{false};
  std::string unavailable_reason;
  std::string node;                  ///< Where it is, empty if not on the map.
  std::vector<std::string> claimed;  ///< Names of the resources it holds.
  std::vector<std::string> route;    ///< Node ids still ahead: granted, then planned.
  std::size_t granted_moves{0};      ///< How many edges of `route` are granted.
  std::optional<TaskId> task;
  bool blocked{false};               ///< Its task found no route in the last plan.
  std::string order_id;
};

struct FleetEvent
{
  TimePoint time{};
  std::string kind;     ///< Short machine-readable tag, e.g. "task_finished".
  std::string vehicle;  ///< Serial number, may be empty.
  std::string text;
};

struct FleetStats
{
  std::uint64_t replans{0};
  std::uint64_t searches{0};         ///< Replans in which the search stage ran.
  std::uint64_t orders_sent{0};
  std::uint64_t orders_resent{0};
  std::uint64_t tasks_finished{0};
  std::uint64_t tasks_failed{0};
  std::uint64_t invalid_plans{0};    ///< Plans rejected by the validity check. Must stay 0.
};

class FleetManager
{
public:
  FleetManager(
    Roadmap roadmap, FleetConfig config, vda5050_core::MqttTransport & transport,
    vda5050_core::HeaderFactory::Clock clock = [] {return std::chrono::system_clock::now();});
  ~FleetManager();
  FleetManager(const FleetManager &) = delete;
  FleetManager & operator=(const FleetManager &) = delete;

  /// MQTT topic filters to subscribe to: state and connection of every vehicle.
  std::vector<std::string> subscriptions() const;

  /// Route an incoming message. Unknown topics and malformed payloads are
  /// counted as events and otherwise ignored; this never throws.
  void on_message(const std::string & topic, std::string_view payload, TimePoint now);

  /// Queue a task. Returns nullopt (and records an event) if the request is
  /// unusable: no stops, or an unknown node.
  std::optional<TaskId> submit(const TaskRequest & request, TimePoint now);

  /// Cancel a task. A running task is stopped with cancelOrder; the vehicle
  /// halts at its next node. False if the task is unknown or already over.
  bool cancel(TaskId task, TimePoint now);

  /// Put a vehicle that was taken out of service back into use.
  bool reset_vehicle(const std::string & serial_number, TimePoint now);

  /// Time-driven work: timeouts, repeated sends, planning. Call regularly
  /// (10 Hz is plenty) and after every batch of messages.
  void tick(TimePoint now);

  std::vector<VehicleView> vehicles() const;
  std::vector<TaskView> tasks() const;
  std::optional<TaskView> task(TaskId id) const;
  const FleetStats & stats() const noexcept {return stats_;}
  /// Events since the last call, oldest first.
  std::vector<FleetEvent> take_events();

  /// What each vehicle holds, as resource numbers, in the order of vehicles().
  /// For monitors and tests that check the claims independently.
  std::vector<std::vector<ResourceId>> claims() const;

  const Roadmap & roadmap() const noexcept {return roadmap_;}
  const ConflictTable & conflicts() const noexcept {return conflicts_;}

private:
  struct Vehicle;
  struct Task;

  Vehicle * find_vehicle(const std::string & manufacturer, const std::string & serial);
  Vehicle & vehicle_for(const std::string & manufacturer, const std::string & serial);
  Task * find_task(TaskId id);
  const Task * find_task(TaskId id) const;

  void on_state(Vehicle & vehicle, const vda5050_core::State & state, TimePoint now);
  void on_connection(Vehicle & vehicle, vda5050_core::ConnectionState state, TimePoint now);
  void locate(Vehicle & vehicle, TimePoint now);
  void place(Vehicle & vehicle, int node, TimePoint now);
  void track_progress(Vehicle & vehicle, TimePoint now);
  void track_task(Vehicle & vehicle, TimePoint now);
  void check_order_feedback(Vehicle & vehicle, TimePoint now);
  void resynchronize(Vehicle & vehicle, const std::string & why, TimePoint now);
  void finish_resynchronization(Vehicle & vehicle, TimePoint now);
  void update_availability(Vehicle & vehicle, TimePoint now);

  void assign_tasks(TimePoint now);
  void replan(TimePoint now);
  void grant_and_send(TimePoint now);
  bool grantable(const Vehicle & vehicle, const PlannedMove & move) const;
  void send_order(Vehicle & vehicle, TimePoint now);
  void send_cancel(Vehicle & vehicle, TimePoint now);
  void send_state_request(Vehicle & vehicle, TimePoint now);
  bool waiting_for_discovery(TimePoint now);
  void finish_task(Task & task, TaskState state, const std::string & note, TimePoint now);

  std::vector<ResourceId> claimed(const Vehicle & vehicle) const;
  int base_end(const Vehicle & vehicle) const;
  bool settled(const Vehicle & vehicle) const;
  void event(
    TimePoint now, const std::string & kind, const std::string & vehicle,
    const std::string & text);

  Roadmap roadmap_;
  FleetConfig config_;
  ConflictTable conflicts_;
  vda5050_core::MqttTransport & transport_;
  vda5050_core::HeaderFactory::Clock clock_;

  std::vector<std::unique_ptr<Vehicle>> vehicles_;
  std::deque<Task> tasks_;
  TaskId next_task_id_{1};
  std::uint64_t next_order_number_{1};

  bool plan_dirty_{true};
  std::optional<TimePoint> last_plan_time_;
  bool tasks_waiting_{false};  ///< Some task is queued or was blocked in the last plan.

  FleetStats stats_;
  std::vector<FleetEvent> events_;
};

}  // namespace fleet_core

#endif  // FLEET_CORE__FLEET_MANAGER_HPP_
