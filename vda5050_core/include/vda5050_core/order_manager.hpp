// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// OrderManager: the vehicle-side VDA 5050 order state machine.

#ifndef VDA5050_CORE__ORDER_MANAGER_HPP_
#define VDA5050_CORE__ORDER_MANAGER_HPP_

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/ports.hpp"
#include "vda5050_core/types.hpp"

namespace vda5050_core
{

/// Error type strings placed in State.errors. The first four are named by the
/// specification; the rest are this implementation's own.
namespace error_type
{
inline constexpr char kValidation[] = "validationError";
inline constexpr char kNoRoute[] = "noRouteError";
inline constexpr char kOrder[] = "orderError";
inline constexpr char kOrderUpdate[] = "orderUpdateError";
inline constexpr char kNoOrderToCancel[] = "noOrderToCancel";
inline constexpr char kNavigation[] = "navigationError";
inline constexpr char kAction[] = "actionError";
}  // namespace error_type

struct OrderManagerConfig
{
  /// How far (metres) the vehicle may be from the first node of a new order
  /// when that node does not specify allowedDeviationXY itself.
  double default_deviation_xy{0.5};

  /// Upper bound on State.errors. When exceeded the oldest entry is dropped,
  /// so a misbehaving master cannot grow the state message without limit.
  std::size_t max_errors{32};
};

/// Implements order acceptance, node-by-node traversal, action scheduling,
/// instant actions and state reporting for one vehicle.
///
/// Threading: this class is NOT thread-safe and never blocks. The owner must
/// call it from one thread at a time (in ROS 2: one mutually-exclusive callback
/// group). All outputs are either port calls made during a method, or the
/// snapshot() pulled afterwards. That makes every behaviour reproducible in a
/// unit test with no clock, no threads and no middleware.
class OrderManager
{
public:
  OrderManager(NavigationPort & navigation, ActionPort & actions, OrderManagerConfig config = {});

  // ---- messages from master control ---------------------------------------

  /// Process a decoded `order` message: validate, then accept as a new order,
  /// accept as an update, ignore (duplicate) or reject with an error.
  void handle_order(const Order & order);

  /// Process a decoded `instantActions` message.
  void handle_instant_actions(const InstantActions & instant_actions);

  /// Record that a payload on `topic` could not be decoded at all.
  void report_malformed_message(const std::string & topic, const std::string & reason);

  // ---- events from the robot ----------------------------------------------

  void on_navigation_succeeded();
  void on_navigation_failed(const std::string & reason);
  void on_action_status(
    const std::string & action_id, ActionStatus status, const std::string & result = "");

  void set_position(const AgvPosition & position);
  void set_velocity(const Velocity & velocity);
  void set_battery(const BatteryState & battery);
  void set_operating_mode(OperatingMode mode);
  void set_safety_state(const SafetyState & safety);

  // ---- outputs ------------------------------------------------------------

  /// The current vehicle state. The header is left empty for the caller to stamp.
  State snapshot() const;

  /// True once after something changed that master control should hear about
  /// immediately (the spec asks for a state message on every relevant event,
  /// in addition to the periodic one). Reading the flag clears it.
  bool consume_state_changed();

  /// True once after a factsheetRequest instant action was received.
  bool consume_factsheet_request();

  /// True while the vehicle has nodes left to visit, is driving, or still has
  /// order actions that have not finished. A new order is only accepted when false.
  bool has_active_order() const;

private:
  enum class Scope {Node, Edge, Instant};

  struct TrackedAction
  {
    Action action;
    ActionState state;
    Scope scope;
    std::uint32_t sequence_id;  ///< sequenceId of the owning node/edge (0 for instant).
  };

  // Order intake
  void accept_new_order(const Order & order);
  void accept_order_update(const Order & order);
  void reject_order(const Order & order, const char * type, const std::string & description);
  bool first_node_reachable(const Node & node) const;
  std::optional<std::string> first_unsupported_action(const Order & order) const;
  void track_released_actions(const std::vector<Node> & nodes, const std::vector<Edge> & edges);

  // Execution
  void advance();
  void advance_once();
  void start_ready_actions(Scope scope, std::uint32_t sequence_id);
  bool motion_blocked_by(Scope scope, std::uint32_t sequence_id) const;
  bool any_action_running() const;
  bool hard_action_running() const;

  // Instant actions
  void handle_instant_action(const Action & action);
  void cancel_order(TrackedAction & tracked);
  void finish_cancel();

  TrackedAction * find_action(const std::string & action_id);
  void add_error(
    const char * type, ErrorLevel level, const std::string & description,
    std::vector<ErrorReference> references = {});
  void mark_changed() {state_changed_ = true;}

  NavigationPort & navigation_;
  ActionPort & action_port_;
  OrderManagerConfig config_;

  // Order progress. nodes_/edges_ hold what is still ahead of the vehicle:
  // exactly what the spec calls nodeStates/edgeStates.
  std::string order_id_;
  std::uint32_t order_update_id_{0};
  std::optional<std::string> zone_set_id_;
  std::string last_node_id_;
  std::uint32_t last_node_sequence_id_{0};
  std::deque<Node> nodes_;
  std::deque<Edge> edges_;
  std::vector<TrackedAction> actions_;

  bool driving_{false};
  bool paused_{false};
  bool halted_{false};      ///< Navigation failed; wait for master to cancel.
  bool cancelling_{false};  ///< cancelOrder received, waiting for the vehicle to stop.
  std::string cancel_action_id_;

  // Vehicle telemetry mirrored into the state message.
  std::optional<AgvPosition> position_;
  std::optional<Velocity> velocity_;
  BatteryState battery_;
  OperatingMode operating_mode_{OperatingMode::Automatic};
  SafetyState safety_;
  std::vector<Error> errors_;

  bool state_changed_{false};
  bool factsheet_requested_{false};

  // Re-entrancy guard for advance(): a port may report a result from inside
  // the call that started the work.
  bool advancing_{false};
  bool advance_again_{false};
};

}  // namespace vda5050_core

#endif  // VDA5050_CORE__ORDER_MANAGER_HPP_
