// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_core/order_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "vda5050_core/order_validator.hpp"

namespace vda5050_core
{

namespace
{

bool is_started(ActionStatus status)
{
  return status == ActionStatus::Initializing || status == ActionStatus::Running ||
         status == ActionStatus::Paused;
}

std::string join(const std::vector<std::string> & parts)
{
  std::string out;
  for (const std::string & part : parts) {
    if (!out.empty()) {
      out += "; ";
    }
    out += part;
  }
  return out;
}

}  // namespace

OrderManager::OrderManager(
  NavigationPort & navigation, ActionPort & actions, OrderManagerConfig config)
: navigation_(navigation), action_port_(actions), config_(config) {}

// ============================================================ order intake ==

void OrderManager::handle_order(const Order & order)
{
  // Step 1: is the order well-formed on its own?
  const std::vector<std::string> problems = validate_order(order);
  if (!problems.empty()) {
    reject_order(order, error_type::kValidation, join(problems));
    return;
  }
  if (const auto unsupported = first_unsupported_action(order)) {
    reject_order(order, error_type::kOrder, "unsupported actionType '" + *unsupported + "'");
    return;
  }

  // Outside the automatic modes a person is in charge of the vehicle, so
  // master control must not be able to set it in motion.
  if (operating_mode_ != OperatingMode::Automatic &&
    operating_mode_ != OperatingMode::Semiautomatic)
  {
    reject_order(order, error_type::kOrder, "operating mode does not accept orders");
    return;
  }

  // Step 2: a different orderId means a brand-new order.
  if (order.order_id != order_id_) {
    if (has_active_order()) {
      reject_order(
        order, error_type::kOrder,
        "vehicle is still executing order '" + order_id_ + "'; cancel it first");
      return;
    }
    if (!first_node_reachable(order.nodes.front())) {
      reject_order(
        order, error_type::kNoRoute,
        "vehicle is not at the first node '" + order.nodes.front().node_id + "'");
      return;
    }
    accept_new_order(order);
    return;
  }

  // Step 3: same orderId, so this is an update, a duplicate or a stale message.
  if (order.order_update_id < order_update_id_) {
    reject_order(
      order, error_type::kOrderUpdate,
      "orderUpdateId " + std::to_string(order.order_update_id) + " is older than current " +
      std::to_string(order_update_id_));
    return;
  }
  if (order.order_update_id == order_update_id_) {
    // MQTT QoS and broker reconnects can redeliver. Processing the same order
    // twice must have no effect (idempotency), so this is silently ignored.
    return;
  }

  // The update must continue exactly where the current base ends: its first
  // node has to be the last released node the vehicle already knows about
  // (or the node it is standing on, if the base is used up).
  std::string stitch_id = last_node_id_;
  std::uint32_t stitch_sequence = last_node_sequence_id_;
  for (const Node & node : nodes_) {
    if (node.released) {
      stitch_id = node.node_id;
      stitch_sequence = node.sequence_id;
    }
  }
  const Node & first = order.nodes.front();
  if (first.node_id != stitch_id || first.sequence_id != stitch_sequence) {
    reject_order(
      order, error_type::kOrderUpdate,
      "update starts at node '" + first.node_id + "' (sequenceId " +
      std::to_string(first.sequence_id) + ") but the current base ends at '" + stitch_id +
      "' (sequenceId " + std::to_string(stitch_sequence) + ")");
    return;
  }
  accept_order_update(order);
}

void OrderManager::accept_new_order(const Order & order)
{
  order_id_ = order.order_id;
  order_update_id_ = order.order_update_id;
  zone_set_id_ = order.zone_set_id;

  // A new order starts with a clean slate: old action states and errors go.
  errors_.clear();
  actions_.clear();
  halted_ = false;

  nodes_.assign(order.nodes.begin(), order.nodes.end());
  edges_.assign(order.edges.begin(), order.edges.end());
  track_released_actions(order.nodes, order.edges);

  // The vehicle is already standing on the first node (checked by the caller),
  // so that node counts as reached immediately.
  last_node_id_ = nodes_.front().node_id;
  last_node_sequence_id_ = nodes_.front().sequence_id;
  nodes_.pop_front();

  mark_changed();
  advance();
}

void OrderManager::accept_order_update(const Order & order)
{
  // Whatever was only a preview (horizon) is replaced by the update's content.
  while (!nodes_.empty() && !nodes_.back().released) {
    nodes_.pop_back();
  }
  while (!edges_.empty() && !edges_.back().released) {
    edges_.pop_back();
  }

  // The update's first node is the stitching node the vehicle already has.
  const std::vector<Node> new_nodes(order.nodes.begin() + 1, order.nodes.end());
  track_released_actions(new_nodes, order.edges);
  nodes_.insert(nodes_.end(), new_nodes.begin(), new_nodes.end());
  edges_.insert(edges_.end(), order.edges.begin(), order.edges.end());

  order_update_id_ = order.order_update_id;
  zone_set_id_ = order.zone_set_id;

  mark_changed();
  advance();
}

void OrderManager::reject_order(
  const Order & order, const char * type, const std::string & description)
{
  // Rejection never disturbs the order that is currently running.
  add_error(
    type, ErrorLevel::Warning, description,
    {{"orderId", order.order_id}, {"orderUpdateId", std::to_string(order.order_update_id)}});
}

bool OrderManager::first_node_reachable(const Node & node) const
{
  if (node.node_position && position_ && position_->position_initialized) {
    const NodePosition & target = *node.node_position;
    if (!target.map_id.empty() && !position_->map_id.empty() &&
      target.map_id != position_->map_id)
    {
      return false;
    }
    const double allowed =
      (target.allowed_deviation_xy && *target.allowed_deviation_xy > 0.0) ?
      *target.allowed_deviation_xy : config_.default_deviation_xy;
    return std::hypot(target.x - position_->x, target.y - position_->y) <= allowed;
  }
  // No usable coordinates: fall back to the logical position.
  return !last_node_id_.empty() && node.node_id == last_node_id_;
}

std::optional<std::string> OrderManager::first_unsupported_action(const Order & order) const
{
  const auto check = [this](const std::vector<Action> & actions) -> std::optional<std::string> {
      for (const Action & action : actions) {
        if (!action_port_.supports(action.action_type)) {
          return action.action_type;
        }
      }
      return std::nullopt;
    };
  for (const Node & node : order.nodes) {
    if (auto unsupported = check(node.actions)) {
      return unsupported;
    }
  }
  for (const Edge & edge : order.edges) {
    if (auto unsupported = check(edge.actions)) {
      return unsupported;
    }
  }
  return std::nullopt;
}

void OrderManager::track_released_actions(
  const std::vector<Node> & nodes, const std::vector<Edge> & edges)
{
  const auto track = [this](const Action & action, Scope scope, std::uint32_t sequence_id) {
      if (find_action(action.action_id) != nullptr) {
        return;  // already known (e.g. a node re-sent in an update)
      }
      ActionState state;
      state.action_id = action.action_id;
      state.action_type = action.action_type;
      state.action_description = action.action_description;
      state.action_status = ActionStatus::Waiting;
      actions_.push_back(TrackedAction{action, std::move(state), scope, sequence_id});
    };
  // Only the base is executable, so only its actions get a state entry.
  for (const Node & node : nodes) {
    if (node.released) {
      for (const Action & action : node.actions) {
        track(action, Scope::Node, node.sequence_id);
      }
    }
  }
  for (const Edge & edge : edges) {
    if (edge.released) {
      for (const Action & action : edge.actions) {
        track(action, Scope::Edge, edge.sequence_id);
      }
    }
  }
}

// =============================================================== execution ==

void OrderManager::advance()
{
  // Ports are allowed to report a result synchronously, which would re-enter
  // advance() while it is half-way through. Instead of recursing, remember
  // that another pass is needed and loop.
  if (advancing_) {
    advance_again_ = true;
    return;
  }
  advancing_ = true;
  do {
    advance_again_ = false;
    advance_once();
  } while (advance_again_);
  advancing_ = false;
}

void OrderManager::advance_once()
{
  if (order_id_.empty() || paused_ || cancelling_ || driving_ || halted_) {
    return;
  }

  // 1. Actions of the node the vehicle is standing on.
  start_ready_actions(Scope::Node, last_node_sequence_id_);
  if (motion_blocked_by(Scope::Node, last_node_sequence_id_)) {
    return;
  }

  // 2. Is there a released node to drive to? If the next node is horizon the
  //    vehicle waits here for an order update; if there is none, it is done.
  if (nodes_.empty() || !nodes_.front().released) {
    return;
  }

  // 3. Actions of the edge about to be entered. Blocking edge actions are
  //    completed at the start of the edge (documented simplification).
  const std::uint32_t edge_sequence = edges_.front().sequence_id;
  start_ready_actions(Scope::Edge, edge_sequence);
  if (motion_blocked_by(Scope::Edge, edge_sequence)) {
    return;
  }

  // 4. Drive. Copies are passed because a synchronous result would pop the
  //    deque entries while the port is still using them.
  const Node target = nodes_.front();
  const Edge via = edges_.front();
  driving_ = true;
  mark_changed();
  navigation_.navigate_to(target, via);
}

void OrderManager::start_ready_actions(Scope scope, std::uint32_t sequence_id)
{
  for (std::size_t i = 0; i < actions_.size(); ++i) {
    TrackedAction & tracked = actions_[i];
    if (tracked.scope != scope || tracked.sequence_id != sequence_id ||
      tracked.state.action_status != ActionStatus::Waiting)
    {
      continue;
    }
    // HARD must run alone; anything else must not overlap a running HARD.
    // `break` (not `continue`) keeps the order's sequence: a later action
    // never overtakes an earlier one that is still waiting.
    if (tracked.action.blocking_type == BlockingType::Hard) {
      if (any_action_running()) {
        break;
      }
    } else if (hard_action_running()) {
      break;
    }
    tracked.state.action_status = ActionStatus::Initializing;
    mark_changed();
    const Action action = tracked.action;  // copy: see advance_once() step 4
    action_port_.start(action);
  }
}

bool OrderManager::motion_blocked_by(Scope scope, std::uint32_t sequence_id) const
{
  return std::any_of(
    actions_.begin(), actions_.end(), [&](const TrackedAction & tracked) {
      if (tracked.scope != scope || tracked.sequence_id != sequence_id) {
        return false;
      }
      const ActionStatus status = tracked.state.action_status;
      if (status == ActionStatus::Waiting) {
        return true;  // not even started yet: it belongs to this location
      }
      return !is_terminal(status) && tracked.action.blocking_type != BlockingType::None;
    });
}

bool OrderManager::any_action_running() const
{
  return std::any_of(
    actions_.begin(), actions_.end(), [](const TrackedAction & tracked) {
      return is_started(tracked.state.action_status);
    });
}

bool OrderManager::hard_action_running() const
{
  return std::any_of(
    actions_.begin(), actions_.end(), [](const TrackedAction & tracked) {
      return is_started(tracked.state.action_status) &&
             tracked.action.blocking_type == BlockingType::Hard;
    });
}

void OrderManager::on_navigation_succeeded()
{
  if (!driving_) {
    return;  // stale result, e.g. delivered after a cancel already completed
  }
  driving_ = false;
  if (cancelling_) {
    finish_cancel();
    return;
  }
  // The target node is now behind the vehicle, together with the edge to it.
  last_node_id_ = nodes_.front().node_id;
  last_node_sequence_id_ = nodes_.front().sequence_id;
  nodes_.pop_front();
  edges_.pop_front();
  mark_changed();
  advance();
}

void OrderManager::on_navigation_failed(const std::string & reason)
{
  if (!driving_) {
    return;
  }
  driving_ = false;
  if (cancelling_) {
    finish_cancel();  // a cancelled goal reports "failed": that is the expected path
    return;
  }
  // Do not retry blindly: stay put, tell master control, and let it decide
  // (typically cancelOrder followed by a new order).
  halted_ = true;
  const std::string target = nodes_.empty() ? std::string() : nodes_.front().node_id;
  add_error(
    error_type::kNavigation, ErrorLevel::Fatal,
    "could not reach node '" + target + "': " + reason,
    {{"orderId", order_id_}, {"nodeId", target}});
}

void OrderManager::on_action_status(
  const std::string & action_id, ActionStatus status, const std::string & result)
{
  TrackedAction * tracked = find_action(action_id);
  if (tracked == nullptr || is_terminal(tracked->state.action_status) ||
    status == ActionStatus::Waiting)
  {
    return;  // unknown id, late report after cancel, or an illegal transition
  }
  tracked->state.action_status = status;
  if (!result.empty()) {
    tracked->state.result_description = result;
  }
  if (status == ActionStatus::Failed) {
    add_error(
      error_type::kAction, ErrorLevel::Warning,
      "action '" + tracked->action.action_type + "' failed" +
      (result.empty() ? std::string() : ": " + result),
      {{"actionId", action_id}});
  }
  mark_changed();
  advance();
}

// ========================================================= instant actions ==

void OrderManager::handle_instant_actions(const InstantActions & instant_actions)
{
  for (const Action & action : instant_actions.actions) {
    handle_instant_action(action);
  }
}

void OrderManager::handle_instant_action(const Action & action)
{
  if (find_action(action.action_id) != nullptr) {
    return;  // redelivered message: already handled
  }

  // Decide this before the action itself is tracked.
  const bool order_active = has_active_order();

  ActionState state;
  state.action_id = action.action_id;
  state.action_type = action.action_type;
  state.action_description = action.action_description;
  actions_.push_back(TrackedAction{action, std::move(state), Scope::Instant, 0});
  const std::size_t index = actions_.size() - 1;
  const auto set_status = [this, index](ActionStatus status, const std::string & result = "") {
      actions_[index].state.action_status = status;
      if (!result.empty()) {
        actions_[index].state.result_description = result;
      }
    };
  mark_changed();

  const std::string & type = action.action_type;
  if (type == "cancelOrder") {
    if (!order_active) {
      set_status(ActionStatus::Failed, "no order to cancel");
      add_error(
        error_type::kNoOrderToCancel, ErrorLevel::Warning, "cancelOrder received while idle",
        {{"actionId", action.action_id}});
    } else if (cancelling_) {
      set_status(ActionStatus::Failed, "a cancel is already in progress");
    } else {
      cancel_order(actions_[index]);
    }
  } else if (type == "startPause") {
    if (!paused_) {
      paused_ = true;
      if (driving_) {
        navigation_.pause();
      }
    }
    set_status(ActionStatus::Finished);
  } else if (type == "stopPause") {
    if (paused_) {
      paused_ = false;
      if (driving_) {
        navigation_.resume();
      }
    }
    set_status(ActionStatus::Finished);
    advance();
  } else if (type == "stateRequest") {
    set_status(ActionStatus::Finished);  // mark_changed() above triggers the publish
  } else if (type == "factsheetRequest") {
    factsheet_requested_ = true;
    set_status(ActionStatus::Finished);
  } else if (action_port_.supports(type)) {
    // Vehicle-specific instant action: runs immediately, outside the order's schedule.
    set_status(ActionStatus::Initializing);
    action_port_.start(action);
  } else {
    set_status(ActionStatus::Failed, "unsupported actionType");
    add_error(
      error_type::kAction, ErrorLevel::Warning, "unsupported instant action '" + type + "'",
      {{"actionId", action.action_id}});
  }
}

void OrderManager::cancel_order(TrackedAction & cancel_action)
{
  cancel_action_id_ = cancel_action.action.action_id;
  cancel_action.state.action_status = ActionStatus::Running;

  // Order actions: not-yet-started ones simply fail; started ones are stopped.
  for (TrackedAction & tracked : actions_) {
    if (tracked.scope == Scope::Instant || is_terminal(tracked.state.action_status)) {
      continue;
    }
    if (is_started(tracked.state.action_status)) {
      action_port_.cancel(tracked.action.action_id);
    }
    tracked.state.action_status = ActionStatus::Failed;
    tracked.state.result_description = "order cancelled";
  }

  if (driving_) {
    // The cancel is only complete once the vehicle has really stopped, which
    // the navigation port reports through on_navigation_succeeded/failed.
    cancelling_ = true;
    navigation_.cancel();
  } else {
    finish_cancel();
  }
}

void OrderManager::finish_cancel()
{
  nodes_.clear();
  edges_.clear();
  driving_ = false;
  cancelling_ = false;
  halted_ = false;
  // orderId, orderUpdateId and lastNodeId are kept on purpose: the spec wants
  // the state to keep naming the cancelled order until a new one arrives.
  if (TrackedAction * cancel_action = find_action(cancel_action_id_)) {
    cancel_action->state.action_status = ActionStatus::Finished;
  }
  cancel_action_id_.clear();
  mark_changed();
}

// ================================================================= outputs ==

void OrderManager::report_malformed_message(const std::string & topic, const std::string & reason)
{
  add_error(error_type::kValidation, ErrorLevel::Warning, reason, {{"topic", topic}});
}

void OrderManager::set_position(const AgvPosition & position) {position_ = position;}
void OrderManager::set_velocity(const Velocity & velocity) {velocity_ = velocity;}
void OrderManager::set_battery(const BatteryState & battery) {battery_ = battery;}

void OrderManager::set_operating_mode(OperatingMode mode)
{
  if (mode != operating_mode_) {
    operating_mode_ = mode;
    mark_changed();
  }
}

void OrderManager::set_safety_state(const SafetyState & safety)
{
  if (safety.e_stop != safety_.e_stop || safety.field_violation != safety_.field_violation) {
    safety_ = safety;
    mark_changed();
  }
}

State OrderManager::snapshot() const
{
  State state;
  state.order_id = order_id_;
  state.order_update_id = order_update_id_;
  state.zone_set_id = zone_set_id_;
  state.last_node_id = last_node_id_;
  state.last_node_sequence_id = last_node_sequence_id_;

  std::size_t released_ahead = 0;
  for (const Node & node : nodes_) {
    state.node_states.push_back(
      NodeState{node.node_id, node.sequence_id, node.released, node.node_position});
    released_ahead += node.released ? 1 : 0;
  }
  for (const Edge & edge : edges_) {
    state.edge_states.push_back(EdgeState{edge.edge_id, edge.sequence_id, edge.released});
  }
  for (const TrackedAction & tracked : actions_) {
    state.action_states.push_back(tracked.state);
  }

  state.agv_position = position_;
  state.velocity = velocity_;
  state.driving = driving_ && !paused_;
  state.paused = paused_;
  // Ask for more base when a horizon exists and at most one released node is left.
  state.new_base_request =
    !nodes_.empty() && !nodes_.back().released && released_ahead <= 1;
  state.battery_state = battery_;
  state.operating_mode = operating_mode_;
  state.errors = errors_;
  state.safety_state = safety_;
  return state;
}

bool OrderManager::consume_state_changed()
{
  return std::exchange(state_changed_, false);
}

bool OrderManager::consume_factsheet_request()
{
  return std::exchange(factsheet_requested_, false);
}

bool OrderManager::has_active_order() const
{
  if (driving_ || cancelling_ || !nodes_.empty()) {
    return true;
  }
  return std::any_of(
    actions_.begin(), actions_.end(), [](const TrackedAction & tracked) {
      return tracked.scope != Scope::Instant && !is_terminal(tracked.state.action_status);
    });
}

OrderManager::TrackedAction * OrderManager::find_action(const std::string & action_id)
{
  const auto it = std::find_if(
    actions_.begin(), actions_.end(), [&](const TrackedAction & tracked) {
      return tracked.action.action_id == action_id;
    });
  return it == actions_.end() ? nullptr : &*it;
}

void OrderManager::add_error(
  const char * type, ErrorLevel level, const std::string & description,
  std::vector<ErrorReference> references)
{
  errors_.push_back(Error{type, std::move(references), description, level});
  if (errors_.size() > config_.max_errors) {
    errors_.erase(errors_.begin());
  }
  mark_changed();
}

}  // namespace vda5050_core
