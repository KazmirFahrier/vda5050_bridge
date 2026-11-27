// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Plain data types mirroring the VDA 5050 v2.0 messages this library handles.
// Field names follow the specification in snake_case; the JSON spelling lives
// only in codec.cpp. Optional protocol fields are std::optional so "absent"
// and "zero" are never confused.

#ifndef VDA5050_CORE__TYPES_HPP_
#define VDA5050_CORE__TYPES_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/json.hpp"

namespace vda5050_core
{

// ---------------------------------------------------------------- common ----

/// Fields every VDA 5050 message starts with.
struct Header
{
  std::uint32_t header_id{0};   ///< Per-topic counter, incremented for each sent message.
  std::string timestamp;        ///< ISO 8601 UTC, e.g. 2026-11-26T19:50:00.123Z
  std::string version;          ///< Protocol version, e.g. "2.0.0"
  std::string manufacturer;
  std::string serial_number;
};

/// How an action interacts with driving and with other actions.
enum class BlockingType
{
  None,  ///< May run while driving and alongside other actions.
  Soft,  ///< May run alongside other actions, but the vehicle must not drive.
  Hard,  ///< Must run alone, and the vehicle must not drive.
};

struct ActionParameter
{
  std::string key;
  Json value;  ///< The spec allows array, boolean, number or string.
};

struct Action
{
  std::string action_type;
  std::string action_id;
  std::optional<std::string> action_description;
  BlockingType blocking_type{BlockingType::Hard};
  std::vector<ActionParameter> action_parameters;
};

// ----------------------------------------------------------------- order ----

struct NodePosition
{
  double x{0.0};
  double y{0.0};
  std::optional<double> theta;
  std::optional<double> allowed_deviation_xy;
  std::optional<double> allowed_deviation_theta;
  std::string map_id;
};

struct Node
{
  std::string node_id;
  std::uint32_t sequence_id{0};  ///< Even numbers; edges take the odd numbers between.
  std::optional<std::string> node_description;
  bool released{false};          ///< true = part of the base, false = horizon (preview only).
  std::optional<NodePosition> node_position;
  std::vector<Action> actions;
};

struct Edge
{
  std::string edge_id;
  std::uint32_t sequence_id{0};
  std::optional<std::string> edge_description;
  bool released{false};
  std::string start_node_id;
  std::string end_node_id;
  std::optional<double> max_speed;
  std::optional<double> orientation;
  std::vector<Action> actions;
};

struct Order
{
  Header header;
  std::string order_id;
  std::uint32_t order_update_id{0};
  std::optional<std::string> zone_set_id;
  std::vector<Node> nodes;
  std::vector<Edge> edges;
};

struct InstantActions
{
  Header header;
  std::vector<Action> actions;
};

// ----------------------------------------------------------------- state ----

enum class ActionStatus {Waiting, Initializing, Running, Paused, Finished, Failed};
enum class ErrorLevel {Warning, Fatal};
enum class OperatingMode {Automatic, Semiautomatic, Manual, Service, Teachin};
enum class EStop {Autoack, Manual, Remote, None};
enum class ConnectionState {Online, Offline, ConnectionBroken};

/// FINISHED and FAILED are terminal: an action never leaves them.
inline bool is_terminal(ActionStatus status)
{
  return status == ActionStatus::Finished || status == ActionStatus::Failed;
}

struct NodeState
{
  std::string node_id;
  std::uint32_t sequence_id{0};
  bool released{false};
  std::optional<NodePosition> node_position;
};

struct EdgeState
{
  std::string edge_id;
  std::uint32_t sequence_id{0};
  bool released{false};
};

struct ActionState
{
  std::string action_id;
  std::string action_type;
  std::optional<std::string> action_description;
  ActionStatus action_status{ActionStatus::Waiting};
  std::optional<std::string> result_description;
};

struct AgvPosition
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  std::string map_id;
  bool position_initialized{false};
  std::optional<double> localization_score;
};

struct Velocity
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

struct BatteryState
{
  double battery_charge{0.0};  ///< Percent, 0..100.
  bool charging{false};
  std::optional<double> battery_voltage;
};

struct ErrorReference
{
  std::string reference_key;
  std::string reference_value;
};

struct Error
{
  std::string error_type;
  std::vector<ErrorReference> error_references;
  std::string error_description;
  ErrorLevel error_level{ErrorLevel::Warning};
};

struct SafetyState
{
  EStop e_stop{EStop::None};
  bool field_violation{false};
};

/// The complete vehicle state published on the `state` topic.
struct State
{
  Header header;
  std::string order_id;
  std::uint32_t order_update_id{0};
  std::optional<std::string> zone_set_id;
  std::string last_node_id;
  std::uint32_t last_node_sequence_id{0};
  std::vector<NodeState> node_states;
  std::vector<EdgeState> edge_states;
  std::optional<AgvPosition> agv_position;
  std::optional<Velocity> velocity;
  bool driving{false};
  bool paused{false};
  bool new_base_request{false};
  std::vector<ActionState> action_states;
  BatteryState battery_state;
  OperatingMode operating_mode{OperatingMode::Automatic};
  std::vector<Error> errors;
  SafetyState safety_state;
};

struct Connection
{
  Header header;
  ConnectionState connection_state{ConnectionState::Offline};
};

// --------------------------------------------- visualization / factsheet ----

/// High-rate position message for display purposes. Carries no order logic.
struct Visualization
{
  Header header;
  std::optional<AgvPosition> agv_position;
  std::optional<Velocity> velocity;
};

/// One action the vehicle can execute, as advertised in the factsheet.
struct FactsheetAction
{
  std::string action_type;
  std::optional<std::string> action_description;
  std::vector<std::string> action_scopes;  ///< Any of "INSTANT", "NODE", "EDGE".
};

/// Static description of the vehicle type, published on request.
/// This covers the fields the v2.0 schema marks as required plus the action
/// list; geometry and load specification are not modelled.
struct Factsheet
{
  Header header;

  // typeSpecification
  std::string series_name;
  std::optional<std::string> series_description;
  std::string agv_kinematic{"DIFF"};      ///< DIFF, OMNI or THREEWHEEL
  std::string agv_class{"CARRIER"};       ///< FORKLIFT, CONVEYOR, TUGGER or CARRIER
  double max_load_mass{0.0};              ///< kg
  std::vector<std::string> localization_types{"NATURAL"};
  std::vector<std::string> navigation_types{"AUTONOMOUS"};

  // physicalParameters (SI units)
  double speed_min{0.0};
  double speed_max{0.0};
  double acceleration_max{0.0};
  double deceleration_max{0.0};
  double height_max{0.0};
  double width{0.0};
  double length{0.0};

  // protocolLimits.timing (seconds)
  double min_order_interval{0.0};
  double min_state_interval{0.0};
  std::optional<double> default_state_interval;
  std::optional<double> visualization_interval;

  // protocolFeatures
  std::vector<FactsheetAction> agv_actions;
};

}  // namespace vda5050_core

#endif  // VDA5050_CORE__TYPES_HPP_
