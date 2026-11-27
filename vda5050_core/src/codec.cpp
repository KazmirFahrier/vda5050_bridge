// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_core/codec.hpp"

#include <cmath>
#include <limits>
#include <optional>

namespace vda5050_core
{

namespace
{

// ------------------------------------------------------- decode helpers ----
// Every helper takes the JSON path of the value it reads so that an error
// message pinpoints the field ("order.edges[0].endNodeId: expected string").

[[noreturn]] void fail(const std::string & path, const std::string & reason)
{
  throw DecodeError(path + ": " + reason);
}

const Json & require_object(const Json & value, const std::string & path)
{
  if (!value.is_object()) {
    fail(path, "expected object");
  }
  return value;
}

const Json & require_member(const Json & object, const char * key, const std::string & path)
{
  const Json * member = object.find(key);
  if (member == nullptr) {
    fail(path + "." + key, "missing required field");
  }
  return *member;
}

std::string as_string(const Json & value, const std::string & path)
{
  if (!value.is_string()) {
    fail(path, "expected string");
  }
  return value.as_string();
}

bool as_bool(const Json & value, const std::string & path)
{
  if (!value.is_bool()) {
    fail(path, "expected boolean");
  }
  return value.as_bool();
}

double as_number(const Json & value, const std::string & path)
{
  if (!value.is_number()) {
    fail(path, "expected number");
  }
  return value.as_number();
}

std::uint32_t as_uint32(const Json & value, const std::string & path)
{
  const double number = as_number(value, path);
  if (number < 0.0 || number > static_cast<double>(std::numeric_limits<std::uint32_t>::max()) ||
    number != std::floor(number))
  {
    fail(path, "expected unsigned 32-bit integer");
  }
  return static_cast<std::uint32_t>(number);
}

std::string required_string(const Json & object, const char * key, const std::string & path)
{
  return as_string(require_member(object, key, path), path + "." + key);
}

std::uint32_t required_uint32(const Json & object, const char * key, const std::string & path)
{
  return as_uint32(require_member(object, key, path), path + "." + key);
}

bool required_bool(const Json & object, const char * key, const std::string & path)
{
  return as_bool(require_member(object, key, path), path + "." + key);
}

double required_number(const Json & object, const char * key, const std::string & path)
{
  return as_number(require_member(object, key, path), path + "." + key);
}

std::optional<std::string> optional_string(
  const Json & object, const char * key, const std::string & path)
{
  const Json * member = object.find(key);
  if (member == nullptr) {
    return std::nullopt;
  }
  return as_string(*member, path + "." + key);
}

std::optional<double> optional_number(
  const Json & object, const char * key, const std::string & path)
{
  const Json * member = object.find(key);
  if (member == nullptr) {
    return std::nullopt;
  }
  return as_number(*member, path + "." + key);
}

const Json::Array & required_array(const Json & object, const char * key, const std::string & path)
{
  const Json & member = require_member(object, key, path);
  if (!member.is_array()) {
    fail(path + "." + key, "expected array");
  }
  return member.as_array();
}

std::string indexed(const std::string & path, const char * key, std::size_t index)
{
  return path + "." + key + "[" + std::to_string(index) + "]";
}

Header decode_header(const Json & object, const std::string & path)
{
  Header header;
  header.header_id = required_uint32(object, "headerId", path);
  header.timestamp = required_string(object, "timestamp", path);
  header.version = required_string(object, "version", path);
  header.manufacturer = required_string(object, "manufacturer", path);
  header.serial_number = required_string(object, "serialNumber", path);
  return header;
}

BlockingType decode_blocking_type(const std::string & text, const std::string & path)
{
  if (text == "NONE") {return BlockingType::None;}
  if (text == "SOFT") {return BlockingType::Soft;}
  if (text == "HARD") {return BlockingType::Hard;}
  fail(path, "expected one of NONE, SOFT, HARD");
}

Action decode_action(const Json & value, const std::string & path)
{
  require_object(value, path);
  Action action;
  action.action_type = required_string(value, "actionType", path);
  action.action_id = required_string(value, "actionId", path);
  action.action_description = optional_string(value, "actionDescription", path);
  action.blocking_type =
    decode_blocking_type(required_string(value, "blockingType", path), path + ".blockingType");
  if (const Json * parameters = value.find("actionParameters")) {
    if (!parameters->is_array()) {
      fail(path + ".actionParameters", "expected array");
    }
    std::size_t index = 0;
    for (const Json & parameter : parameters->as_array()) {
      const std::string parameter_path = indexed(path, "actionParameters", index++);
      require_object(parameter, parameter_path);
      ActionParameter decoded;
      decoded.key = required_string(parameter, "key", parameter_path);
      decoded.value = require_member(parameter, "value", parameter_path);
      action.action_parameters.push_back(std::move(decoded));
    }
  }
  return action;
}

std::vector<Action> decode_actions(const Json & object, const char * key, const std::string & path)
{
  std::vector<Action> actions;
  std::size_t index = 0;
  for (const Json & action : required_array(object, key, path)) {
    actions.push_back(decode_action(action, indexed(path, key, index++)));
  }
  return actions;
}

NodePosition decode_node_position(const Json & value, const std::string & path)
{
  require_object(value, path);
  NodePosition position;
  position.x = required_number(value, "x", path);
  position.y = required_number(value, "y", path);
  position.map_id = required_string(value, "mapId", path);
  position.theta = optional_number(value, "theta", path);
  // Tolerant reader: the specification text spells this field
  // "allowedDeviationXY" while some published JSON schemas and client
  // libraries use "allowedDeviationXy". Accept both; emit the spec spelling.
  position.allowed_deviation_xy = optional_number(value, "allowedDeviationXY", path);
  if (!position.allowed_deviation_xy) {
    position.allowed_deviation_xy = optional_number(value, "allowedDeviationXy", path);
  }
  position.allowed_deviation_theta = optional_number(value, "allowedDeviationTheta", path);
  return position;
}

Node decode_node(const Json & value, const std::string & path)
{
  require_object(value, path);
  Node node;
  node.node_id = required_string(value, "nodeId", path);
  node.sequence_id = required_uint32(value, "sequenceId", path);
  node.released = required_bool(value, "released", path);
  node.node_description = optional_string(value, "nodeDescription", path);
  if (const Json * position = value.find("nodePosition")) {
    node.node_position = decode_node_position(*position, path + ".nodePosition");
  }
  node.actions = decode_actions(value, "actions", path);
  return node;
}

Edge decode_edge(const Json & value, const std::string & path)
{
  require_object(value, path);
  Edge edge;
  edge.edge_id = required_string(value, "edgeId", path);
  edge.sequence_id = required_uint32(value, "sequenceId", path);
  edge.released = required_bool(value, "released", path);
  edge.start_node_id = required_string(value, "startNodeId", path);
  edge.end_node_id = required_string(value, "endNodeId", path);
  edge.edge_description = optional_string(value, "edgeDescription", path);
  edge.max_speed = optional_number(value, "maxSpeed", path);
  edge.orientation = optional_number(value, "orientation", path);
  edge.actions = decode_actions(value, "actions", path);
  return edge;
}

Json parse_payload(std::string_view payload, const std::string & path)
{
  try {
    return Json::parse(payload);
  } catch (const JsonParseError & error) {
    fail(path, std::string("invalid JSON: ") + error.what());
  }
}

// ------------------------------------------------------- encode helpers ----

void encode_header(const Header & header, Json & out)
{
  out["headerId"] = header.header_id;
  out["timestamp"] = header.timestamp;
  out["version"] = header.version;
  out["manufacturer"] = header.manufacturer;
  out["serialNumber"] = header.serial_number;
}

Json encode_node_position(const NodePosition & position)
{
  Json out;
  out["x"] = position.x;
  out["y"] = position.y;
  out["mapId"] = position.map_id;
  if (position.theta) {out["theta"] = *position.theta;}
  if (position.allowed_deviation_xy) {out["allowedDeviationXY"] = *position.allowed_deviation_xy;}
  if (position.allowed_deviation_theta) {
    out["allowedDeviationTheta"] = *position.allowed_deviation_theta;
  }
  return out;
}

Json encode_agv_position(const AgvPosition & position)
{
  Json out;
  out["x"] = position.x;
  out["y"] = position.y;
  out["theta"] = position.theta;
  out["mapId"] = position.map_id;
  out["positionInitialized"] = position.position_initialized;
  if (position.localization_score) {out["localizationScore"] = *position.localization_score;}
  return out;
}

Json encode_velocity(const Velocity & velocity)
{
  Json out;
  out["vx"] = velocity.vx;
  out["vy"] = velocity.vy;
  out["omega"] = velocity.omega;
  return out;
}

Json string_array(const std::vector<std::string> & values)
{
  Json out{Json::Array{}};
  for (const std::string & value : values) {
    out.push_back(value);
  }
  return out;
}

}  // namespace

// ----------------------------------------------------------- enum names ----

const char * to_string(BlockingType value)
{
  switch (value) {
    case BlockingType::None: return "NONE";
    case BlockingType::Soft: return "SOFT";
    case BlockingType::Hard: return "HARD";
  }
  return "HARD";
}

const char * to_string(ActionStatus value)
{
  switch (value) {
    case ActionStatus::Waiting: return "WAITING";
    case ActionStatus::Initializing: return "INITIALIZING";
    case ActionStatus::Running: return "RUNNING";
    case ActionStatus::Paused: return "PAUSED";
    case ActionStatus::Finished: return "FINISHED";
    case ActionStatus::Failed: return "FAILED";
  }
  return "FAILED";
}

const char * to_string(ErrorLevel value)
{
  return value == ErrorLevel::Fatal ? "FATAL" : "WARNING";
}

const char * to_string(OperatingMode value)
{
  switch (value) {
    case OperatingMode::Automatic: return "AUTOMATIC";
    case OperatingMode::Semiautomatic: return "SEMIAUTOMATIC";
    case OperatingMode::Manual: return "MANUAL";
    case OperatingMode::Service: return "SERVICE";
    case OperatingMode::Teachin: return "TEACHIN";
  }
  return "MANUAL";
}

const char * to_string(EStop value)
{
  switch (value) {
    case EStop::Autoack: return "AUTOACK";
    case EStop::Manual: return "MANUAL";
    case EStop::Remote: return "REMOTE";
    case EStop::None: return "NONE";
  }
  return "NONE";
}

const char * to_string(ConnectionState value)
{
  switch (value) {
    case ConnectionState::Online: return "ONLINE";
    case ConnectionState::Offline: return "OFFLINE";
    case ConnectionState::ConnectionBroken: return "CONNECTIONBROKEN";
  }
  return "OFFLINE";
}

// --------------------------------------------------------------- decode ----

Order decode_order(std::string_view payload)
{
  const std::string path = "order";
  const Json root = parse_payload(payload, path);
  require_object(root, path);

  Order order;
  order.header = decode_header(root, path);
  order.order_id = required_string(root, "orderId", path);
  order.order_update_id = required_uint32(root, "orderUpdateId", path);
  order.zone_set_id = optional_string(root, "zoneSetId", path);

  std::size_t index = 0;
  for (const Json & node : required_array(root, "nodes", path)) {
    order.nodes.push_back(decode_node(node, indexed(path, "nodes", index++)));
  }
  index = 0;
  for (const Json & edge : required_array(root, "edges", path)) {
    order.edges.push_back(decode_edge(edge, indexed(path, "edges", index++)));
  }
  return order;
}

InstantActions decode_instant_actions(std::string_view payload)
{
  const std::string path = "instantActions";
  const Json root = parse_payload(payload, path);
  require_object(root, path);

  InstantActions message;
  message.header = decode_header(root, path);
  // v2 names the array "actions"; v1.1 named it "instantActions". Accept the
  // older key as well so a master that still uses it is not rejected.
  const char * key = (root.find("actions") == nullptr && root.find("instantActions") != nullptr) ?
    "instantActions" : "actions";
  message.actions = decode_actions(root, key, path);
  return message;
}

// --------------------------------------------------------------- encode ----

std::string encode_state(const State & state)
{
  Json out;
  encode_header(state.header, out);
  out["orderId"] = state.order_id;
  out["orderUpdateId"] = state.order_update_id;
  if (state.zone_set_id) {out["zoneSetId"] = *state.zone_set_id;}
  out["lastNodeId"] = state.last_node_id;
  out["lastNodeSequenceId"] = state.last_node_sequence_id;
  out["driving"] = state.driving;
  out["paused"] = state.paused;
  out["newBaseRequest"] = state.new_base_request;
  out["operatingMode"] = to_string(state.operating_mode);

  // Required arrays are always emitted, even when empty.
  Json node_states{Json::Array{}};
  for (const NodeState & node : state.node_states) {
    Json item;
    item["nodeId"] = node.node_id;
    item["sequenceId"] = node.sequence_id;
    item["released"] = node.released;
    if (node.node_position) {item["nodePosition"] = encode_node_position(*node.node_position);}
    node_states.push_back(std::move(item));
  }
  out["nodeStates"] = std::move(node_states);

  Json edge_states{Json::Array{}};
  for (const EdgeState & edge : state.edge_states) {
    Json item;
    item["edgeId"] = edge.edge_id;
    item["sequenceId"] = edge.sequence_id;
    item["released"] = edge.released;
    edge_states.push_back(std::move(item));
  }
  out["edgeStates"] = std::move(edge_states);

  if (state.agv_position) {
    out["agvPosition"] = encode_agv_position(*state.agv_position);
  }
  if (state.velocity) {
    out["velocity"] = encode_velocity(*state.velocity);
  }

  Json action_states{Json::Array{}};
  for (const ActionState & action : state.action_states) {
    Json item;
    item["actionId"] = action.action_id;
    item["actionType"] = action.action_type;
    if (action.action_description) {item["actionDescription"] = *action.action_description;}
    item["actionStatus"] = to_string(action.action_status);
    if (action.result_description) {item["resultDescription"] = *action.result_description;}
    action_states.push_back(std::move(item));
  }
  out["actionStates"] = std::move(action_states);

  Json battery;
  battery["batteryCharge"] = state.battery_state.battery_charge;
  battery["charging"] = state.battery_state.charging;
  if (state.battery_state.battery_voltage) {
    battery["batteryVoltage"] = *state.battery_state.battery_voltage;
  }
  out["batteryState"] = std::move(battery);

  Json errors{Json::Array{}};
  for (const Error & error : state.errors) {
    Json item;
    item["errorType"] = error.error_type;
    item["errorDescription"] = error.error_description;
    item["errorLevel"] = to_string(error.error_level);
    Json references{Json::Array{}};
    for (const ErrorReference & reference : error.error_references) {
      Json ref;
      ref["referenceKey"] = reference.reference_key;
      ref["referenceValue"] = reference.reference_value;
      references.push_back(std::move(ref));
    }
    item["errorReferences"] = std::move(references);
    errors.push_back(std::move(item));
  }
  out["errors"] = std::move(errors);

  Json safety;
  safety["eStop"] = to_string(state.safety_state.e_stop);
  safety["fieldViolation"] = state.safety_state.field_violation;
  out["safetyState"] = std::move(safety);

  return out.dump();
}

std::string encode_connection(const Connection & connection)
{
  Json out;
  encode_header(connection.header, out);
  out["connectionState"] = to_string(connection.connection_state);
  return out.dump();
}

std::string encode_visualization(const Visualization & visualization)
{
  Json out;
  encode_header(visualization.header, out);
  if (visualization.agv_position) {
    out["agvPosition"] = encode_agv_position(*visualization.agv_position);
  }
  if (visualization.velocity) {
    out["velocity"] = encode_velocity(*visualization.velocity);
  }
  return out.dump();
}

std::string encode_factsheet(const Factsheet & factsheet)
{
  Json out;
  encode_header(factsheet.header, out);

  Json type;
  type["seriesName"] = factsheet.series_name;
  if (factsheet.series_description) {type["seriesDescription"] = *factsheet.series_description;}
  type["agvKinematic"] = factsheet.agv_kinematic;
  type["agvClass"] = factsheet.agv_class;
  type["maxLoadMass"] = factsheet.max_load_mass;
  type["localizationTypes"] = string_array(factsheet.localization_types);
  type["navigationTypes"] = string_array(factsheet.navigation_types);
  out["typeSpecification"] = std::move(type);

  Json physical;
  physical["speedMin"] = factsheet.speed_min;
  physical["speedMax"] = factsheet.speed_max;
  physical["accelerationMax"] = factsheet.acceleration_max;
  physical["decelerationMax"] = factsheet.deceleration_max;
  physical["heightMax"] = factsheet.height_max;
  physical["width"] = factsheet.width;
  physical["length"] = factsheet.length;
  out["physicalParameters"] = std::move(physical);

  // maxStringLens and maxArrayLens are required objects whose members are all
  // optional; this implementation imposes no such limits, so they are empty.
  Json timing;
  timing["minOrderInterval"] = factsheet.min_order_interval;
  timing["minStateInterval"] = factsheet.min_state_interval;
  if (factsheet.default_state_interval) {
    timing["defaultStateInterval"] = *factsheet.default_state_interval;
  }
  if (factsheet.visualization_interval) {
    timing["visualizationInterval"] = *factsheet.visualization_interval;
  }
  Json limits;
  limits["maxStringLens"] = Json{Json::Object{}};
  limits["maxArrayLens"] = Json{Json::Object{}};
  limits["timing"] = std::move(timing);
  out["protocolLimits"] = std::move(limits);

  Json actions{Json::Array{}};
  for (const FactsheetAction & action : factsheet.agv_actions) {
    Json item;
    item["actionType"] = action.action_type;
    if (action.action_description) {item["actionDescription"] = *action.action_description;}
    item["actionScopes"] = string_array(action.action_scopes);
    actions.push_back(std::move(item));
  }
  Json features;
  features["optionalParameters"] = Json{Json::Array{}};
  features["agvActions"] = std::move(actions);
  out["protocolFeatures"] = std::move(features);

  return out.dump();
}

}  // namespace vda5050_core
