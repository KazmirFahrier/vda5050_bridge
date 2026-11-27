// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Schema-conformance helper.
//
//   emit_samples <output-dir> [inbound.json ...]
//
// Writes one fully populated example of every message this library publishes
// (state, connection, visualization, factsheet) into <output-dir>, so that an
// external validator can check them against the official VDA 5050 JSON
// schemas. Each additional argument is an inbound message (its file name must
// contain "order" or "instantActions"); it is decoded with the library, and
// the program fails if the decoder rejects it. Together with the validator
// this proves both directions: what we send is schema-valid, and messages
// that are schema-valid are accepted.

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "vda5050_core/codec.hpp"
#include "vda5050_core/header.hpp"
#include "vda5050_core/order_validator.hpp"

using namespace vda5050_core;  // NOLINT(build/namespaces)

namespace
{

bool write_file(const std::string & path, const std::string & content)
{
  std::ofstream out(path);
  out << content << "\n";
  return static_cast<bool>(out);
}

State populated_state(HeaderFactory & headers)
{
  State state;
  state.header = headers.next("state");
  state.order_id = "order-1";
  state.order_update_id = 3;
  state.zone_set_id = "zones-a";
  state.last_node_id = "N1";
  state.last_node_sequence_id = 2;
  state.driving = true;
  state.paused = false;
  state.new_base_request = true;

  NodePosition node_position;
  node_position.x = 4.0;
  node_position.y = -2.0;
  node_position.theta = 1.57;
  node_position.allowed_deviation_xy = 0.3;
  node_position.allowed_deviation_theta = 0.2;
  node_position.map_id = "floor1";
  state.node_states.push_back(NodeState{"N2", 4, true, node_position});
  state.node_states.push_back(NodeState{"N3", 6, false, std::nullopt});
  state.edge_states.push_back(EdgeState{"E1", 3, true});
  state.edge_states.push_back(EdgeState{"E2", 5, false});

  AgvPosition position;
  position.x = 1.5;
  position.y = -2.0;
  position.theta = 0.25;
  position.map_id = "floor1";
  position.position_initialized = true;
  position.localization_score = 0.93;
  state.agv_position = position;
  state.velocity = Velocity{0.5, 0.0, 0.1};

  // PAUSED is left out here and written to its own file by main(): the
  // specification text defines it, but the official schemas' enum omits it.
  for (const ActionStatus status : {ActionStatus::Waiting, ActionStatus::Initializing,
      ActionStatus::Running, ActionStatus::Finished, ActionStatus::Failed})
  {
    ActionState action;
    action.action_id = std::string("action-") + to_string(status);
    action.action_type = "pick";
    action.action_description = "pick a pallet";
    action.action_status = status;
    action.result_description = "example";
    state.action_states.push_back(action);
  }

  state.battery_state.battery_charge = 81.5;
  state.battery_state.battery_voltage = 25.2;
  state.battery_state.charging = false;
  state.operating_mode = OperatingMode::Automatic;
  state.errors.push_back(
    Error{"orderError", {{"orderId", "order-9"}, {"orderUpdateId", "0"}}, "vehicle busy",
      ErrorLevel::Warning});
  state.errors.push_back(
    Error{"navigationError", {{"nodeId", "N2"}}, "path blocked", ErrorLevel::Fatal});
  state.safety_state.e_stop = EStop::None;
  state.safety_state.field_violation = false;
  return state;
}

Factsheet populated_factsheet(HeaderFactory & headers)
{
  Factsheet factsheet;
  factsheet.header = headers.next("factsheet");
  factsheet.series_name = "carrier-100";
  factsheet.series_description = "example carrier";
  factsheet.max_load_mass = 100.0;
  factsheet.speed_min = 0.0;
  factsheet.speed_max = 1.5;
  factsheet.acceleration_max = 1.0;
  factsheet.deceleration_max = 1.5;
  factsheet.height_max = 0.5;
  factsheet.width = 0.6;
  factsheet.length = 0.9;
  factsheet.min_order_interval = 0.05;
  factsheet.min_state_interval = 0.05;
  factsheet.default_state_interval = 30.0;
  factsheet.visualization_interval = 1.0;
  factsheet.agv_actions.push_back(
    FactsheetAction{"pick", std::string("lift a load"), {"NODE", "INSTANT"}});
  factsheet.agv_actions.push_back(FactsheetAction{"cancelOrder", std::nullopt, {"INSTANT"}});
  return factsheet;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: emit_samples <output-dir> [inbound.json ...]\n";
    return 2;
  }
  const std::string out_dir = argv[1];

  Identity identity;
  identity.manufacturer = "acme";
  identity.serial_number = "amr-001";
  HeaderFactory headers(
    identity, [] {return std::chrono::system_clock::time_point{std::chrono::seconds{1795722600}};});

  bool ok = true;
  ok &= write_file(out_dir + "/state.json", encode_state(populated_state(headers)));
  // Known deviation, kept apart so the validator can treat it separately.
  State paused = populated_state(headers);
  paused.action_states.resize(1);
  paused.action_states[0].action_status = ActionStatus::Paused;
  ok &= write_file(out_dir + "/deviation_state_paused_action.json", encode_state(paused));

  // The minimal state: nothing optional set, every list empty.
  State empty;
  empty.header = headers.next("state");
  ok &= write_file(out_dir + "/state_minimal.json", encode_state(empty));

  int index = 0;
  for (const ConnectionState connection_state : {ConnectionState::Online,
      ConnectionState::Offline, ConnectionState::ConnectionBroken})
  {
    Connection connection;
    connection.header = headers.next("connection");
    connection.connection_state = connection_state;
    ok &= write_file(
      out_dir + "/connection_" + std::to_string(index++) + ".json", encode_connection(connection));
  }

  Visualization visualization;
  visualization.header = headers.next("visualization");
  visualization.agv_position = populated_state(headers).agv_position;
  visualization.velocity = Velocity{0.5, 0.0, 0.1};
  ok &= write_file(out_dir + "/visualization.json", encode_visualization(visualization));

  ok &= write_file(out_dir + "/factsheet.json", encode_factsheet(populated_factsheet(headers)));
  if (!ok) {
    std::cerr << "could not write to " << out_dir << "\n";
    return 2;
  }

  // Inbound messages: must be accepted by the decoder (and, for orders, by
  // the protocol-level validator).
  for (int i = 2; i < argc; ++i) {
    const std::string path = argv[i];
    std::ifstream in(path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    try {
      if (path.find("instantActions") != std::string::npos) {
        const InstantActions message = decode_instant_actions(buffer.str());
        std::cout << "decoded " << path << ": " << message.actions.size() << " action(s)\n";
      } else {
        const Order order = decode_order(buffer.str());
        const auto problems = validate_order(order);
        if (!problems.empty()) {
          std::cerr << path << ": " << problems.front() << "\n";
          return 1;
        }
        std::cout << "decoded " << path << ": " << order.nodes.size() << " node(s)\n";
      }
    } catch (const DecodeError & error) {
      std::cerr << path << ": " << error.what() << "\n";
      return 1;
    }
  }
  return 0;
}
