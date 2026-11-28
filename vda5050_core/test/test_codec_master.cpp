// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// The master-control side of the codec: encode_order, encode_instant_actions,
// decode_state, decode_connection. The main tool is the round trip: whatever
// one side encodes, the other side must decode to the same value.

#include <gtest/gtest.h>

#include <string>

#include "vda5050_core/codec.hpp"
#include "vda5050_core/json.hpp"

using namespace vda5050_core;  // NOLINT(build/namespaces)

namespace
{

Header header(std::uint32_t id)
{
  return Header{id, "2026-11-28T00:00:00.000Z", "2.0.0", "acme", "amr-001"};
}

Order sample_order()
{
  Order order;
  order.header = header(7);
  order.order_id = "task-12";
  order.order_update_id = 3;
  order.zone_set_id = "zones-a";

  Node first;
  first.node_id = "N1";
  first.sequence_id = 4;
  first.released = true;
  first.node_description = "dock";
  first.node_position = NodePosition{1.5, -2.0, 0.5, 0.2, 0.1, "hall"};
  Action pick;
  pick.action_type = "pick";
  pick.action_id = "a1";
  pick.action_description = "take the pallet";
  pick.blocking_type = BlockingType::Hard;
  pick.action_parameters.push_back(ActionParameter{"stationType", Json("floor")});
  pick.action_parameters.push_back(ActionParameter{"height", Json(0.25)});
  first.actions.push_back(pick);

  Node second;
  second.node_id = "N2";
  second.sequence_id = 6;
  second.released = false;

  Edge edge;
  edge.edge_id = "E1";
  edge.sequence_id = 5;
  edge.released = false;
  edge.start_node_id = "N1";
  edge.end_node_id = "N2";
  edge.max_speed = 0.8;
  edge.orientation = 1.57;
  Action beep;
  beep.action_type = "beep";
  beep.action_id = "a2";
  beep.blocking_type = BlockingType::None;
  edge.actions.push_back(beep);

  order.nodes = {first, second};
  order.edges = {edge};
  return order;
}

State sample_state()
{
  State state;
  state.header = header(99);
  state.order_id = "task-12";
  state.order_update_id = 3;
  state.zone_set_id = "zones-a";
  state.last_node_id = "N1";
  state.last_node_sequence_id = 4;
  state.driving = true;
  state.paused = true;
  state.new_base_request = true;
  state.operating_mode = OperatingMode::Semiautomatic;
  state.node_states.push_back(
    NodeState{"N2", 6, false, NodePosition{3.0, 4.0, std::nullopt, std::nullopt, std::nullopt,
        "hall"}});
  state.edge_states.push_back(EdgeState{"E1", 5, true});
  state.agv_position = AgvPosition{1.0, 2.0, 0.3, "hall", true, 0.9};
  state.velocity = Velocity{0.4, 0.0, 0.1};
  state.action_states.push_back(
    ActionState{"a1", "pick", std::string("take it"), ActionStatus::Running,
      std::string("lifting")});
  state.action_states.push_back(
    ActionState{"a2", "beep", std::nullopt, ActionStatus::Failed, std::nullopt});
  state.battery_state = BatteryState{71.5, true, 48.2};
  state.errors.push_back(
    Error{"orderError", {{"orderId", "task-11"}, {"topic", "order"}}, "rejected",
      ErrorLevel::Warning});
  state.errors.push_back(Error{"LIDAR-001", {}, "no scans", ErrorLevel::Fatal});
  state.safety_state = SafetyState{EStop::Manual, true};
  return state;
}

}  // namespace

TEST(CodecMaster, OrderSurvivesTheRoundTripToTheVehicle)
{
  const Order sent = sample_order();
  const Order received = decode_order(encode_order(sent));

  EXPECT_EQ(received.header.header_id, 7u);
  EXPECT_EQ(received.header.serial_number, "amr-001");
  EXPECT_EQ(received.order_id, "task-12");
  EXPECT_EQ(received.order_update_id, 3u);
  EXPECT_EQ(*received.zone_set_id, "zones-a");
  ASSERT_EQ(received.nodes.size(), 2u);
  ASSERT_EQ(received.edges.size(), 1u);

  const Node & node = received.nodes[0];
  EXPECT_EQ(node.node_id, "N1");
  EXPECT_EQ(node.sequence_id, 4u);
  EXPECT_TRUE(node.released);
  EXPECT_EQ(*node.node_description, "dock");
  EXPECT_DOUBLE_EQ(node.node_position->x, 1.5);
  EXPECT_DOUBLE_EQ(node.node_position->y, -2.0);
  EXPECT_DOUBLE_EQ(*node.node_position->theta, 0.5);
  EXPECT_DOUBLE_EQ(*node.node_position->allowed_deviation_xy, 0.2);
  EXPECT_DOUBLE_EQ(*node.node_position->allowed_deviation_theta, 0.1);
  EXPECT_EQ(node.node_position->map_id, "hall");
  ASSERT_EQ(node.actions.size(), 1u);
  EXPECT_EQ(node.actions[0].action_type, "pick");
  EXPECT_EQ(node.actions[0].action_id, "a1");
  EXPECT_EQ(*node.actions[0].action_description, "take the pallet");
  EXPECT_TRUE(node.actions[0].blocking_type == BlockingType::Hard);
  ASSERT_EQ(node.actions[0].action_parameters.size(), 2u);
  EXPECT_EQ(node.actions[0].action_parameters[0].key, "stationType");
  EXPECT_EQ(node.actions[0].action_parameters[0].value.as_string(), "floor");
  EXPECT_DOUBLE_EQ(node.actions[0].action_parameters[1].value.as_number(), 0.25);

  EXPECT_FALSE(received.nodes[1].released);
  EXPECT_FALSE(received.nodes[1].node_position.has_value());
  EXPECT_TRUE(received.nodes[1].actions.empty());

  const Edge & edge = received.edges[0];
  EXPECT_EQ(edge.edge_id, "E1");
  EXPECT_EQ(edge.sequence_id, 5u);
  EXPECT_FALSE(edge.released);
  EXPECT_EQ(edge.start_node_id, "N1");
  EXPECT_EQ(edge.end_node_id, "N2");
  EXPECT_DOUBLE_EQ(*edge.max_speed, 0.8);
  EXPECT_DOUBLE_EQ(*edge.orientation, 1.57);
  ASSERT_EQ(edge.actions.size(), 1u);
  EXPECT_TRUE(edge.actions[0].blocking_type == BlockingType::None);
}

TEST(CodecMaster, EncodingAnOrderTwiceGivesTheSameText)
{
  // decode(encode(x)) == x is checked field by field above; this checks the
  // other composition, encode(decode(encode(x))) == encode(x).
  const std::string once = encode_order(sample_order());
  EXPECT_EQ(encode_order(decode_order(once)), once);
}

TEST(CodecMaster, InstantActionsSurviveTheRoundTrip)
{
  InstantActions sent;
  sent.header = header(2);
  Action cancel;
  cancel.action_type = "cancelOrder";
  cancel.action_id = "c-1";
  cancel.blocking_type = BlockingType::Hard;
  Action pause;
  pause.action_type = "startPause";
  pause.action_id = "p-1";
  pause.blocking_type = BlockingType::Hard;
  sent.actions = {cancel, pause};

  const InstantActions received = decode_instant_actions(encode_instant_actions(sent));
  EXPECT_EQ(received.header.header_id, 2u);
  ASSERT_EQ(received.actions.size(), 2u);
  EXPECT_EQ(received.actions[0].action_type, "cancelOrder");
  EXPECT_EQ(received.actions[0].action_id, "c-1");
  EXPECT_EQ(received.actions[1].action_type, "startPause");
}

TEST(CodecMaster, StateSurvivesTheRoundTripFromTheVehicle)
{
  const State sent = sample_state();
  const State received = decode_state(encode_state(sent));

  EXPECT_EQ(received.header.header_id, 99u);
  EXPECT_EQ(received.order_id, "task-12");
  EXPECT_EQ(received.order_update_id, 3u);
  EXPECT_EQ(*received.zone_set_id, "zones-a");
  EXPECT_EQ(received.last_node_id, "N1");
  EXPECT_EQ(received.last_node_sequence_id, 4u);
  EXPECT_TRUE(received.driving);
  EXPECT_TRUE(received.paused);
  EXPECT_TRUE(received.new_base_request);
  EXPECT_TRUE(received.operating_mode == OperatingMode::Semiautomatic);

  ASSERT_EQ(received.node_states.size(), 1u);
  EXPECT_EQ(received.node_states[0].node_id, "N2");
  EXPECT_EQ(received.node_states[0].sequence_id, 6u);
  EXPECT_FALSE(received.node_states[0].released);
  EXPECT_DOUBLE_EQ(received.node_states[0].node_position->x, 3.0);
  ASSERT_EQ(received.edge_states.size(), 1u);
  EXPECT_EQ(received.edge_states[0].edge_id, "E1");
  EXPECT_TRUE(received.edge_states[0].released);

  EXPECT_DOUBLE_EQ(received.agv_position->x, 1.0);
  EXPECT_DOUBLE_EQ(received.agv_position->theta, 0.3);
  EXPECT_TRUE(received.agv_position->position_initialized);
  EXPECT_DOUBLE_EQ(*received.agv_position->localization_score, 0.9);
  EXPECT_DOUBLE_EQ(received.velocity->vx, 0.4);
  EXPECT_DOUBLE_EQ(received.velocity->omega, 0.1);

  ASSERT_EQ(received.action_states.size(), 2u);
  EXPECT_EQ(received.action_states[0].action_id, "a1");
  EXPECT_EQ(received.action_states[0].action_type, "pick");
  EXPECT_TRUE(received.action_states[0].action_status == ActionStatus::Running);
  EXPECT_EQ(*received.action_states[0].result_description, "lifting");
  EXPECT_TRUE(received.action_states[1].action_status == ActionStatus::Failed);

  EXPECT_DOUBLE_EQ(received.battery_state.battery_charge, 71.5);
  EXPECT_TRUE(received.battery_state.charging);
  EXPECT_DOUBLE_EQ(*received.battery_state.battery_voltage, 48.2);

  ASSERT_EQ(received.errors.size(), 2u);
  EXPECT_EQ(received.errors[0].error_type, "orderError");
  EXPECT_TRUE(received.errors[0].error_level == ErrorLevel::Warning);
  ASSERT_EQ(received.errors[0].error_references.size(), 2u);
  EXPECT_EQ(received.errors[0].error_references[0].reference_key, "orderId");
  EXPECT_EQ(received.errors[0].error_references[0].reference_value, "task-11");
  EXPECT_TRUE(received.errors[1].error_level == ErrorLevel::Fatal);

  EXPECT_TRUE(received.safety_state.e_stop == EStop::Manual);
  EXPECT_TRUE(received.safety_state.field_violation);

  EXPECT_EQ(encode_state(received), encode_state(sent));
}

TEST(CodecMaster, EveryEnumValueOfTheStateRoundTrips)
{
  for (const ActionStatus status : {ActionStatus::Waiting, ActionStatus::Initializing,
      ActionStatus::Running, ActionStatus::Paused, ActionStatus::Finished, ActionStatus::Failed})
  {
    State state = sample_state();
    state.action_states[0].action_status = status;
    EXPECT_TRUE(decode_state(encode_state(state)).action_states[0].action_status == status);
  }
  for (const OperatingMode mode : {OperatingMode::Automatic, OperatingMode::Semiautomatic,
      OperatingMode::Manual, OperatingMode::Service, OperatingMode::Teachin})
  {
    State state = sample_state();
    state.operating_mode = mode;
    EXPECT_TRUE(decode_state(encode_state(state)).operating_mode == mode);
  }
  for (const EStop e_stop : {EStop::Autoack, EStop::Manual, EStop::Remote, EStop::None}) {
    State state = sample_state();
    state.safety_state.e_stop = e_stop;
    EXPECT_TRUE(decode_state(encode_state(state)).safety_state.e_stop == e_stop);
  }
}

TEST(CodecMaster, StateFromAMinimalVehicleIsAccepted)
{
  // Only what the fleet manager needs. Battery, safety state, position and
  // velocity are absent; a vehicle that omits them is still usable.
  const State state = decode_state(
    R"({"headerId":1,"timestamp":"t","version":"2.0.0","manufacturer":"m","serialNumber":"s",)"
    R"("orderId":"","orderUpdateId":0,"lastNodeId":"","lastNodeSequenceId":0,"driving":false,)"
    R"("operatingMode":"AUTOMATIC","nodeStates":[],"edgeStates":[],"actionStates":[],)"
    R"("errors":[]})");
  EXPECT_FALSE(state.agv_position.has_value());
  EXPECT_FALSE(state.velocity.has_value());
  EXPECT_FALSE(state.paused);
  EXPECT_TRUE(state.safety_state.e_stop == EStop::None);
  EXPECT_DOUBLE_EQ(state.battery_state.battery_charge, 0.0);
}

TEST(CodecMaster, MalformedStateNamesTheField)
{
  const auto error_of = [](const std::string & from, const std::string & to) {
      std::string text = encode_state(sample_state());
      const std::size_t at = text.find(from);
      EXPECT_NE(at, std::string::npos);
      text.replace(at, from.size(), to);
      try {
        decode_state(text);
      } catch (const DecodeError & error) {
        return std::string(error.what());
      }
      return std::string();
    };
  const auto contains = [](const std::string & text, const std::string & part) {
      return text.find(part) != std::string::npos;
    };
  EXPECT_TRUE(contains(error_of("\"RUNNING\"", "\"JOGGING\""), "actionStates[0].actionStatus"));
  EXPECT_TRUE(contains(error_of("\"SEMIAUTOMATIC\"", "\"AUTO\""), "state.operatingMode"));
  EXPECT_TRUE(contains(error_of("\"FATAL\"", "\"BAD\""), "errors[1].errorLevel"));
  EXPECT_TRUE(contains(error_of("\"MANUAL\"", "\"SOMETIMES\""), "safetyState.eStop"));
  EXPECT_TRUE(contains(error_of("\"lastNodeSequenceId\":4", "\"lastNodeSequenceId\":-1"),
    "lastNodeSequenceId"));
  EXPECT_TRUE(contains(error_of("\"driving\":true", "\"driving\":\"yes\""), "driving"));
  EXPECT_TRUE(contains(error_of("\"nodeStates\":[", "\"nodeStatez\":["), "nodeStates"));
  EXPECT_TRUE(contains(error_of("\"referenceKey\":\"topic\"", "\"referenceKey\":5"),
    "errorReferences[1]"));
  EXPECT_TRUE(contains(error_of("\"errorReferences\":[", "\"errorReferences\":7,\"x\":["),
    "errorReferences"));
  EXPECT_THROW(decode_state("not json"), DecodeError);
  EXPECT_THROW(decode_state("[]"), DecodeError);
}

TEST(CodecMaster, ConnectionRoundTripsForEveryState)
{
  for (const ConnectionState value : {ConnectionState::Online, ConnectionState::Offline,
      ConnectionState::ConnectionBroken})
  {
    Connection sent;
    sent.header = header(5);
    sent.connection_state = value;
    const Connection received = decode_connection(encode_connection(sent));
    EXPECT_EQ(received.header.header_id, 5u);
    EXPECT_TRUE(received.connection_state == value);
  }
  EXPECT_THROW(
    decode_connection(
      R"({"headerId":1,"timestamp":"t","version":"2.0.0","manufacturer":"m",)"
      R"("serialNumber":"s","connectionState":"MAYBE"})"),
    DecodeError);
}
