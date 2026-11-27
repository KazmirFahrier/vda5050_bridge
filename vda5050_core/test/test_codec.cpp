// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <string>

#include "vda5050_core/codec.hpp"
#include "vda5050_core/json.hpp"

using namespace vda5050_core;  // NOLINT(build/namespaces)

namespace
{

const char kOrderJson[] = R"({
  "headerId": 7,
  "timestamp": "2026-11-26T19:50:00.000Z",
  "version": "2.0.0",
  "manufacturer": "acme",
  "serialNumber": "amr-001",
  "orderId": "order-1",
  "orderUpdateId": 0,
  "zoneSetId": "zones-a",
  "nodes": [
    {
      "nodeId": "N0", "sequenceId": 0, "released": true,
      "nodePosition": {"x": 1.5, "y": -2.0, "theta": 0.5, "mapId": "floor1",
                       "allowedDeviationXY": 0.3},
      "actions": []
    },
    {
      "nodeId": "N1", "sequenceId": 2, "released": true, "nodeDescription": "dock",
      "nodePosition": {"x": 4.0, "y": -2.0, "mapId": "floor1"},
      "actions": [
        {
          "actionType": "pick", "actionId": "a1", "blockingType": "HARD",
          "actionDescription": "pick pallet",
          "actionParameters": [
            {"key": "stationType", "value": "floor"},
            {"key": "height", "value": 0.2},
            {"key": "tags", "value": ["a", "b"]}
          ]
        }
      ]
    }
  ],
  "edges": [
    {
      "edgeId": "E0", "sequenceId": 1, "released": true,
      "startNodeId": "N0", "endNodeId": "N1", "maxSpeed": 1.2,
      "actions": []
    }
  ]
})";

/// Returns the DecodeError message for `payload`, or "" if it decodes.
std::string order_error(const std::string & payload)
{
  try {
    decode_order(payload);
  } catch (const DecodeError & error) {
    return error.what();
  }
  return "";
}

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

}  // namespace

TEST(Codec, DecodesCompleteOrder)
{
  const Order order = decode_order(kOrderJson);

  EXPECT_EQ(order.header.header_id, 7u);
  EXPECT_EQ(order.header.manufacturer, "acme");
  EXPECT_EQ(order.header.serial_number, "amr-001");
  EXPECT_EQ(order.order_id, "order-1");
  EXPECT_EQ(order.order_update_id, 0u);
  ASSERT_TRUE(order.zone_set_id.has_value());
  EXPECT_EQ(*order.zone_set_id, "zones-a");

  ASSERT_EQ(order.nodes.size(), 2u);
  const Node & first = order.nodes[0];
  EXPECT_EQ(first.node_id, "N0");
  EXPECT_TRUE(first.released);
  ASSERT_TRUE(first.node_position.has_value());
  EXPECT_DOUBLE_EQ(first.node_position->x, 1.5);
  EXPECT_DOUBLE_EQ(first.node_position->y, -2.0);
  EXPECT_EQ(first.node_position->map_id, "floor1");
  ASSERT_TRUE(first.node_position->theta.has_value());
  EXPECT_DOUBLE_EQ(*first.node_position->allowed_deviation_xy, 0.3);
  EXPECT_FALSE(first.node_description.has_value());

  const Node & second = order.nodes[1];
  EXPECT_EQ(second.sequence_id, 2u);
  EXPECT_EQ(*second.node_description, "dock");
  EXPECT_FALSE(second.node_position->theta.has_value());
  ASSERT_EQ(second.actions.size(), 1u);
  const Action & action = second.actions[0];
  EXPECT_EQ(action.action_type, "pick");
  EXPECT_EQ(action.action_id, "a1");
  EXPECT_TRUE(action.blocking_type == BlockingType::Hard);
  ASSERT_EQ(action.action_parameters.size(), 3u);
  EXPECT_EQ(action.action_parameters[0].key, "stationType");
  EXPECT_EQ(action.action_parameters[0].value.as_string(), "floor");
  EXPECT_DOUBLE_EQ(action.action_parameters[1].value.as_number(), 0.2);
  EXPECT_TRUE(action.action_parameters[2].value.is_array());

  ASSERT_EQ(order.edges.size(), 1u);
  EXPECT_EQ(order.edges[0].edge_id, "E0");
  EXPECT_EQ(order.edges[0].start_node_id, "N0");
  EXPECT_EQ(order.edges[0].end_node_id, "N1");
  EXPECT_DOUBLE_EQ(*order.edges[0].max_speed, 1.2);
}

TEST(Codec, AcceptsBothSpellingsOfAllowedDeviation)
{
  const std::string prefix =
    R"({"headerId":0,"timestamp":"t","version":"2.0.0","manufacturer":"m","serialNumber":"s",)"
    R"("orderId":"o","orderUpdateId":0,"edges":[],"nodes":[{"nodeId":"N0","sequenceId":0,)"
    R"("released":true,"actions":[],"nodePosition":{"x":0,"y":0,"mapId":"m",)";
  const Order spec = decode_order(prefix + R"("allowedDeviationXY":0.7}}]})");
  const Order schema = decode_order(prefix + R"("allowedDeviationXy":0.9}}]})");
  EXPECT_DOUBLE_EQ(*spec.nodes[0].node_position->allowed_deviation_xy, 0.7);
  EXPECT_DOUBLE_EQ(*schema.nodes[0].node_position->allowed_deviation_xy, 0.9);
}

TEST(Codec, OrderErrorsNameTheOffendingField)
{
  EXPECT_TRUE(contains(order_error("not json"), "order: invalid JSON"));
  EXPECT_TRUE(contains(order_error("[]"), "order: expected object"));
  EXPECT_TRUE(contains(order_error("{}"), "order.headerId: missing required field"));

  // Break one field at a time in an otherwise valid order.
  std::string payload = kOrderJson;
  payload.replace(payload.find("\"sequenceId\": 2"), 15, "\"sequenceId\": 2.5");
  EXPECT_TRUE(contains(order_error(payload), "order.nodes[1].sequenceId"));

  payload = kOrderJson;
  payload.replace(payload.find("\"HARD\""), 6, "\"MEDIUM\"");
  EXPECT_TRUE(contains(order_error(payload), "order.nodes[1].actions[0].blockingType"));

  payload = kOrderJson;
  payload.replace(payload.find("\"endNodeId\": \"N1\""), 17, "\"endNodeId\": 5");
  EXPECT_TRUE(contains(order_error(payload), "order.edges[0].endNodeId: expected string"));

  payload = kOrderJson;
  payload.replace(payload.find("\"orderUpdateId\": 0"), 18, "\"orderUpdateId\": -1");
  EXPECT_TRUE(contains(order_error(payload), "order.orderUpdateId"));
}

TEST(Codec, DecodesInstantActions)
{
  const InstantActions message = decode_instant_actions(
    R"({"headerId":3,"timestamp":"t","version":"2.0.0","manufacturer":"m","serialNumber":"s",)"
    R"("actions":[{"actionType":"cancelOrder","actionId":"c1","blockingType":"HARD"}]})");
  EXPECT_EQ(message.header.header_id, 3u);
  ASSERT_EQ(message.actions.size(), 1u);
  EXPECT_EQ(message.actions[0].action_type, "cancelOrder");
  EXPECT_TRUE(message.actions[0].action_parameters.empty());

  EXPECT_THROW(decode_instant_actions(R"({"headerId":3})"), DecodeError);
}

TEST(Codec, EncodesStateWithAllRequiredFields)
{
  State state;
  state.header = Header{12, "2026-11-26T19:50:00.000Z", "2.0.0", "acme", "amr-001"};
  state.order_id = "order-1";
  state.order_update_id = 2;
  state.last_node_id = "N0";
  state.last_node_sequence_id = 0;
  state.driving = true;
  state.node_states.push_back(NodeState{"N1", 2, true, std::nullopt});
  state.edge_states.push_back(EdgeState{"E0", 1, true});
  ActionState action;
  action.action_id = "a1";
  action.action_type = "pick";
  action.action_status = ActionStatus::Running;
  state.action_states.push_back(action);
  AgvPosition position;
  position.x = 1.0;
  position.y = 2.0;
  position.theta = 0.25;
  position.map_id = "floor1";
  position.position_initialized = true;
  state.agv_position = position;
  state.battery_state.battery_charge = 81.5;
  state.errors.push_back(
    Error{"orderError", {{"orderId", "order-9"}}, "busy", ErrorLevel::Warning});
  state.safety_state.e_stop = EStop::None;

  // Decode our own output with the generic parser and inspect it as master control would.
  const Json json = Json::parse(encode_state(state));

  // Every field the v2.0 state schema lists as required must be present.
  for (const char * key : {"headerId", "timestamp", "version", "manufacturer", "serialNumber",
      "orderId", "orderUpdateId", "lastNodeId", "lastNodeSequenceId", "nodeStates",
      "edgeStates", "driving", "actionStates", "batteryState", "operatingMode", "errors",
      "safetyState"})
  {
    EXPECT_NE(json.find(key), nullptr) << "missing required field " << key;
  }

  EXPECT_DOUBLE_EQ(json.find("headerId")->as_number(), 12.0);
  EXPECT_EQ(json.find("orderId")->as_string(), "order-1");
  EXPECT_TRUE(json.find("driving")->as_bool());
  EXPECT_EQ(json.find("operatingMode")->as_string(), "AUTOMATIC");
  EXPECT_EQ(json.find("nodeStates")->as_array()[0].find("nodeId")->as_string(), "N1");
  EXPECT_EQ(json.find("edgeStates")->as_array()[0].find("edgeId")->as_string(), "E0");
  EXPECT_EQ(
    json.find("actionStates")->as_array()[0].find("actionStatus")->as_string(), "RUNNING");
  EXPECT_TRUE(json.find("agvPosition")->find("positionInitialized")->as_bool());
  EXPECT_DOUBLE_EQ(json.find("batteryState")->find("batteryCharge")->as_number(), 81.5);
  EXPECT_FALSE(json.find("batteryState")->find("charging")->as_bool());
  const Json & error = json.find("errors")->as_array()[0];
  EXPECT_EQ(error.find("errorType")->as_string(), "orderError");
  EXPECT_EQ(error.find("errorLevel")->as_string(), "WARNING");
  EXPECT_EQ(
    error.find("errorReferences")->as_array()[0].find("referenceKey")->as_string(), "orderId");
  EXPECT_EQ(json.find("safetyState")->find("eStop")->as_string(), "NONE");
  EXPECT_FALSE(json.find("safetyState")->find("fieldViolation")->as_bool());

  // Optional fields that were not set must be absent, not null.
  EXPECT_EQ(json.find("zoneSetId"), nullptr);
  EXPECT_EQ(json.find("velocity"), nullptr);
}

TEST(Codec, EncodesEmptyStateWithEmptyArrays)
{
  const Json json = Json::parse(encode_state(State{}));
  EXPECT_TRUE(json.find("nodeStates")->as_array().empty());
  EXPECT_TRUE(json.find("edgeStates")->as_array().empty());
  EXPECT_TRUE(json.find("actionStates")->as_array().empty());
  EXPECT_TRUE(json.find("errors")->as_array().empty());
  EXPECT_EQ(json.find("orderId")->as_string(), "");
  EXPECT_EQ(json.find("agvPosition"), nullptr);
}

TEST(Codec, EncodesConnection)
{
  Connection connection;
  connection.header = Header{0, "t", "2.0.0", "acme", "amr-001"};
  connection.connection_state = ConnectionState::ConnectionBroken;
  const Json json = Json::parse(encode_connection(connection));
  EXPECT_EQ(json.find("connectionState")->as_string(), "CONNECTIONBROKEN");
  EXPECT_EQ(json.find("serialNumber")->as_string(), "amr-001");
}

TEST(Codec, AcceptsActionNameInPlaceOfActionType)
{
  // What the official 2.0.0 instantActions schema describes.
  const InstantActions message = decode_instant_actions(
    R"({"headerId":3,"timestamp":"t","version":"2.0.0","manufacturer":"m","serialNumber":"s",)"
    R"("actions":[{"actionName":"cancelOrder","actionId":"c1","blockingType":"HARD"}]})");
  ASSERT_EQ(message.actions.size(), 1u);
  EXPECT_EQ(message.actions[0].action_type, "cancelOrder");

  // Neither field present is still an error, and it names actionType.
  try {
    decode_instant_actions(
      R"({"headerId":3,"timestamp":"t","version":"2.0.0","manufacturer":"m","serialNumber":"s",)"
      R"("actions":[{"actionId":"c1","blockingType":"HARD"}]})");
    EXPECT_TRUE(false) << "expected DecodeError";
  } catch (const DecodeError & error) {
    EXPECT_TRUE(contains(error.what(), "actions[0].actionType: missing required field"));
  }
}

TEST(Codec, AcceptsLegacyInstantActionsKey)
{
  const InstantActions message = decode_instant_actions(
    R"({"headerId":3,"timestamp":"t","version":"2.0.0","manufacturer":"m","serialNumber":"s",)"
    R"("instantActions":[{"actionType":"stateRequest","actionId":"s1","blockingType":"NONE"}]})");
  ASSERT_EQ(message.actions.size(), 1u);
  EXPECT_EQ(message.actions[0].action_id, "s1");
}

TEST(Codec, EncodesVisualization)
{
  Visualization visualization;
  visualization.header = Header{4, "t", "2.0.0", "acme", "amr-001"};
  AgvPosition position;
  position.x = 1.25;
  position.map_id = "floor1";
  position.position_initialized = true;
  visualization.agv_position = position;
  visualization.velocity = Velocity{0.5, 0.0, -0.1};

  const Json json = Json::parse(encode_visualization(visualization));
  EXPECT_DOUBLE_EQ(json.find("headerId")->as_number(), 4.0);
  EXPECT_DOUBLE_EQ(json.find("agvPosition")->find("x")->as_number(), 1.25);
  EXPECT_EQ(json.find("agvPosition")->find("mapId")->as_string(), "floor1");
  EXPECT_DOUBLE_EQ(json.find("velocity")->find("omega")->as_number(), -0.1);
}

TEST(Codec, EncodesFactsheetWithSchemaRequiredFields)
{
  Factsheet factsheet;
  factsheet.header = Header{0, "t", "2.0.0", "acme", "amr-001"};
  factsheet.series_name = "carrier-100";
  factsheet.max_load_mass = 100.0;
  factsheet.speed_max = 1.5;
  factsheet.width = 0.6;
  factsheet.length = 0.9;
  factsheet.min_state_interval = 0.05;
  factsheet.default_state_interval = 30.0;
  factsheet.agv_actions.push_back(
    FactsheetAction{"pick", std::string("lift a load"), {"NODE", "INSTANT"}});

  const Json json = Json::parse(encode_factsheet(factsheet));

  const Json * type = json.find("typeSpecification");
  ASSERT_NE(type, nullptr);
  for (const char * key : {"seriesName", "agvKinematic", "agvClass", "maxLoadMass",
      "localizationTypes", "navigationTypes"})
  {
    EXPECT_NE(type->find(key), nullptr) << "typeSpecification." << key;
  }
  EXPECT_EQ(type->find("agvKinematic")->as_string(), "DIFF");
  EXPECT_EQ(type->find("navigationTypes")->as_array()[0].as_string(), "AUTONOMOUS");

  const Json * physical = json.find("physicalParameters");
  ASSERT_NE(physical, nullptr);
  for (const char * key : {"speedMin", "speedMax", "accelerationMax", "decelerationMax",
      "heightMax", "width", "length"})
  {
    EXPECT_NE(physical->find(key), nullptr) << "physicalParameters." << key;
  }

  const Json * limits = json.find("protocolLimits");
  ASSERT_NE(limits, nullptr);
  EXPECT_TRUE(limits->find("maxStringLens")->is_object());
  EXPECT_TRUE(limits->find("maxArrayLens")->is_object());
  EXPECT_DOUBLE_EQ(limits->find("timing")->find("minStateInterval")->as_number(), 0.05);
  EXPECT_DOUBLE_EQ(limits->find("timing")->find("defaultStateInterval")->as_number(), 30.0);
  EXPECT_EQ(limits->find("timing")->find("visualizationInterval"), nullptr);

  // Required by the schema; empty because geometry and loads are not modelled.
  ASSERT_NE(json.find("agvGeometry"), nullptr);
  EXPECT_TRUE(json.find("agvGeometry")->is_object());
  ASSERT_NE(json.find("loadSpecification"), nullptr);
  EXPECT_TRUE(json.find("loadSpecification")->is_object());

  const Json * features = json.find("protocolFeatures");
  ASSERT_NE(features, nullptr);
  EXPECT_TRUE(features->find("optionalParameters")->as_array().empty());
  const Json & action = features->find("agvActions")->as_array()[0];
  EXPECT_EQ(action.find("actionType")->as_string(), "pick");
  EXPECT_EQ(action.find("actionScopes")->as_array().size(), 2u);
  EXPECT_EQ(action.find("actionDescription")->as_string(), "lift a load");
}
