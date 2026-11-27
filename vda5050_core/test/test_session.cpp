// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Tests for the MQTT-facing behaviour: which message is published on which
// topic, with which QoS/retain flags, and when.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "fakes.hpp"
#include "vda5050_core/json.hpp"
#include "vda5050_core/session.hpp"

using namespace vda5050_core;  // NOLINT(build/namespaces)
using std::chrono::milliseconds;
using std::chrono::seconds;
using test_support::FakeActions;
using test_support::FakeNavigation;
using test_support::position_at;

namespace
{

class FakeTransport : public MqttTransport
{
public:
  void publish(const MqttMessage & message) override {sent.push_back(message);}

  /// Messages published on `<prefix>/<name>`, oldest first.
  std::vector<MqttMessage> on(const std::string & name) const
  {
    std::vector<MqttMessage> matching;
    for (const MqttMessage & message : sent) {
      if (message.topic == "uagv/v2/acme/amr-001/" + name) {
        matching.push_back(message);
      }
    }
    return matching;
  }

  std::vector<MqttMessage> sent;
};

const char kOrderTopic[] = "uagv/v2/acme/amr-001/order";
const char kInstantTopic[] = "uagv/v2/acme/amr-001/instantActions";

std::string header_json(const std::string & serial = "amr-001", const std::string & version = "2.0.0")
{
  return R"("headerId":1,"timestamp":"2026-11-26T20:00:00.000Z","version":")" + version +
         R"(","manufacturer":"acme","serialNumber":")" + serial + R"(",)";
}

/// A two-node order N0 -> N1 on map "map".
std::string order_json(const std::string & order_id, const std::string & header = header_json())
{
  return "{" + header + R"("orderId":")" + order_id + R"(","orderUpdateId":0,"nodes":[)"
         R"({"nodeId":"N0","sequenceId":0,"released":true,"actions":[],)"
         R"("nodePosition":{"x":0,"y":0,"mapId":"map"}},)"
         R"({"nodeId":"N1","sequenceId":2,"released":true,"actions":[],)"
         R"("nodePosition":{"x":1,"y":0,"mapId":"map"}}],)"
         R"("edges":[{"edgeId":"E0","sequenceId":1,"released":true,)"
         R"("startNodeId":"N0","endNodeId":"N1","actions":[]}]})";
}

std::string instant_json(const std::string & id, const std::string & type)
{
  return "{" + header_json() + R"("actions":[{"actionType":")" + type + R"(","actionId":")" + id +
         R"(","blockingType":"HARD"}]})";
}

}  // namespace

class SessionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    session.orders().set_position(position_at(0.0, 0.0));
  }

  static Identity identity()
  {
    Identity id;
    id.manufacturer = "acme";
    id.serial_number = "amr-001";
    return id;
  }

  Json last_state() const
  {
    const std::vector<MqttMessage> states = transport.on("state");
    return states.empty() ? Json() : Json::parse(states.back().payload);
  }

  Session::TimePoint t0{};  // an arbitrary fixed origin; tests add offsets
  FakeTransport transport;
  FakeNavigation navigation;
  FakeActions actions;
  Session session{identity(), transport, navigation, actions};
};

TEST_F(SessionTest, SubscribesToOrderAndInstantActionTopics)
{
  const std::vector<std::string> topics = session.subscriptions();
  ASSERT_EQ(topics.size(), 2u);
  EXPECT_EQ(topics[0], kOrderTopic);
  EXPECT_EQ(topics[1], kInstantTopic);
}

TEST_F(SessionTest, LastWillAnnouncesBrokenConnectionRetained)
{
  const MqttMessage will = session.last_will();
  EXPECT_EQ(will.topic, "uagv/v2/acme/amr-001/connection");
  EXPECT_EQ(will.qos, 1);
  EXPECT_TRUE(will.retain);
  EXPECT_EQ(Json::parse(will.payload).find("connectionState")->as_string(), "CONNECTIONBROKEN");
  EXPECT_TRUE(transport.sent.empty());  // the will is handed to the broker, not published
}

TEST_F(SessionTest, ConnectingPublishesOnlineThenFullState)
{
  session.on_connected(t0);

  ASSERT_EQ(transport.sent.size(), 2u);
  const MqttMessage & connection = transport.sent[0];
  EXPECT_EQ(connection.topic, "uagv/v2/acme/amr-001/connection");
  EXPECT_EQ(connection.qos, 1);
  EXPECT_TRUE(connection.retain);
  EXPECT_EQ(Json::parse(connection.payload).find("connectionState")->as_string(), "ONLINE");

  const MqttMessage & state = transport.sent[1];
  EXPECT_EQ(state.topic, "uagv/v2/acme/amr-001/state");
  EXPECT_EQ(state.qos, 0);
  EXPECT_FALSE(state.retain);
  EXPECT_TRUE(session.connected());
}

TEST_F(SessionTest, ShutdownPublishesOfflineOnce)
{
  session.on_connected(t0);
  session.shutdown();
  session.shutdown();  // already offline: nothing more to say

  const std::vector<MqttMessage> connection = transport.on("connection");
  ASSERT_EQ(connection.size(), 2u);
  EXPECT_EQ(Json::parse(connection[1].payload).find("connectionState")->as_string(), "OFFLINE");
  EXPECT_TRUE(connection[1].retain);
  EXPECT_FALSE(session.connected());
}

TEST_F(SessionTest, HeaderIdsIncreasePerTopic)
{
  session.on_connected(t0);
  session.on_message(kOrderTopic, order_json("o1"));
  session.tick(t0 + milliseconds(50));

  const std::vector<MqttMessage> states = transport.on("state");
  ASSERT_EQ(states.size(), 2u);
  EXPECT_DOUBLE_EQ(Json::parse(states[0].payload).find("headerId")->as_number(), 0.0);
  EXPECT_DOUBLE_EQ(Json::parse(states[1].payload).find("headerId")->as_number(), 1.0);
  EXPECT_EQ(Json::parse(states[1].payload).find("serialNumber")->as_string(), "amr-001");
}

TEST_F(SessionTest, OrderMessageStartsNavigationAndStateFollowsOnNextTick)
{
  session.on_connected(t0);
  session.on_message(kOrderTopic, order_json("o1"));

  ASSERT_EQ(navigation.goals.size(), 1u);
  EXPECT_EQ(navigation.goals[0], "N1");

  session.tick(t0 + milliseconds(50));
  const Json state = last_state();
  EXPECT_EQ(state.find("orderId")->as_string(), "o1");
  EXPECT_TRUE(state.find("driving")->as_bool());
  EXPECT_EQ(state.find("nodeStates")->as_array().size(), 1u);
}

TEST_F(SessionTest, StateIsPublishedOnChangeAndOtherwiseOnlyWhenIntervalElapses)
{
  session.on_connected(t0);
  const std::size_t after_connect = transport.on("state").size();

  session.tick(t0 + seconds(1));
  session.tick(t0 + seconds(29));
  EXPECT_EQ(transport.on("state").size(), after_connect);  // nothing changed, not yet due

  session.tick(t0 + seconds(30));
  EXPECT_EQ(transport.on("state").size(), after_connect + 1);  // periodic heartbeat

  session.orders().on_navigation_succeeded();  // stale: no change
  session.tick(t0 + seconds(31));
  EXPECT_EQ(transport.on("state").size(), after_connect + 1);

  session.orders().set_operating_mode(OperatingMode::Manual);  // a real change
  session.tick(t0 + seconds(32));
  EXPECT_EQ(transport.on("state").size(), after_connect + 2);
  EXPECT_EQ(last_state().find("operatingMode")->as_string(), "MANUAL");
}

TEST_F(SessionTest, UndecodablePayloadBecomesValidationErrorInState)
{
  session.on_connected(t0);
  EXPECT_NO_THROW(session.on_message(kOrderTopic, "{ not json"));
  EXPECT_NO_THROW(session.on_message(kInstantTopic, R"({"headerId":"x"})"));
  session.tick(t0 + milliseconds(50));

  const Json state = last_state();  // keep the document alive while reading from it
  const Json::Array & errors = state.find("errors")->as_array();
  ASSERT_EQ(errors.size(), 2u);
  EXPECT_EQ(errors[0].find("errorType")->as_string(), "validationError");
  EXPECT_EQ(
    errors[0].find("errorReferences")->as_array()[0].find("referenceValue")->as_string(), "order");
  EXPECT_EQ(
    errors[1].find("errorReferences")->as_array()[0].find("referenceValue")->as_string(),
    "instantActions");
  EXPECT_TRUE(navigation.goals.empty());
}

TEST_F(SessionTest, MessageForAnotherVehicleIsRefused)
{
  session.on_connected(t0);
  session.on_message(kOrderTopic, order_json("o1", header_json("amr-999")));
  session.tick(t0 + milliseconds(50));

  EXPECT_TRUE(navigation.goals.empty());
  const Json state = last_state();
  EXPECT_EQ(state.find("orderId")->as_string(), "");
  EXPECT_EQ(state.find("errors")->as_array()[0].find("errorType")->as_string(), "validationError");
}

TEST_F(SessionTest, OtherMajorVersionIsRefusedButMinorDifferenceIsAccepted)
{
  session.on_connected(t0);
  session.on_message(kOrderTopic, order_json("o1", header_json("amr-001", "3.0.0")));
  EXPECT_TRUE(navigation.goals.empty());

  session.on_message(kOrderTopic, order_json("o2", header_json("amr-001", "2.1.0")));
  EXPECT_EQ(navigation.goals.size(), 1u);
}

TEST_F(SessionTest, UnrelatedTopicIsIgnored)
{
  session.on_connected(t0);
  session.on_message("uagv/v2/acme/amr-001/somethingElse", "garbage");
  session.tick(t0 + milliseconds(50));
  EXPECT_TRUE(last_state().find("errors")->as_array().empty());
}

TEST_F(SessionTest, InstantActionsAreRouted)
{
  session.on_connected(t0);
  session.on_message(kOrderTopic, order_json("o1"));
  session.on_message(kInstantTopic, instant_json("p1", "startPause"));
  session.tick(t0 + milliseconds(50));

  EXPECT_EQ(navigation.pause_calls, 1);
  EXPECT_TRUE(last_state().find("paused")->as_bool());
}

TEST_F(SessionTest, NothingIsPublishedWhileDisconnectedAndChangesSurviveReconnect)
{
  session.on_connected(t0);
  session.on_disconnected();
  const std::size_t before = transport.sent.size();

  session.on_message(kOrderTopic, order_json("o1"));  // e.g. delivered just before the drop
  session.orders().on_navigation_succeeded();
  session.tick(t0 + seconds(60));
  EXPECT_EQ(transport.sent.size(), before);

  session.on_connected(t0 + seconds(61));
  const Json state = last_state();
  EXPECT_EQ(state.find("orderId")->as_string(), "o1");
  EXPECT_EQ(state.find("lastNodeId")->as_string(), "N1");  // progress made offline is reported
  EXPECT_EQ(
    Json::parse(transport.on("connection").back().payload).find("connectionState")->as_string(),
    "ONLINE");
}

TEST_F(SessionTest, VisualizationIsPublishedAtItsOwnRate)
{
  session.on_connected(t0);
  session.tick(t0);
  session.tick(t0 + milliseconds(500));
  EXPECT_EQ(transport.on("visualization").size(), 1u);
  session.tick(t0 + milliseconds(1000));
  ASSERT_EQ(transport.on("visualization").size(), 2u);

  const Json message = Json::parse(transport.on("visualization")[1].payload);
  EXPECT_DOUBLE_EQ(message.find("headerId")->as_number(), 1.0);
  EXPECT_TRUE(message.find("agvPosition")->find("positionInitialized")->as_bool());
}

TEST_F(SessionTest, VisualizationCanBeDisabledAndNeedsAPosition)
{
  SessionConfig config;
  config.visualization_interval = milliseconds(0);
  Session quiet(identity(), transport, navigation, actions, config);
  quiet.orders().set_position(position_at(0.0, 0.0));
  quiet.on_connected(t0);
  quiet.tick(t0 + seconds(5));
  EXPECT_TRUE(transport.on("visualization").empty());

  Session lost(identity(), transport, navigation, actions);  // no position known
  lost.on_connected(t0);
  lost.tick(t0 + seconds(5));
  EXPECT_TRUE(transport.on("visualization").empty());
}

TEST_F(SessionTest, FactsheetIsPublishedOnRequest)
{
  Factsheet factsheet;
  factsheet.series_name = "carrier-100";
  factsheet.speed_max = 1.5;
  factsheet.agv_actions.push_back(FactsheetAction{"pick", std::nullopt, {"NODE"}});
  session.set_factsheet(factsheet);
  session.on_connected(t0);
  session.tick(t0);
  EXPECT_TRUE(transport.on("factsheet").empty());  // only on request

  session.on_message(kInstantTopic, instant_json("f1", "factsheetRequest"));
  session.tick(t0 + milliseconds(50));
  session.tick(t0 + milliseconds(100));

  const std::vector<MqttMessage> sheets = transport.on("factsheet");
  ASSERT_EQ(sheets.size(), 1u);  // once per request
  const Json json = Json::parse(sheets[0].payload);
  EXPECT_EQ(json.find("typeSpecification")->find("seriesName")->as_string(), "carrier-100");
  EXPECT_EQ(json.find("manufacturer")->as_string(), "acme");
}
