// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// End-to-end demonstration without ROS, a broker or a robot.
//
// A scripted "master control" feeds VDA 5050 JSON into a Session. A simulated
// robot stands in for Nav2: each loop step it completes whatever drive or
// action was requested. Every MQTT message the vehicle would publish is
// printed, so you can watch the protocol conversation.
//
// Scenario: order N0 -> N1 (pick) -> N2, with N2 only on the horizon. The
// vehicle stops at N1 and asks for more base; master control then releases N2.
//
// Exit code 0 means the vehicle ended idle at N2 with the pick FINISHED.

#include <chrono>
#include <cstdio>
#include <iostream>
#include <optional>
#include <string>

#include "vda5050_core/json.hpp"
#include "vda5050_core/session.hpp"

using namespace vda5050_core;  // NOLINT(build/namespaces)

namespace
{

/// Prints a one-line summary of each published message in place of sending it.
class PrintingTransport : public MqttTransport
{
public:
  void publish(const MqttMessage & message) override
  {
    const std::string name = message.topic.substr(message.topic.rfind('/') + 1);
    const Json json = Json::parse(message.payload);
    std::cout << "  vehicle -> " << name;
    if (name == "connection") {
      std::cout << "  " << json.find("connectionState")->as_string();
    } else if (name == "state") {
      last_state = json;
      std::cout << "  order=" << json.find("orderId")->as_string()
                << "/" << json.find("orderUpdateId")->as_number()
                << " lastNode=" << json.find("lastNodeId")->as_string()
                << " driving=" << (json.find("driving")->as_bool() ? "yes" : "no")
                << " nodesAhead=" << json.find("nodeStates")->as_array().size()
                << " newBaseRequest=" << (json.find("newBaseRequest")->as_bool() ? "yes" : "no");
      for (const Json & action : json.find("actionStates")->as_array()) {
        std::cout << " [" << action.find("actionType")->as_string() << ":"
                  << action.find("actionStatus")->as_string() << "]";
      }
      for (const Json & error : json.find("errors")->as_array()) {
        std::cout << " ERROR(" << error.find("errorType")->as_string() << ")";
      }
    }
    std::cout << "\n";
  }

  Json last_state;
};

/// Remembers the requested goal; the main loop "drives" there one step later.
class SimulatedNavigation : public NavigationPort
{
public:
  void navigate_to(const Node & target, const Edge &) override {goal = target;}
  void cancel() override {}
  void pause() override {}
  void resume() override {}
  std::optional<Node> goal;
};

class SimulatedActions : public ActionPort
{
public:
  bool supports(const std::string & action_type) const override {return action_type == "pick";}
  void start(const Action & action) override {running = action.action_id;}
  void cancel(const std::string &) override {running.reset();}
  std::optional<std::string> running;
};

const char kHeader[] =
  R"("headerId":0,"timestamp":"2026-11-26T20:00:00.000Z","version":"2.0.0",)"
  R"("manufacturer":"acme","serialNumber":"amr-001",)";

std::string node(const char * id, int sequence, bool released, double x, const char * actions)
{
  char buffer[512];
  std::snprintf(
    buffer, sizeof(buffer),
    R"({"nodeId":"%s","sequenceId":%d,"released":%s,"actions":[%s],)"
    R"("nodePosition":{"x":%.1f,"y":0,"mapId":"warehouse"}})",
    id, sequence, released ? "true" : "false", actions, x);
  return buffer;
}

std::string edge(const char * id, int sequence, bool released, const char * from, const char * to)
{
  char buffer[256];
  std::snprintf(
    buffer, sizeof(buffer),
    R"({"edgeId":"%s","sequenceId":%d,"released":%s,"startNodeId":"%s","endNodeId":"%s",)"
    R"("actions":[]})",
    id, sequence, released ? "true" : "false", from, to);
  return buffer;
}

}  // namespace

int main()
{
  Identity identity;
  identity.manufacturer = "acme";
  identity.serial_number = "amr-001";

  PrintingTransport transport;
  SimulatedNavigation navigation;
  SimulatedActions actions;
  Session session(identity, transport, navigation, actions);

  AgvPosition position;
  position.map_id = "warehouse";
  position.position_initialized = true;
  session.orders().set_position(position);

  Session::TimePoint now{};
  const auto step = [&] {
      // The simulated robot finishes one piece of work per step.
      if (navigation.goal) {
        position.x = navigation.goal->node_position->x;
        session.orders().set_position(position);
        navigation.goal.reset();
        std::cout << "  (robot arrived)\n";
        session.orders().on_navigation_succeeded();
      } else if (actions.running) {
        const std::string id = *actions.running;
        actions.running.reset();
        std::cout << "  (robot finished action)\n";
        session.orders().on_action_status(id, ActionStatus::Finished);
      }
      now += std::chrono::milliseconds(100);
      session.tick(now);
    };

  std::cout << "connect\n";
  session.on_connected(now);

  const char pick[] = R"({"actionType":"pick","actionId":"pick-1","blockingType":"HARD"})";

  std::cout << "master -> order o1/0: N0 -> N1 [pick] released, N2 on horizon\n";
  session.on_message(
    topic_name(identity, "order"),
    std::string("{") + kHeader + R"("orderId":"o1","orderUpdateId":0,"nodes":[)" +
    node("N0", 0, true, 0.0, "") + "," + node("N1", 2, true, 5.0, pick) + "," +
    node("N2", 4, false, 9.0, "") + R"(],"edges":[)" +
    edge("E0", 1, true, "N0", "N1") + "," + edge("E1", 3, false, "N1", "N2") + "]}");
  session.tick(now);
  for (int i = 0; i < 4; ++i) {
    step();
  }

  std::cout << "master -> order o1/1: releases N1 -> N2\n";
  session.on_message(
    topic_name(identity, "order"),
    std::string("{") + kHeader + R"("orderId":"o1","orderUpdateId":1,"nodes":[)" +
    node("N1", 2, true, 5.0, "") + "," + node("N2", 4, true, 9.0, "") + R"(],"edges":[)" +
    edge("E1", 3, true, "N1", "N2") + "]}");
  session.tick(now);
  for (int i = 0; i < 3; ++i) {
    step();
  }

  std::cout << "disconnect\n";
  session.shutdown();

  const Json & state = transport.last_state;
  const bool ok =
    state.find("lastNodeId")->as_string() == "N2" &&
    state.find("nodeStates")->as_array().empty() &&
    !state.find("driving")->as_bool() &&
    state.find("errors")->as_array().empty() &&
    state.find("actionStates")->as_array()[0].find("actionStatus")->as_string() == "FINISHED";
  std::cout << (ok ? "RESULT: order completed\n" : "RESULT: FAILED\n");
  return ok ? 0 : 1;
}
