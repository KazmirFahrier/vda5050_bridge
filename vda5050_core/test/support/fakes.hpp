// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Test doubles for the ports, plus builders for orders. The fakes only record
// what OrderManager asked for; each test then plays the robot by calling
// on_navigation_succeeded(), on_action_status() and so on.

#ifndef SUPPORT__FAKES_HPP_
#define SUPPORT__FAKES_HPP_

#include <set>
#include <string>
#include <vector>

#include "vda5050_core/order_manager.hpp"

namespace test_support
{

class FakeNavigation : public vda5050_core::NavigationPort
{
public:
  void navigate_to(const vda5050_core::Node & target, const vda5050_core::Edge & via) override
  {
    goals.push_back(target.node_id);
    edges.push_back(via.edge_id);
  }
  void cancel() override {++cancel_calls;}
  void pause() override {++pause_calls;}
  void resume() override {++resume_calls;}

  std::vector<std::string> goals;
  std::vector<std::string> edges;
  int cancel_calls{0};
  int pause_calls{0};
  int resume_calls{0};
};

class FakeActions : public vda5050_core::ActionPort
{
public:
  bool supports(const std::string & action_type) const override
  {
    return supported.count(action_type) > 0;
  }
  void start(const vda5050_core::Action & action) override {started.push_back(action.action_id);}
  void cancel(const std::string & action_id) override {cancelled.push_back(action_id);}

  std::set<std::string> supported{"pick", "drop", "beep"};
  std::vector<std::string> started;
  std::vector<std::string> cancelled;
};

inline vda5050_core::Action make_action(
  const std::string & id, const std::string & type, vda5050_core::BlockingType blocking)
{
  vda5050_core::Action action;
  action.action_id = id;
  action.action_type = type;
  action.blocking_type = blocking;
  return action;
}

/// Build a straight-line order: nodes N<first>..N<first+count-1>, one metre
/// apart along x on map "map", joined by edges E<i>. The first `released`
/// nodes form the base; the rest are horizon.
inline vda5050_core::Order make_order(
  const std::string & order_id, std::uint32_t update_id, std::size_t count, std::size_t released,
  std::size_t first = 0)
{
  vda5050_core::Order order;
  order.order_id = order_id;
  order.order_update_id = update_id;
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t number = first + i;
    vda5050_core::Node node;
    node.node_id = "N" + std::to_string(number);
    node.sequence_id = static_cast<std::uint32_t>(2 * number);
    node.released = i < released;
    vda5050_core::NodePosition position;
    position.x = static_cast<double>(number);
    position.y = 0.0;
    position.map_id = "map";
    node.node_position = position;
    order.nodes.push_back(node);
    if (i > 0) {
      vda5050_core::Edge edge;
      edge.edge_id = "E" + std::to_string(number - 1);
      edge.sequence_id = static_cast<std::uint32_t>(2 * number - 1);
      edge.released = node.released;
      edge.start_node_id = "N" + std::to_string(number - 1);
      edge.end_node_id = node.node_id;
      order.edges.push_back(edge);
    }
  }
  return order;
}

inline vda5050_core::AgvPosition position_at(double x, double y)
{
  vda5050_core::AgvPosition position;
  position.x = x;
  position.y = y;
  position.map_id = "map";
  position.position_initialized = true;
  return position;
}

}  // namespace test_support

#endif  // SUPPORT__FAKES_HPP_
