// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_core/order_validator.hpp"

#include <set>

namespace vda5050_core
{

std::vector<std::string> validate_order(const Order & order)
{
  std::vector<std::string> problems;

  if (order.order_id.empty()) {
    problems.push_back("orderId must not be empty");
  }
  if (order.nodes.empty()) {
    problems.push_back("order must contain at least one node");
    return problems;  // nothing else can be checked
  }
  if (order.edges.size() != order.nodes.size() - 1) {
    problems.push_back(
      "expected " + std::to_string(order.nodes.size() - 1) + " edges for " +
      std::to_string(order.nodes.size()) + " nodes, got " + std::to_string(order.edges.size()));
    return problems;  // the pairing below needs matching counts
  }
  if (!order.nodes.front().released) {
    problems.push_back("first node must be released");
  }

  bool horizon_started = false;
  for (std::size_t i = 0; i < order.nodes.size(); ++i) {
    const Node & node = order.nodes[i];
    const std::string label = "nodes[" + std::to_string(i) + "]";

    if (node.node_id.empty()) {
      problems.push_back(label + ": nodeId must not be empty");
    }
    if (node.sequence_id % 2 != 0) {
      problems.push_back(label + ": sequenceId must be even");
    }
    if (i > 0 && node.sequence_id != order.nodes[i - 1].sequence_id + 2) {
      problems.push_back(label + ": sequenceId must be previous node's sequenceId + 2");
    }
    if (node.released && horizon_started) {
      problems.push_back(label + ": released node after an unreleased node");
    }
    horizon_started = horizon_started || !node.released;

    if (i == 0) {
      continue;
    }
    const Edge & edge = order.edges[i - 1];
    const Node & previous = order.nodes[i - 1];
    const std::string edge_label = "edges[" + std::to_string(i - 1) + "]";

    if (edge.edge_id.empty()) {
      problems.push_back(edge_label + ": edgeId must not be empty");
    }
    if (edge.sequence_id != previous.sequence_id + 1) {
      problems.push_back(edge_label + ": sequenceId must be start node's sequenceId + 1");
    }
    if (edge.start_node_id != previous.node_id) {
      problems.push_back(edge_label + ": startNodeId does not match the preceding node");
    }
    if (edge.end_node_id != node.node_id) {
      problems.push_back(edge_label + ": endNodeId does not match the following node");
    }
    if (edge.released != node.released) {
      problems.push_back(edge_label + ": released flag must equal that of its end node");
    }
  }

  std::set<std::string> action_ids;
  const auto check_actions = [&](const std::vector<Action> & actions, const std::string & owner) {
      for (const Action & action : actions) {
        if (action.action_id.empty()) {
          problems.push_back(owner + ": actionId must not be empty");
        } else if (!action_ids.insert(action.action_id).second) {
          problems.push_back(owner + ": duplicate actionId '" + action.action_id + "'");
        }
        if (action.action_type.empty()) {
          problems.push_back(owner + ": actionType must not be empty");
        }
      }
    };
  for (std::size_t i = 0; i < order.nodes.size(); ++i) {
    check_actions(order.nodes[i].actions, "nodes[" + std::to_string(i) + "]");
  }
  for (std::size_t i = 0; i < order.edges.size(); ++i) {
    check_actions(order.edges[i].actions, "edges[" + std::to_string(i) + "]");
  }

  return problems;
}

}  // namespace vda5050_core
