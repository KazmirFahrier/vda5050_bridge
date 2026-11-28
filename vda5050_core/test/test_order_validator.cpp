// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "fakes.hpp"
#include "vda5050_core/order_validator.hpp"

using namespace vda5050_core;  // NOLINT(build/namespaces)
using test_support::make_action;
using test_support::make_order;

namespace
{

bool mentions(const std::vector<std::string> & problems, const std::string & text)
{
  for (const std::string & problem : problems) {
    if (problem.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(OrderValidator, AcceptsWellFormedOrders)
{
  EXPECT_TRUE(validate_order(make_order("o", 0, 1, 1)).empty());  // single node
  EXPECT_TRUE(validate_order(make_order("o", 0, 4, 4)).empty());  // all base
  EXPECT_TRUE(validate_order(make_order("o", 0, 4, 2)).empty());  // base + horizon
  EXPECT_TRUE(validate_order(make_order("o", 1, 3, 3, 5)).empty());  // update starting at N5
}

TEST(OrderValidator, RejectsEmptyOrderIdAndMissingNodes)
{
  Order order = make_order("", 0, 2, 2);
  EXPECT_TRUE(mentions(validate_order(order), "orderId must not be empty"));

  order = make_order("o", 0, 2, 2);
  order.nodes.clear();
  EXPECT_TRUE(mentions(validate_order(order), "at least one node"));
}

TEST(OrderValidator, RejectsWrongEdgeCount)
{
  Order order = make_order("o", 0, 3, 3);
  order.edges.pop_back();
  EXPECT_TRUE(mentions(validate_order(order), "expected 2 edges for 3 nodes, got 1"));
}

TEST(OrderValidator, RejectsBadSequenceIds)
{
  Order order = make_order("o", 0, 3, 3);
  order.nodes[1].sequence_id = 3;
  EXPECT_TRUE(mentions(validate_order(order), "nodes[1]: sequenceId must be even"));

  order = make_order("o", 0, 3, 3);
  order.nodes[2].sequence_id = 8;
  EXPECT_TRUE(mentions(validate_order(order), "nodes[2]: sequenceId must be previous"));

  order = make_order("o", 0, 3, 3);
  order.edges[0].sequence_id = 5;
  EXPECT_TRUE(mentions(validate_order(order), "edges[0]: sequenceId must be start node"));
}

TEST(OrderValidator, RejectsEdgesThatDoNotConnectTheirNodes)
{
  Order order = make_order("o", 0, 3, 3);
  order.edges[1].start_node_id = "elsewhere";
  EXPECT_TRUE(mentions(validate_order(order), "edges[1]: startNodeId"));

  order = make_order("o", 0, 3, 3);
  order.edges[1].end_node_id = "elsewhere";
  EXPECT_TRUE(mentions(validate_order(order), "edges[1]: endNodeId"));
}

TEST(OrderValidator, RejectsBrokenBaseHorizonStructure)
{
  Order order = make_order("o", 0, 3, 3);
  order.nodes[0].released = false;
  EXPECT_TRUE(mentions(validate_order(order), "first node must be released"));

  // released, unreleased, released: the base has a hole in it.
  order = make_order("o", 0, 3, 3);
  order.nodes[1].released = false;
  order.edges[0].released = false;
  EXPECT_TRUE(mentions(validate_order(order), "nodes[2]: released node after an unreleased"));

  order = make_order("o", 0, 3, 3);
  order.edges[1].released = false;
  EXPECT_TRUE(mentions(validate_order(order), "edges[1]: released flag"));
}

TEST(OrderValidator, RejectsDuplicateOrEmptyActionIds)
{
  Order order = make_order("o", 0, 3, 3);
  order.nodes[0].actions.push_back(make_action("a1", "pick", BlockingType::Hard));
  order.edges[1].actions.push_back(make_action("a1", "beep", BlockingType::None));
  EXPECT_TRUE(mentions(validate_order(order), "duplicate actionId 'a1'"));

  order = make_order("o", 0, 2, 2);
  order.nodes[1].actions.push_back(make_action("", "pick", BlockingType::Hard));
  EXPECT_TRUE(mentions(validate_order(order), "actionId must not be empty"));
}

TEST(OrderValidator, ReportsEveryProblemNotJustTheFirst)
{
  Order order = make_order("o", 0, 3, 3);
  order.nodes[1].sequence_id = 3;
  order.edges[1].end_node_id = "elsewhere";
  EXPECT_GE(validate_order(order).size(), 2u);
}

TEST(OrderValidator, RejectsEmptyIdentifiers)
{
  Order order = make_order("o", 0, 3, 3);
  order.nodes[1].node_id = "";
  order.edges[0].end_node_id = "";  // keep the edge consistent with its node
  order.edges[1].start_node_id = "";
  EXPECT_TRUE(mentions(validate_order(order), "nodes[1]: nodeId must not be empty"));

  order = make_order("o", 0, 3, 3);
  order.edges[1].edge_id = "";
  EXPECT_TRUE(mentions(validate_order(order), "edges[1]: edgeId must not be empty"));

  order = make_order("o", 0, 2, 2);
  order.nodes[0].actions.push_back(make_action("a1", "", BlockingType::Hard));
  EXPECT_TRUE(mentions(validate_order(order), "actionType must not be empty"));
}
