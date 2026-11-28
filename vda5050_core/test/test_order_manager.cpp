// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// Behavioural tests for the order state machine. Each test plays both sides:
// master control (handle_order / handle_instant_actions) and the robot
// (on_navigation_*, on_action_status), then checks the port calls and the
// published state.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "fakes.hpp"
#include "vda5050_core/order_manager.hpp"

using namespace vda5050_core;  // NOLINT(build/namespaces)
using test_support::FakeActions;
using test_support::FakeNavigation;
using test_support::make_action;
using test_support::make_order;
using test_support::position_at;

class OrderManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    manager.set_position(position_at(0.0, 0.0));  // standing on N0
  }

  InstantActions instant(const std::string & id, const std::string & type)
  {
    InstantActions message;
    message.actions.push_back(make_action(id, type, BlockingType::Hard));
    return message;
  }

  ActionStatus status_of(const std::string & action_id)
  {
    for (const ActionState & state : manager.snapshot().action_states) {
      if (state.action_id == action_id) {
        return state.action_status;
      }
    }
    report_missing_action(action_id);
    return ActionStatus::Failed;
  }

  void report_missing_action(const std::string & action_id)
  {
    EXPECT_TRUE(false) << "no action state for " << action_id;
  }

  bool has_error(const std::string & type)
  {
    for (const Error & error : manager.snapshot().errors) {
      if (error.error_type == type) {
        return true;
      }
    }
    return false;
  }

  FakeNavigation navigation;
  FakeActions actions;
  OrderManager manager{navigation, actions};
};

// ------------------------------------------------------------ traversal ----

TEST_F(OrderManagerTest, IdleVehicleReportsEmptyState)
{
  const State state = manager.snapshot();
  EXPECT_EQ(state.order_id, "");
  EXPECT_FALSE(state.driving);
  EXPECT_TRUE(state.node_states.empty());
  EXPECT_FALSE(manager.has_active_order());
  EXPECT_FALSE(manager.consume_state_changed());
}

TEST_F(OrderManagerTest, AcceptsNewOrderAndDrivesToNextNode)
{
  manager.handle_order(make_order("o1", 0, 3, 3));

  ASSERT_EQ(navigation.goals.size(), 1u);
  EXPECT_EQ(navigation.goals[0], "N1");
  EXPECT_EQ(navigation.edges[0], "E0");

  const State state = manager.snapshot();
  EXPECT_EQ(state.order_id, "o1");
  EXPECT_EQ(state.last_node_id, "N0");  // first node counts as reached at once
  EXPECT_EQ(state.last_node_sequence_id, 0u);
  EXPECT_TRUE(state.driving);
  ASSERT_EQ(state.node_states.size(), 2u);  // N1 and N2 still ahead
  EXPECT_EQ(state.node_states[0].node_id, "N1");
  EXPECT_EQ(state.edge_states.size(), 2u);
  EXPECT_TRUE(state.errors.empty());
  EXPECT_TRUE(manager.consume_state_changed());
  EXPECT_FALSE(manager.consume_state_changed());  // flag is cleared by reading it
}

TEST_F(OrderManagerTest, VisitsEveryNodeThenBecomesIdle)
{
  manager.handle_order(make_order("o1", 0, 3, 3));

  manager.on_navigation_succeeded();  // arrived at N1
  EXPECT_EQ(manager.snapshot().last_node_id, "N1");
  ASSERT_EQ(navigation.goals.size(), 2u);
  EXPECT_EQ(navigation.goals[1], "N2");

  manager.on_navigation_succeeded();  // arrived at N2
  const State state = manager.snapshot();
  EXPECT_EQ(state.last_node_id, "N2");
  EXPECT_EQ(state.last_node_sequence_id, 4u);
  EXPECT_FALSE(state.driving);
  EXPECT_TRUE(state.node_states.empty());
  EXPECT_TRUE(state.edge_states.empty());
  EXPECT_EQ(state.order_id, "o1");  // kept until the next order arrives
  EXPECT_FALSE(manager.has_active_order());
  EXPECT_EQ(navigation.goals.size(), 2u);
}

TEST_F(OrderManagerTest, SingleNodeOrderCompletesImmediately)
{
  manager.handle_order(make_order("o1", 0, 1, 1));
  EXPECT_TRUE(navigation.goals.empty());
  EXPECT_FALSE(manager.has_active_order());
  EXPECT_EQ(manager.snapshot().last_node_id, "N0");
}

TEST_F(OrderManagerTest, StaleNavigationResultIsIgnored)
{
  manager.on_navigation_succeeded();  // nothing is driving
  manager.on_navigation_failed("late");
  EXPECT_TRUE(manager.snapshot().errors.empty());
}

// ------------------------------------------------------ order acceptance ----

TEST_F(OrderManagerTest, RejectsOrderWhenVehicleIsNotAtFirstNode)
{
  manager.set_position(position_at(5.0, 5.0));
  manager.handle_order(make_order("o1", 0, 3, 3));

  EXPECT_TRUE(navigation.goals.empty());
  EXPECT_TRUE(has_error(error_type::kNoRoute));
  EXPECT_EQ(manager.snapshot().order_id, "");
  EXPECT_TRUE(manager.consume_state_changed());  // the rejection is reported promptly
}

TEST_F(OrderManagerTest, FirstNodeDeviationComesFromNodeOrConfig)
{
  manager.set_position(position_at(0.4, 0.0));  // inside the 0.5 m default
  Order order = make_order("o1", 0, 2, 2);
  order.nodes[0].node_position->allowed_deviation_xy = 0.1;  // node is stricter
  manager.handle_order(order);
  EXPECT_TRUE(has_error(error_type::kNoRoute));

  manager.handle_order(make_order("o2", 0, 2, 2));  // falls back to the default
  EXPECT_EQ(manager.snapshot().order_id, "o2");
}

TEST_F(OrderManagerTest, RejectsOrderOnDifferentMap)
{
  Order order = make_order("o1", 0, 2, 2);
  order.nodes[0].node_position->map_id = "other-floor";
  manager.handle_order(order);
  EXPECT_TRUE(has_error(error_type::kNoRoute));
}

TEST_F(OrderManagerTest, WithoutCoordinatesFirstNodeMustBeLastNode)
{
  manager.handle_order(make_order("o1", 0, 2, 2));
  manager.on_navigation_succeeded();  // now at N1, idle

  Order next = make_order("o2", 0, 2, 2, 1);  // N1 -> N2
  next.nodes[0].node_position.reset();
  manager.handle_order(next);
  EXPECT_EQ(manager.snapshot().order_id, "o2");

  manager.on_navigation_succeeded();  // at N2
  Order wrong = make_order("o3", 0, 2, 2, 7);  // starts at N7
  wrong.nodes[0].node_position.reset();
  manager.handle_order(wrong);
  EXPECT_TRUE(has_error(error_type::kNoRoute));
  EXPECT_EQ(manager.snapshot().order_id, "o2");
}

TEST_F(OrderManagerTest, RejectsNewOrderWhileBusyWithoutDisturbingCurrentOrder)
{
  manager.handle_order(make_order("o1", 0, 3, 3));
  manager.handle_order(make_order("o2", 0, 3, 3));

  EXPECT_TRUE(has_error(error_type::kOrder));
  const State state = manager.snapshot();
  EXPECT_EQ(state.order_id, "o1");
  EXPECT_TRUE(state.driving);
  EXPECT_EQ(navigation.goals.size(), 1u);
  EXPECT_EQ(navigation.cancel_calls, 0);
}

TEST_F(OrderManagerTest, RejectsMalformedOrderWithValidationError)
{
  Order order = make_order("o1", 0, 3, 3);
  order.nodes[1].sequence_id = 7;
  manager.handle_order(order);
  EXPECT_TRUE(has_error(error_type::kValidation));
  EXPECT_TRUE(navigation.goals.empty());

  // The error names the order it refers to.
  const Error error = manager.snapshot().errors.front();
  ASSERT_EQ(error.error_references.size(), 2u);
  EXPECT_EQ(error.error_references[0].reference_key, "orderId");
  EXPECT_EQ(error.error_references[0].reference_value, "o1");
  EXPECT_TRUE(error.error_level == ErrorLevel::Warning);
}

TEST_F(OrderManagerTest, RejectsOrderWithUnsupportedActionBeforeMoving)
{
  Order order = make_order("o1", 0, 3, 3);
  order.nodes[2].actions.push_back(make_action("a1", "teleport", BlockingType::Hard));
  manager.handle_order(order);
  EXPECT_TRUE(has_error(error_type::kOrder));
  EXPECT_TRUE(navigation.goals.empty());
  EXPECT_FALSE(manager.has_active_order());
}

TEST_F(OrderManagerTest, RejectsOrdersOutsideAutomaticModes)
{
  manager.set_operating_mode(OperatingMode::Manual);
  manager.handle_order(make_order("o1", 0, 2, 2));
  EXPECT_TRUE(has_error(error_type::kOrder));
  EXPECT_TRUE(navigation.goals.empty());
  EXPECT_EQ(manager.snapshot().order_id, "");

  manager.set_operating_mode(OperatingMode::Semiautomatic);
  manager.handle_order(make_order("o1", 0, 2, 2));
  EXPECT_EQ(manager.snapshot().order_id, "o1");
}

TEST_F(OrderManagerTest, RejectsOrderWithUnsupportedEdgeAction)
{
  Order order = make_order("o1", 0, 3, 3);
  order.edges[1].actions.push_back(make_action("a1", "teleport", BlockingType::None));
  manager.handle_order(order);
  EXPECT_TRUE(has_error(error_type::kOrder));
  EXPECT_TRUE(navigation.goals.empty());
}

TEST_F(OrderManagerTest, UpdateRepeatingAKnownActionDoesNotDuplicateItsState)
{
  Order order = make_order("o1", 0, 3, 3);
  order.nodes[2].actions.push_back(make_action("a1", "drop", BlockingType::Hard));
  manager.handle_order(order);
  manager.on_navigation_succeeded();  // at N1; base still reaches N2

  // The update stitches at N2 and (as a master may) repeats N2's action on the
  // node it re-sends... which is the stitching node, so it is not re-added.
  Order update = make_order("o1", 1, 2, 2, 2);  // N2 -> N3
  update.nodes[0].actions.push_back(make_action("a1", "drop", BlockingType::Hard));
  update.nodes[1].actions.push_back(make_action("a1b", "pick", BlockingType::Hard));
  manager.handle_order(update);

  int count = 0;
  for (const ActionState & state : manager.snapshot().action_states) {
    count += state.action_id == "a1" ? 1 : 0;
  }
  EXPECT_EQ(count, 1);
  EXPECT_TRUE(status_of("a1b") == ActionStatus::Waiting);
}

TEST_F(OrderManagerTest, NewOrderClearsOldErrorsAndActionStates)
{
  manager.handle_instant_actions(instant("c0", "cancelOrder"));  // fails: nothing to cancel
  EXPECT_TRUE(has_error(error_type::kNoOrderToCancel));
  EXPECT_EQ(manager.snapshot().action_states.size(), 1u);

  manager.handle_order(make_order("o1", 0, 2, 2));
  EXPECT_TRUE(manager.snapshot().errors.empty());
  EXPECT_TRUE(manager.snapshot().action_states.empty());
}

TEST_F(OrderManagerTest, ErrorListIsBounded)
{
  OrderManagerConfig config;
  config.max_errors = 3;
  OrderManager small(navigation, actions, config);
  for (int i = 0; i < 10; ++i) {
    small.report_malformed_message("order", "bad payload " + std::to_string(i));
  }
  const State state = small.snapshot();
  ASSERT_EQ(state.errors.size(), 3u);
  EXPECT_EQ(state.errors.back().error_description, "bad payload 9");  // newest kept
}

// --------------------------------------------------------- order updates ----

TEST_F(OrderManagerTest, StopsAtEndOfBaseAndRequestsMore)
{
  manager.handle_order(make_order("o1", 0, 4, 2));  // base N0-N1, horizon N2-N3
  EXPECT_TRUE(manager.snapshot().new_base_request);  // only one released node ahead

  manager.on_navigation_succeeded();  // at N1: end of base
  const State state = manager.snapshot();
  EXPECT_FALSE(state.driving);
  EXPECT_EQ(navigation.goals.size(), 1u);  // did not drive into the horizon
  ASSERT_EQ(state.node_states.size(), 2u);
  EXPECT_FALSE(state.node_states[0].released);
  EXPECT_TRUE(state.new_base_request);
  EXPECT_TRUE(manager.has_active_order());  // still waiting, not finished
}

TEST_F(OrderManagerTest, UpdateReleasesHorizonAndVehicleContinues)
{
  manager.handle_order(make_order("o1", 0, 4, 2));
  manager.on_navigation_succeeded();  // waiting at N1

  // Update stitches at N1 (last base node) and releases N2, N3.
  manager.handle_order(make_order("o1", 1, 3, 3, 1));

  const State state = manager.snapshot();
  EXPECT_EQ(state.order_update_id, 1u);
  EXPECT_TRUE(state.errors.empty());
  ASSERT_EQ(state.node_states.size(), 2u);  // N2, N3 - no duplicates of the old horizon
  EXPECT_EQ(state.node_states[0].node_id, "N2");
  EXPECT_TRUE(state.node_states[0].released);
  EXPECT_EQ(state.edge_states.size(), 2u);
  ASSERT_EQ(navigation.goals.size(), 2u);
  EXPECT_EQ(navigation.goals[1], "N2");
  EXPECT_FALSE(state.new_base_request);
}

TEST_F(OrderManagerTest, UpdateWhileDrivingExtendsBaseWithoutInterruptingMotion)
{
  manager.handle_order(make_order("o1", 0, 3, 2));  // driving N0 -> N1, N2 is horizon
  manager.handle_order(make_order("o1", 1, 2, 2, 1));  // N1 -> N2 released

  EXPECT_EQ(navigation.goals.size(), 1u);  // current goal untouched
  EXPECT_EQ(navigation.cancel_calls, 0);
  manager.on_navigation_succeeded();
  ASSERT_EQ(navigation.goals.size(), 2u);
  EXPECT_EQ(navigation.goals[1], "N2");
}

TEST_F(OrderManagerTest, UpdateCanReplaceHorizonWithADifferentRoute)
{
  manager.handle_order(make_order("o1", 0, 3, 2));  // horizon: N2

  Order update = make_order("o1", 1, 2, 2, 1);  // N1 -> (renamed) X
  update.nodes[1].node_id = "X";
  update.edges[0].end_node_id = "X";
  manager.handle_order(update);

  const State state = manager.snapshot();
  ASSERT_EQ(state.node_states.size(), 2u);
  EXPECT_EQ(state.node_states[0].node_id, "N1");
  EXPECT_EQ(state.node_states[1].node_id, "X");  // old horizon N2 is gone
}

TEST_F(OrderManagerTest, DuplicateOrderIsIgnoredSilently)
{
  const Order order = make_order("o1", 0, 3, 3);
  manager.handle_order(order);
  manager.consume_state_changed();

  manager.handle_order(order);  // redelivery

  EXPECT_TRUE(manager.snapshot().errors.empty());
  EXPECT_EQ(navigation.goals.size(), 1u);
  EXPECT_FALSE(manager.consume_state_changed());
}

TEST_F(OrderManagerTest, RejectsStaleUpdate)
{
  manager.handle_order(make_order("o1", 0, 3, 2));
  manager.handle_order(make_order("o1", 2, 2, 2, 1));
  manager.handle_order(make_order("o1", 1, 2, 2, 1));  // older than 2

  EXPECT_TRUE(has_error(error_type::kOrderUpdate));
  EXPECT_EQ(manager.snapshot().order_update_id, 2u);
}

TEST_F(OrderManagerTest, RejectsUpdateThatDoesNotStartAtEndOfBase)
{
  manager.handle_order(make_order("o1", 0, 4, 2));  // base ends at N1
  manager.handle_order(make_order("o1", 1, 2, 2, 2));  // starts at N2: gap

  EXPECT_TRUE(has_error(error_type::kOrderUpdate));
  const State state = manager.snapshot();
  EXPECT_EQ(state.order_update_id, 0u);
  EXPECT_EQ(state.node_states.size(), 3u);  // untouched
}

TEST_F(OrderManagerTest, UpdateAfterCompletionContinuesFromLastNode)
{
  manager.handle_order(make_order("o1", 0, 2, 2));
  manager.on_navigation_succeeded();  // order complete at N1
  EXPECT_FALSE(manager.has_active_order());

  manager.handle_order(make_order("o1", 1, 2, 2, 1));  // N1 -> N2
  ASSERT_EQ(navigation.goals.size(), 2u);
  EXPECT_EQ(navigation.goals[1], "N2");
  EXPECT_TRUE(manager.snapshot().errors.empty());
}

// ---------------------------------------------------------------- actions ----

TEST_F(OrderManagerTest, HardNodeActionBlocksDrivingUntilFinished)
{
  Order order = make_order("o1", 0, 2, 2);
  order.nodes[0].actions.push_back(make_action("a1", "pick", BlockingType::Hard));
  manager.handle_order(order);

  ASSERT_EQ(actions.started.size(), 1u);
  EXPECT_TRUE(status_of("a1") == ActionStatus::Initializing);
  EXPECT_TRUE(navigation.goals.empty());
  EXPECT_FALSE(manager.snapshot().driving);

  manager.on_action_status("a1", ActionStatus::Running);
  EXPECT_TRUE(navigation.goals.empty());

  manager.on_action_status("a1", ActionStatus::Finished, "picked");
  EXPECT_TRUE(status_of("a1") == ActionStatus::Finished);
  EXPECT_EQ(*manager.snapshot().action_states[0].result_description, "picked");
  ASSERT_EQ(navigation.goals.size(), 1u);
  EXPECT_TRUE(manager.snapshot().driving);
}

TEST_F(OrderManagerTest, SoftActionBlocksDrivingButRunsAlongsideOthers)
{
  Order order = make_order("o1", 0, 2, 2);
  order.nodes[0].actions.push_back(make_action("a1", "pick", BlockingType::Soft));
  order.nodes[0].actions.push_back(make_action("a2", "beep", BlockingType::None));
  manager.handle_order(order);

  EXPECT_EQ(actions.started.size(), 2u);  // both started together
  EXPECT_TRUE(navigation.goals.empty());

  manager.on_action_status("a1", ActionStatus::Finished);
  EXPECT_EQ(navigation.goals.size(), 1u);  // a2 (NONE) may still be running
  EXPECT_TRUE(status_of("a2") == ActionStatus::Initializing);
}

TEST_F(OrderManagerTest, NoneActionDoesNotBlockDriving)
{
  Order order = make_order("o1", 0, 2, 2);
  order.nodes[0].actions.push_back(make_action("a1", "beep", BlockingType::None));
  manager.handle_order(order);

  EXPECT_EQ(actions.started.size(), 1u);
  EXPECT_EQ(navigation.goals.size(), 1u);
  EXPECT_TRUE(manager.snapshot().driving);
}

TEST_F(OrderManagerTest, HardActionWaitsForRunningActionsAndKeepsOrder)
{
  Order order = make_order("o1", 0, 2, 2);
  order.nodes[0].actions.push_back(make_action("a1", "beep", BlockingType::None));
  order.nodes[0].actions.push_back(make_action("a2", "pick", BlockingType::Hard));
  order.nodes[0].actions.push_back(make_action("a3", "beep", BlockingType::None));
  manager.handle_order(order);

  // a1 runs; a2 (HARD) must run alone so it waits; a3 must not overtake a2.
  ASSERT_EQ(actions.started.size(), 1u);
  EXPECT_EQ(actions.started[0], "a1");
  EXPECT_TRUE(status_of("a2") == ActionStatus::Waiting);
  EXPECT_TRUE(navigation.goals.empty());

  manager.on_action_status("a1", ActionStatus::Finished);
  ASSERT_EQ(actions.started.size(), 2u);
  EXPECT_EQ(actions.started[1], "a2");
  EXPECT_TRUE(status_of("a3") == ActionStatus::Waiting);

  manager.on_action_status("a2", ActionStatus::Finished);
  ASSERT_EQ(actions.started.size(), 3u);
  EXPECT_EQ(actions.started[2], "a3");
  EXPECT_EQ(navigation.goals.size(), 1u);  // a3 is NONE: driving may begin
}

TEST_F(OrderManagerTest, ActionsOnLaterNodesStartOnlyOnArrival)
{
  Order order = make_order("o1", 0, 3, 3);
  order.nodes[1].actions.push_back(make_action("a1", "drop", BlockingType::Hard));
  manager.handle_order(order);

  EXPECT_TRUE(actions.started.empty());
  EXPECT_TRUE(status_of("a1") == ActionStatus::Waiting);  // visible to master from the start

  manager.on_navigation_succeeded();  // at N1
  ASSERT_EQ(actions.started.size(), 1u);
  EXPECT_EQ(navigation.goals.size(), 1u);  // held at N1

  manager.on_action_status("a1", ActionStatus::Finished);
  EXPECT_EQ(navigation.goals.size(), 2u);
}

TEST_F(OrderManagerTest, BlockingEdgeActionRunsBeforeEnteringEdge)
{
  Order order = make_order("o1", 0, 2, 2);
  order.edges[0].actions.push_back(make_action("a1", "beep", BlockingType::Hard));
  manager.handle_order(order);

  ASSERT_EQ(actions.started.size(), 1u);
  EXPECT_TRUE(navigation.goals.empty());
  manager.on_action_status("a1", ActionStatus::Finished);
  EXPECT_EQ(navigation.goals.size(), 1u);
}

TEST_F(OrderManagerTest, HorizonActionsAreNotTrackedUntilReleased)
{
  Order order = make_order("o1", 0, 3, 2);
  order.nodes[2].actions.push_back(make_action("a1", "drop", BlockingType::Hard));
  manager.handle_order(order);
  EXPECT_TRUE(manager.snapshot().action_states.empty());

  Order update = make_order("o1", 1, 2, 2, 1);
  update.nodes[1].actions.push_back(make_action("a1", "drop", BlockingType::Hard));
  manager.handle_order(update);
  EXPECT_TRUE(status_of("a1") == ActionStatus::Waiting);
}

TEST_F(OrderManagerTest, FailedActionIsReportedAndOrderContinues)
{
  Order order = make_order("o1", 0, 2, 2);
  order.nodes[0].actions.push_back(make_action("a1", "pick", BlockingType::Hard));
  manager.handle_order(order);

  manager.on_action_status("a1", ActionStatus::Failed, "gripper jammed");
  EXPECT_TRUE(status_of("a1") == ActionStatus::Failed);
  EXPECT_TRUE(has_error(error_type::kAction));
  EXPECT_EQ(navigation.goals.size(), 1u);
}

TEST_F(OrderManagerTest, LateOrUnknownActionReportsAreIgnored)
{
  Order order = make_order("o1", 0, 2, 2);
  order.nodes[0].actions.push_back(make_action("a1", "pick", BlockingType::Hard));
  manager.handle_order(order);
  manager.on_action_status("a1", ActionStatus::Finished);

  manager.on_action_status("a1", ActionStatus::Failed, "too late");  // terminal stays terminal
  manager.on_action_status("ghost", ActionStatus::Finished);
  EXPECT_TRUE(status_of("a1") == ActionStatus::Finished);
  EXPECT_FALSE(has_error(error_type::kAction));
}

TEST_F(OrderManagerTest, OrderStaysActiveWhileNoneActionStillRuns)
{
  Order order = make_order("o1", 0, 1, 1);
  order.nodes[0].actions.push_back(make_action("a1", "beep", BlockingType::None));
  manager.handle_order(order);
  EXPECT_TRUE(manager.has_active_order());
  manager.on_action_status("a1", ActionStatus::Finished);
  EXPECT_FALSE(manager.has_active_order());
}

// -------------------------------------------------------- instant actions ----

TEST_F(OrderManagerTest, CancelWhileDrivingCompletesWhenVehicleStops)
{
  Order order = make_order("o1", 0, 3, 3);
  order.nodes[1].actions.push_back(make_action("a1", "drop", BlockingType::Hard));
  manager.handle_order(order);

  manager.handle_instant_actions(instant("c1", "cancelOrder"));
  EXPECT_EQ(navigation.cancel_calls, 1);
  EXPECT_TRUE(status_of("c1") == ActionStatus::Running);  // not done until stopped
  EXPECT_TRUE(status_of("a1") == ActionStatus::Failed);  // never started
  EXPECT_TRUE(manager.has_active_order());

  manager.on_navigation_failed("goal cancelled");  // the robot confirms it stopped

  const State state = manager.snapshot();
  EXPECT_TRUE(status_of("c1") == ActionStatus::Finished);
  EXPECT_TRUE(state.node_states.empty());
  EXPECT_TRUE(state.edge_states.empty());
  EXPECT_FALSE(state.driving);
  EXPECT_EQ(state.order_id, "o1");  // still names the cancelled order
  EXPECT_EQ(state.last_node_id, "N0");
  EXPECT_FALSE(has_error(error_type::kNavigation));  // a cancelled goal is not an error
  EXPECT_FALSE(manager.has_active_order());
}

TEST_F(OrderManagerTest, CancelWhileStandingFinishesImmediatelyAndStopsActions)
{
  Order order = make_order("o1", 0, 2, 2);
  order.nodes[0].actions.push_back(make_action("a1", "pick", BlockingType::Hard));
  manager.handle_order(order);
  manager.on_action_status("a1", ActionStatus::Running);

  manager.handle_instant_actions(instant("c1", "cancelOrder"));

  EXPECT_TRUE(status_of("c1") == ActionStatus::Finished);
  EXPECT_TRUE(status_of("a1") == ActionStatus::Failed);
  ASSERT_EQ(actions.cancelled.size(), 1u);
  EXPECT_EQ(actions.cancelled[0], "a1");
  EXPECT_EQ(navigation.cancel_calls, 0);
  EXPECT_FALSE(manager.has_active_order());
}

TEST_F(OrderManagerTest, CancelCompletesEvenIfTheLegFinishesFirst)
{
  // The robot reaches the node in the instant between the cancel request and
  // Nav2 acting on it: the port reports success, not failure.
  manager.handle_order(make_order("o1", 0, 3, 3));
  manager.handle_instant_actions(instant("c1", "cancelOrder"));
  manager.on_navigation_succeeded();

  const State state = manager.snapshot();
  EXPECT_TRUE(status_of("c1") == ActionStatus::Finished);
  EXPECT_TRUE(state.node_states.empty());
  EXPECT_FALSE(state.driving);
  EXPECT_EQ(navigation.goals.size(), 1u);  // did not carry on to N2
  EXPECT_FALSE(manager.has_active_order());
}

TEST_F(OrderManagerTest, SecondCancelWhileOneIsInProgressFails)
{
  manager.handle_order(make_order("o1", 0, 3, 3));
  manager.handle_instant_actions(instant("c1", "cancelOrder"));
  manager.handle_instant_actions(instant("c2", "cancelOrder"));

  EXPECT_TRUE(status_of("c1") == ActionStatus::Running);
  EXPECT_TRUE(status_of("c2") == ActionStatus::Failed);
  EXPECT_EQ(navigation.cancel_calls, 1);

  manager.on_navigation_failed("goal cancelled");
  EXPECT_TRUE(status_of("c1") == ActionStatus::Finished);
}

TEST_F(OrderManagerTest, NewOrderIsAcceptedAfterCancel)
{
  manager.handle_order(make_order("o1", 0, 3, 3));
  manager.handle_instant_actions(instant("c1", "cancelOrder"));
  manager.on_navigation_failed("goal cancelled");

  manager.handle_order(make_order("o2", 0, 2, 2));  // vehicle is still at N0
  EXPECT_EQ(manager.snapshot().order_id, "o2");
  EXPECT_EQ(navigation.goals.size(), 2u);
}

TEST_F(OrderManagerTest, CancelWithoutOrderFails)
{
  manager.handle_instant_actions(instant("c1", "cancelOrder"));
  EXPECT_TRUE(status_of("c1") == ActionStatus::Failed);
  EXPECT_TRUE(has_error(error_type::kNoOrderToCancel));
  EXPECT_EQ(navigation.cancel_calls, 0);
}

TEST_F(OrderManagerTest, RedeliveredInstantActionIsHandledOnce)
{
  manager.handle_order(make_order("o1", 0, 3, 3));
  manager.handle_instant_actions(instant("c1", "cancelOrder"));
  manager.handle_instant_actions(instant("c1", "cancelOrder"));
  EXPECT_EQ(navigation.cancel_calls, 1);
  EXPECT_EQ(manager.snapshot().action_states.size(), 1u);
}

TEST_F(OrderManagerTest, PauseHoldsVehicleAndResumeContinues)
{
  manager.handle_order(make_order("o1", 0, 3, 3));

  manager.handle_instant_actions(instant("p1", "startPause"));
  EXPECT_EQ(navigation.pause_calls, 1);
  State state = manager.snapshot();
  EXPECT_TRUE(state.paused);
  EXPECT_FALSE(state.driving);  // a paused vehicle is not driving
  EXPECT_TRUE(status_of("p1") == ActionStatus::Finished);

  manager.handle_instant_actions(instant("p2", "stopPause"));
  EXPECT_EQ(navigation.resume_calls, 1);
  state = manager.snapshot();
  EXPECT_FALSE(state.paused);
  EXPECT_TRUE(state.driving);
  EXPECT_EQ(navigation.goals.size(), 1u);  // same goal, not re-sent
}

TEST_F(OrderManagerTest, PausedVehicleDoesNotStartNextLegUntilResumed)
{
  manager.handle_order(make_order("o1", 0, 3, 3));
  manager.handle_instant_actions(instant("p1", "startPause"));
  manager.on_navigation_succeeded();  // e.g. it was already within tolerance of N1

  EXPECT_EQ(navigation.goals.size(), 1u);
  manager.handle_instant_actions(instant("p2", "stopPause"));
  EXPECT_EQ(navigation.goals.size(), 2u);
}

TEST_F(OrderManagerTest, StateRequestTriggersImmediatePublish)
{
  manager.consume_state_changed();
  manager.handle_instant_actions(instant("s1", "stateRequest"));
  EXPECT_TRUE(manager.consume_state_changed());
  EXPECT_TRUE(status_of("s1") == ActionStatus::Finished);
}

TEST_F(OrderManagerTest, FactsheetRequestIsFlaggedOnce)
{
  EXPECT_FALSE(manager.consume_factsheet_request());
  manager.handle_instant_actions(instant("f1", "factsheetRequest"));
  EXPECT_TRUE(manager.consume_factsheet_request());
  EXPECT_FALSE(manager.consume_factsheet_request());
}

TEST_F(OrderManagerTest, CustomInstantActionIsDelegatedToActionPort)
{
  manager.handle_instant_actions(instant("b1", "beep"));
  ASSERT_EQ(actions.started.size(), 1u);
  EXPECT_TRUE(status_of("b1") == ActionStatus::Initializing);
  manager.on_action_status("b1", ActionStatus::Finished);
  EXPECT_TRUE(status_of("b1") == ActionStatus::Finished);
  EXPECT_FALSE(manager.has_active_order());  // instant actions are not an order
}

TEST_F(OrderManagerTest, UnsupportedInstantActionFails)
{
  manager.handle_instant_actions(instant("x1", "teleport"));
  EXPECT_TRUE(status_of("x1") == ActionStatus::Failed);
  EXPECT_TRUE(has_error(error_type::kAction));
}

// ----------------------------------------------------------------- faults ----

TEST_F(OrderManagerTest, NavigationFailureRaisesFatalErrorAndHaltsUntilCancelled)
{
  Order order = make_order("o1", 0, 3, 3);
  order.nodes[0].actions.push_back(make_action("a1", "beep", BlockingType::None));
  manager.handle_order(order);

  manager.on_navigation_failed("path blocked");

  State state = manager.snapshot();
  EXPECT_FALSE(state.driving);
  ASSERT_EQ(state.errors.size(), 1u);
  EXPECT_EQ(state.errors[0].error_type, error_type::kNavigation);
  EXPECT_TRUE(state.errors[0].error_level == ErrorLevel::Fatal);
  EXPECT_EQ(state.last_node_id, "N0");  // N1 was not reached
  EXPECT_EQ(state.node_states.size(), 2u);

  // Unrelated events must not cause a silent retry.
  manager.on_action_status("a1", ActionStatus::Finished);
  EXPECT_EQ(navigation.goals.size(), 1u);

  // Master control resolves it: cancel, then send a fresh order.
  manager.handle_instant_actions(instant("c1", "cancelOrder"));
  EXPECT_TRUE(status_of("c1") == ActionStatus::Finished);
  manager.handle_order(make_order("o2", 0, 2, 2));
  EXPECT_EQ(manager.snapshot().order_id, "o2");
  EXPECT_EQ(navigation.goals.size(), 2u);
}

TEST(OrderManagerReentrancy, PortsMayReportResultsSynchronously)
{
  // A port that answers from inside the call that started the work. This
  // re-enters OrderManager while advance() is still on the stack.
  struct InstantNavigation : NavigationPort
  {
    void navigate_to(const Node & target, const Edge &) override
    {
      goals.push_back(target.node_id);
      manager->on_navigation_succeeded();
    }
    void cancel() override {}
    void pause() override {}
    void resume() override {}
    OrderManager * manager{nullptr};
    std::vector<std::string> goals;
  };
  struct InstantActions : ActionPort
  {
    bool supports(const std::string &) const override {return true;}
    void start(const Action & action) override
    {
      started.push_back(action.action_id);
      manager->on_action_status(action.action_id, ActionStatus::Finished);
    }
    void cancel(const std::string &) override {}
    OrderManager * manager{nullptr};
    std::vector<std::string> started;
  };

  InstantNavigation navigation;
  InstantActions actions;
  OrderManager manager(navigation, actions);
  navigation.manager = &manager;
  actions.manager = &manager;
  manager.set_position(position_at(0.0, 0.0));

  Order order = make_order("o1", 0, 3, 3);
  order.nodes[0].actions.push_back(make_action("a1", "pick", BlockingType::Hard));
  order.nodes[1].actions.push_back(make_action("a2", "drop", BlockingType::Hard));
  manager.handle_order(order);  // the whole order runs to completion inside this call

  ASSERT_EQ(navigation.goals.size(), 2u);
  EXPECT_EQ(navigation.goals[0], "N1");
  EXPECT_EQ(navigation.goals[1], "N2");
  ASSERT_EQ(actions.started.size(), 2u);
  EXPECT_EQ(actions.started[0], "a1");
  EXPECT_EQ(actions.started[1], "a2");
  const State state = manager.snapshot();
  EXPECT_EQ(state.last_node_id, "N2");
  EXPECT_TRUE(state.node_states.empty());
  EXPECT_FALSE(state.driving);
  EXPECT_FALSE(manager.has_active_order());
}

TEST_F(OrderManagerTest, MalformedMessageIsReportedAsValidationError)
{
  manager.report_malformed_message("order", "order.nodes: missing required field");
  const State state = manager.snapshot();
  ASSERT_EQ(state.errors.size(), 1u);
  EXPECT_EQ(state.errors[0].error_type, error_type::kValidation);
  EXPECT_EQ(state.errors[0].error_references[0].reference_value, "order");
  EXPECT_TRUE(manager.consume_state_changed());
}

TEST_F(OrderManagerTest, TelemetryIsMirroredIntoState)
{
  BatteryState battery;
  battery.battery_charge = 64.0;
  battery.charging = true;
  manager.set_battery(battery);
  manager.set_velocity(Velocity{0.5, 0.0, 0.1});
  manager.set_position(position_at(3.0, 4.0));
  EXPECT_FALSE(manager.consume_state_changed());  // telemetry rides on the periodic state

  manager.set_operating_mode(OperatingMode::Manual);
  EXPECT_TRUE(manager.consume_state_changed());  // mode changes are reported at once
  SafetyState safety;
  safety.e_stop = EStop::Manual;
  manager.set_safety_state(safety);
  EXPECT_TRUE(manager.consume_state_changed());
  manager.set_safety_state(safety);  // unchanged
  EXPECT_FALSE(manager.consume_state_changed());

  const State state = manager.snapshot();
  EXPECT_DOUBLE_EQ(state.battery_state.battery_charge, 64.0);
  EXPECT_TRUE(state.battery_state.charging);
  EXPECT_DOUBLE_EQ(state.velocity->vx, 0.5);
  EXPECT_DOUBLE_EQ(state.agv_position->x, 3.0);
  EXPECT_TRUE(state.operating_mode == OperatingMode::Manual);
  EXPECT_TRUE(state.safety_state.e_stop == EStop::Manual);
}

TEST_F(OrderManagerTest, ExternalErrorsAreReportedAlongsideOrderErrors)
{
  manager.consume_state_changed();
  Error fault{"LIDAR-001", {{"source", "lidar_driver"}}, "no scans", ErrorLevel::Fatal};

  manager.set_external_errors({fault});
  EXPECT_TRUE(manager.consume_state_changed());  // master control hears of it at once
  manager.set_external_errors({fault});
  EXPECT_FALSE(manager.consume_state_changed());  // the same set again is not news

  manager.report_malformed_message("order", "bad");
  State state = manager.snapshot();
  ASSERT_EQ(state.errors.size(), 2u);
  EXPECT_EQ(state.errors[0].error_type, error_type::kValidation);  // own errors first
  EXPECT_EQ(state.errors[1].error_type, "LIDAR-001");
  EXPECT_TRUE(state.errors[1].error_level == ErrorLevel::Fatal);
  EXPECT_EQ(state.errors[1].error_references[0].reference_value, "lidar_driver");

  // A new order clears the order manager's own errors, but a hardware fault
  // does not go away because master control sent something new.
  manager.handle_order(make_order("o1", 0, 1, 1));
  state = manager.snapshot();
  ASSERT_EQ(state.errors.size(), 1u);
  EXPECT_EQ(state.errors[0].error_type, "LIDAR-001");

  manager.consume_state_changed();
  fault.error_description = "scan rate 2 Hz";
  manager.set_external_errors({fault});
  EXPECT_TRUE(manager.consume_state_changed());  // any difference counts as a change
  fault.error_references[0].reference_value = "watchdog";
  manager.set_external_errors({fault});
  EXPECT_TRUE(manager.consume_state_changed());

  manager.set_external_errors({});
  EXPECT_TRUE(manager.consume_state_changed());
  EXPECT_TRUE(manager.snapshot().errors.empty());
}

TEST_F(OrderManagerTest, HoldPausesTheDriveAndReleaseResumesIt)
{
  manager.handle_order(make_order("o1", 0, 3, 3));
  manager.consume_state_changed();

  manager.set_hold(true);
  EXPECT_EQ(navigation.pause_calls, 1);
  EXPECT_TRUE(manager.held());
  EXPECT_TRUE(manager.consume_state_changed());
  State state = manager.snapshot();
  EXPECT_TRUE(state.paused);
  EXPECT_FALSE(state.driving);
  EXPECT_EQ(state.order_id, "o1");  // the order is kept

  manager.set_hold(true);  // repeated: nothing happens
  EXPECT_EQ(navigation.pause_calls, 1);
  EXPECT_FALSE(manager.consume_state_changed());

  manager.set_hold(false);
  EXPECT_EQ(navigation.resume_calls, 1);
  state = manager.snapshot();
  EXPECT_FALSE(state.paused);
  EXPECT_TRUE(state.driving);
  EXPECT_EQ(navigation.goals.size(), 1u);  // same goal, not re-sent
}

TEST_F(OrderManagerTest, HeldVehicleDoesNotStartTheNextLeg)
{
  manager.handle_order(make_order("o1", 0, 3, 3));
  manager.set_hold(true);
  manager.on_navigation_succeeded();
  EXPECT_EQ(navigation.goals.size(), 1u);

  manager.set_hold(false);
  EXPECT_EQ(navigation.goals.size(), 2u);
}

TEST_F(OrderManagerTest, HoldBeforeAnOrderDelaysItsStart)
{
  manager.set_hold(true);
  manager.handle_order(make_order("o1", 0, 3, 3));
  EXPECT_TRUE(navigation.goals.empty());  // accepted, but not started
  EXPECT_EQ(manager.snapshot().order_id, "o1");

  manager.set_hold(false);
  EXPECT_EQ(navigation.goals.size(), 1u);
}

TEST_F(OrderManagerTest, HoldAndMasterPauseAreIndependent)
{
  manager.handle_order(make_order("o1", 0, 3, 3));

  // Master pauses, then a fault holds: one pause reaches navigation.
  manager.handle_instant_actions(instant("p1", "startPause"));
  manager.set_hold(true);
  EXPECT_EQ(navigation.pause_calls, 1);

  // The fault clears, but master control has not resumed: still paused.
  manager.set_hold(false);
  EXPECT_EQ(navigation.resume_calls, 0);
  EXPECT_TRUE(manager.snapshot().paused);

  // The fault returns, then master resumes: the hold still applies.
  manager.set_hold(true);
  manager.handle_instant_actions(instant("p2", "stopPause"));
  EXPECT_EQ(navigation.resume_calls, 0);
  EXPECT_TRUE(manager.snapshot().paused);
  EXPECT_TRUE(status_of("p2") == ActionStatus::Finished);

  manager.set_hold(false);
  EXPECT_EQ(navigation.resume_calls, 1);
  EXPECT_FALSE(manager.snapshot().paused);

  // The other order: hold first, then master's pause adds no second call.
  manager.set_hold(true);
  EXPECT_EQ(navigation.pause_calls, 2);
  manager.handle_instant_actions(instant("p3", "startPause"));
  EXPECT_EQ(navigation.pause_calls, 2);
}

TEST_F(OrderManagerTest, OrderCanBeCancelledWhileHeld)
{
  manager.handle_order(make_order("o1", 0, 3, 3));
  manager.set_hold(true);
  manager.handle_instant_actions(instant("c1", "cancelOrder"));
  EXPECT_EQ(navigation.cancel_calls, 1);
  manager.on_navigation_failed("cancelled");
  EXPECT_TRUE(status_of("c1") == ActionStatus::Finished);

  manager.set_hold(false);
  EXPECT_EQ(navigation.resume_calls, 0);  // nothing left to resume
  EXPECT_FALSE(manager.has_active_order());
}
