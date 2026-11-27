// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_nav2_adapter/nav2_navigation_adapter.hpp"

#include <cmath>
#include <utility>

namespace vda5050_nav2_adapter
{

Nav2NavigationAdapter::Nav2NavigationAdapter(
  rclcpp_lifecycle::LifecycleNode & node, const std::string & action_name,
  const std::string & global_frame, ResultCallback on_result,
  PositionProvider current_position)
: node_(node), global_frame_(global_frame), on_result_(std::move(on_result)),
  current_position_(std::move(current_position))
{
  client_ = rclcpp_action::create_client<NavigateToPose>(
    node_.get_node_base_interface(), node_.get_node_graph_interface(),
    node_.get_node_logging_interface(), node_.get_node_waitables_interface(), action_name);
}

void Nav2NavigationAdapter::navigate_to(
  const vda5050_core::Node & target, const vda5050_core::Edge & /*via*/)
{
  target_ = target;
  send_goal();
}

void Nav2NavigationAdapter::send_goal()
{
  if (!target_ || !target_->node_position) {
    // A node without coordinates only makes sense for line-guided vehicles.
    finish(false, "node has no nodePosition; this vehicle navigates by coordinates");
    return;
  }
  if (!client_->action_server_is_ready()) {
    finish(false, "Nav2 action server is not available");
    return;
  }

  const vda5050_core::NodePosition & position = *target_->node_position;

  // Orientation: use theta when the order gives one. Otherwise ask the robot
  // to arrive facing the way it travelled, measured from where it is now (or,
  // if it is not localised, from the previous node).
  //
  // The fallback must not be a fixed heading. Asking a robot to reach a node
  // 0.5 m to its side while ending up facing its original direction makes
  // Nav2's controller stall: in testing it failed its progress check four
  // times (40 s) before a spin recovery freed it.
  double yaw = 0.0;
  if (position.theta) {
    yaw = *position.theta;
  } else {
    std::optional<std::pair<double, double>> from = current_position_();
    if (!from && previous_position_) {
      from = std::make_pair(previous_position_->x, previous_position_->y);
    }
    // Closer than a centimetre there is no meaningful direction; keep yaw 0.
    if (from && std::hypot(position.x - from->first, position.y - from->second) > 0.01) {
      yaw = std::atan2(position.y - from->second, position.x - from->first);
    }
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = global_frame_;
  goal.pose.header.stamp = node_.now();
  goal.pose.pose.position.x = position.x;
  goal.pose.pose.position.y = position.y;
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

  RCLCPP_INFO(
    node_.get_logger(), "navigating to node '%s' at (%.2f, %.2f), heading %.2f rad",
    target_->node_id.c_str(), position.x, position.y, yaw);

  const std::uint64_t generation = ++generation_;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this, generation](const GoalHandle::SharedPtr & handle) {
      if (generation != generation_) {
        return;
      }
      if (!handle) {
        finish(false, "Nav2 rejected the goal");
      } else {
        goal_handle_ = handle;
      }
    };
  options.result_callback =
    [this, generation](const GoalHandle::WrappedResult & result) {
      if (generation != generation_) {
        return;
      }
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          finish(true, "");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          finish(false, "goal cancelled");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          finish(false, "Nav2 aborted the goal");
          break;
        default:
          finish(false, "Nav2 returned an unknown result");
          break;
      }
    };
  client_->async_send_goal(goal, options);
}

void Nav2NavigationAdapter::cancel()
{
  if (goal_handle_) {
    // The CANCELED result callback reports back once Nav2 has stopped the robot.
    client_->async_cancel_goal(goal_handle_);
    return;
  }
  // No accepted goal (paused, or the goal request is still in flight): there
  // is nothing whose result we could wait for, so invalidate and report now.
  ++generation_;
  client_->async_cancel_all_goals();
  finish(false, "goal cancelled");
}

void Nav2NavigationAdapter::pause()
{
  // NavigateToPose has no pause. Cancel the goal but keep the target, and
  // bump the generation so the resulting CANCELED is not reported as a failure.
  ++generation_;
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  } else {
    client_->async_cancel_all_goals();  // covers a goal request still in flight
  }
}

void Nav2NavigationAdapter::resume()
{
  if (target_) {
    send_goal();
  }
}

void Nav2NavigationAdapter::finish(bool success, const std::string & reason)
{
  if (success && target_ && target_->node_position) {
    previous_position_ = target_->node_position;
  }
  target_.reset();
  goal_handle_.reset();
  on_result_(success, reason);
}

}  // namespace vda5050_nav2_adapter
