// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// NavigationPort implemented with Nav2's NavigateToPose action.

#ifndef VDA5050_NAV2_ADAPTER__NAV2_NAVIGATION_ADAPTER_HPP_
#define VDA5050_NAV2_ADAPTER__NAV2_NAVIGATION_ADAPTER_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "vda5050_core/ports.hpp"

namespace vda5050_nav2_adapter
{

class Nav2NavigationAdapter : public vda5050_core::NavigationPort
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  /// Called exactly once per navigate_to(): success, or failure with a reason.
  using ResultCallback = std::function<void (bool success, const std::string & reason)>;

  Nav2NavigationAdapter(
    rclcpp_lifecycle::LifecycleNode & node, const std::string & action_name,
    const std::string & global_frame, ResultCallback on_result);

  void navigate_to(const vda5050_core::Node & target, const vda5050_core::Edge & via) override;
  void cancel() override;
  void pause() override;
  void resume() override;

private:
  void send_goal();
  void finish(bool success, const std::string & reason);

  rclcpp_lifecycle::LifecycleNode & node_;
  std::string global_frame_;
  ResultCallback on_result_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  std::optional<vda5050_core::Node> target_;
  std::optional<vda5050_core::NodePosition> previous_position_;
  GoalHandle::SharedPtr goal_handle_;

  // Every goal sent gets a new generation number. Callbacks capture theirs and
  // do nothing if it is no longer current, so a result from a goal that was
  // replaced or paused can never be mistaken for the result of the live one.
  std::uint64_t generation_{0};
};

}  // namespace vda5050_nav2_adapter

#endif  // VDA5050_NAV2_ADAPTER__NAV2_NAVIGATION_ADAPTER_HPP_
