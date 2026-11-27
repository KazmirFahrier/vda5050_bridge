// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0
//
// ActionPort that delegates VDA 5050 actions to other ROS 2 nodes over topics.
//
// The bridge cannot know how a particular robot picks, drops or charges. It
// publishes each action as JSON and waits for a status report, so any node
// (a lift controller, a charger manager) can implement actions without the
// bridge being recompiled.
//
//   ~/action_request  (std_msgs/String, out)
//       {"actionId":"a1","actionType":"pick","actionParameters":{"height":0.2}}
//   ~/action_cancel   (std_msgs/String, out)   the actionId to stop
//   ~/action_status   (std_msgs/String, in)
//       {"actionId":"a1","status":"FINISHED","resultDescription":"ok"}
//       status is one of RUNNING, PAUSED, FINISHED, FAILED

#ifndef VDA5050_NAV2_ADAPTER__TOPIC_ACTION_HANDLER_HPP_
#define VDA5050_NAV2_ADAPTER__TOPIC_ACTION_HANDLER_HPP_

#include <functional>
#include <set>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "vda5050_core/ports.hpp"

namespace vda5050_nav2_adapter
{

class TopicActionHandler : public vda5050_core::ActionPort
{
public:
  using StatusCallback = std::function<void (
        const std::string & action_id, vda5050_core::ActionStatus status,
        const std::string & result)>;

  TopicActionHandler(
    rclcpp_lifecycle::LifecycleNode & node, const std::vector<std::string> & supported_types,
    StatusCallback on_status);

  bool supports(const std::string & action_type) const override;
  void start(const vda5050_core::Action & action) override;
  void cancel(const std::string & action_id) override;

private:
  void on_status_message(const std_msgs::msg::String & message);

  rclcpp_lifecycle::LifecycleNode & node_;
  std::set<std::string> supported_types_;
  StatusCallback on_status_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr request_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr cancel_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
};

}  // namespace vda5050_nav2_adapter

#endif  // VDA5050_NAV2_ADAPTER__TOPIC_ACTION_HANDLER_HPP_
