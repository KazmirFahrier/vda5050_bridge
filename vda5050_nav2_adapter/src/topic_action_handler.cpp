// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include "vda5050_nav2_adapter/topic_action_handler.hpp"

#include <optional>
#include <utility>

#include "vda5050_core/json.hpp"

namespace vda5050_nav2_adapter
{

namespace
{

std::optional<vda5050_core::ActionStatus> parse_status(const std::string & text)
{
  using vda5050_core::ActionStatus;
  if (text == "RUNNING") {return ActionStatus::Running;}
  if (text == "PAUSED") {return ActionStatus::Paused;}
  if (text == "FINISHED") {return ActionStatus::Finished;}
  if (text == "FAILED") {return ActionStatus::Failed;}
  return std::nullopt;
}

}  // namespace

TopicActionHandler::TopicActionHandler(
  rclcpp_lifecycle::LifecycleNode & node, const std::vector<std::string> & supported_types,
  StatusCallback on_status)
: node_(node),
  supported_types_(supported_types.begin(), supported_types.end()),
  on_status_(std::move(on_status))
{
  // Reliable + keep-all-ish depth: an action request must not be dropped.
  const rclcpp::QoS qos = rclcpp::QoS(10).reliable();
  request_publisher_ = node_.create_publisher<std_msgs::msg::String>("~/action_request", qos);
  cancel_publisher_ = node_.create_publisher<std_msgs::msg::String>("~/action_cancel", qos);
  status_subscription_ = node_.create_subscription<std_msgs::msg::String>(
    "~/action_status", qos,
    [this](const std_msgs::msg::String & message) {on_status_message(message);});
}

bool TopicActionHandler::supports(const std::string & action_type) const
{
  return supported_types_.count(action_type) > 0;
}

void TopicActionHandler::start(const vda5050_core::Action & action)
{
  vda5050_core::Json request;
  request["actionId"] = action.action_id;
  request["actionType"] = action.action_type;
  vda5050_core::Json parameters{vda5050_core::Json::Object{}};
  for (const vda5050_core::ActionParameter & parameter : action.action_parameters) {
    parameters[parameter.key] = parameter.value;
  }
  request["actionParameters"] = std::move(parameters);

  std_msgs::msg::String message;
  message.data = request.dump();
  request_publisher_->publish(message);
}

void TopicActionHandler::cancel(const std::string & action_id)
{
  std_msgs::msg::String message;
  message.data = action_id;
  cancel_publisher_->publish(message);
}

void TopicActionHandler::on_status_message(const std_msgs::msg::String & message)
{
  // This topic is internal to the robot, but it is still input: parse defensively.
  try {
    const vda5050_core::Json json = vda5050_core::Json::parse(message.data);
    const vda5050_core::Json * action_id = json.find("actionId");
    const vda5050_core::Json * status = json.find("status");
    if (action_id == nullptr || !action_id->is_string() || status == nullptr ||
      !status->is_string())
    {
      RCLCPP_WARN(node_.get_logger(), "action_status without string actionId/status ignored");
      return;
    }
    const auto parsed = parse_status(status->as_string());
    if (!parsed) {
      RCLCPP_WARN(
        node_.get_logger(), "action_status with unknown status '%s' ignored",
        status->as_string().c_str());
      return;
    }
    const vda5050_core::Json * result = json.find("resultDescription");
    on_status_(
      action_id->as_string(), *parsed,
      (result != nullptr && result->is_string()) ? result->as_string() : std::string());
  } catch (const vda5050_core::JsonParseError & error) {
    RCLCPP_WARN(node_.get_logger(), "action_status is not valid JSON: %s", error.what());
  }
}

}  // namespace vda5050_nav2_adapter
