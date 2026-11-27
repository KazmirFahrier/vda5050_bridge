// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_nav2_adapter/adapter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vda5050_nav2_adapter::AdapterNode>();

  // Single-threaded on purpose: see the threading note in adapter_node.hpp.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  if (node->get_parameter("autostart").as_bool()) {
    using lifecycle_msgs::msg::State;
    if (node->configure().id() != State::PRIMARY_STATE_INACTIVE ||
      node->activate().id() != State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_FATAL(node->get_logger(), "autostart failed; see errors above");
      rclcpp::shutdown();
      return 1;
    }
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
