// Copyright 2026 vda5050_bridge contributors
// SPDX-License-Identifier: Apache-2.0

#include <exception>
#include <memory>

#include "fault_manager/fault_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<fault_manager::FaultManagerNode> node;
  try {
    node = std::make_shared<fault_manager::FaultManagerNode>();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fault_manager"), "cannot start: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  // Single-threaded on purpose: see the threading note in fault_manager_node.hpp.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
